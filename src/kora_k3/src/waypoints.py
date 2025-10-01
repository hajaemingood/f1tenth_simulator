# -*- coding: utf-8 -*-

import numpy as np
import cv2
import yaml
import networkx as nx
from skimage.morphology import medial_axis, remove_small_objects
from scipy import interpolate
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

# ================= 사용자 설정 =================
YAML_PATH = "columbia.yaml"     # 예: "columbia.yaml"
PGM_PATH  = "columbia.pgm"      # 예: "columbia.pgm"
OUTPUT_PREFIX = "outputs/waypoints"  # 저장 prefix

# 맵 극성/이진화
FREE_IS_WHITE = True       # 흰색=주행가능이면 True (ROS 기본)
BIN_THRESH = 200           # 이진화 임계(180~230 권장 튜닝)

# 스켈레톤/그래프 파라미터
APPLY_MORPH = True         # skeleton 전 free mask에 open/close 적용
MORPH_KERNEL_SIZE = 3      # 3 또는 5 추천
MORPH_OPEN_ITERS = 1
MORPH_CLOSE_ITERS = 1

MIN_SKELETON_COMPONENT_PIXELS = 200   # 너무 작은 성분 제거
MAX_SPUR_PRUNE_ITER = 80              # 차수1 가지 반복 제거

# 에디팅/리샘플/스무딩
TARGET_SPACING_M = 0.20    # 균일 간격 (Pure Pursuit 0.1~0.3 권장)
SMOOTHNESS = 0.001         # splprep smoothing (0=interpolate, ↑부드러움↑/원본과 차이↑)
# =================================================


def load_map_and_meta(yaml_path, pgm_path):
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)
    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot read {pgm_path}")
    res = float(info.get("resolution", 0.05))
    origin = np.array(info.get("origin", [0.0, 0.0, 0.0]), dtype=float)
    negate = int(info.get("negate", 0))
    return img, res, origin, negate

def to_binary_free(img, free_is_white=True, bin_thresh=200):
    """주행가능=1, 벽=0 이진화"""
    if free_is_white:
        _, b = cv2.threshold(img, bin_thresh, 255, cv2.THRESH_BINARY)
    else:
        _, b = cv2.threshold(img, bin_thresh, 255, cv2.THRESH_BINARY_INV)
    return (b > 0).astype(np.uint8)

def largest_connected_component(mask):
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    sizes = stats[1:, cv2.CC_STAT_AREA]
    largest_idx = 1 + np.argmax(sizes)
    return (labels == largest_idx).astype(np.uint8)

def skeleton_from_free_space(free_mask):
    free_largest = largest_connected_component(free_mask)
    skel, _ = medial_axis(free_largest.astype(bool), return_distance=True)
    skel = skel.astype(np.uint8)
    skel = remove_small_objects(skel.astype(bool), MIN_SKELETON_COMPONENT_PIXELS).astype(np.uint8)
    return skel

def skeleton_to_graph(skel):
    """픽셀 스켈레톤 -> undirected graph(4-이웃)"""
    H, W = skel.shape
    G = nx.Graph()
    idx = np.argwhere(skel > 0)
    # 노드
    for y, x in idx:
        G.add_node((int(y), int(x)))
    # 엣지(4-이웃)
    dirs = [(1,0),(-1,0),(0,1),(0,-1)]
    for y, x in idx:
        for dy, dx in dirs:
            ny, nx_ = y+dy, x+dx
            if 0 <= ny < H and 0 <= nx_ < W and skel[ny, nx_] > 0:
                G.add_edge((int(y), int(x)), (int(ny), int(nx_)))
    return G

def prune_spurs(G, max_iter=50):
    """차수1 말단 반복 제거"""
    G = G.copy()
    for _ in range(max_iter):
        leaves = [n for n in G.nodes if G.degree[n] == 1]
        if not leaves:
            break
        G.remove_nodes_from(leaves)
    return G

def largest_subgraph(G):
    if G.number_of_nodes() == 0:
        return G
    comps = sorted(nx.connected_components(G), key=len, reverse=True)
    return G.subgraph(comps[0]).copy()

def order_loop_nodes_simple(G):
    """차수≈2의 단일 루프 가정 시, 이웃 따라 순회"""
    if G.number_of_nodes() == 0:
        return []
    start = next(iter(G.nodes))
    path = [start]
    nbrs = list(G.neighbors(start))
    if len(nbrs) == 0:
        return [start]
    prev = start
    curr = nbrs[0]
    path.append(curr)
    while True:
        nbrs = [n for n in G.neighbors(curr) if n != prev]
        if not nbrs:
            break
        nxt = nbrs[0]
        if nxt == path[0]:
            break
        path.append(nxt)
        prev, curr = curr, nxt
        if len(path) > G.number_of_nodes() + 5:
            break
    return path

def pick_longest_cycle(G):
    """(A) cycle_basis로 가장 긴 사이클 선택"""
    cycles = nx.cycle_basis(G)
    if not cycles:
        return []
    cycles_sorted = sorted(cycles, key=len, reverse=True)
    return cycles_sorted[0]

def longest_simple_path_heuristic(G):
    """(B) 루프가 없을 때: 최장 단순 경로 휴리스틱"""
    if G.number_of_nodes() == 0:
        return []
    leaves = [n for n in G.nodes if G.degree[n] == 1]
    if len(leaves) < 2:
        # 임의 시작→가장 먼 u, u→가장 먼 v
        start = next(iter(G.nodes))
        lengths = nx.single_source_shortest_path_length(G, start)
        u = max(lengths, key=lengths.get)
        lengths2 = nx.single_source_shortest_path_length(G, u)
        v = max(lengths2, key=lengths2.get)
        try:
            return nx.shortest_path(G, u, v)
        except:
            return []
    best_path = []
    for i in range(len(leaves)):
        for j in range(i+1, len(leaves)):
            u, v = leaves[i], leaves[j]
            try:
                p = nx.shortest_path(G, u, v)
                if len(p) > len(best_path):
                    best_path = p
            except:
                pass
    return best_path

def pix_to_world(loop_pixels, res, origin, H):
    """(C) 빈/형식 이상 입력도 안전하게 처리"""
    if loop_pixels is None:
        return np.zeros((0,2), dtype=float)
    loop_pixels = list(loop_pixels)
    if len(loop_pixels) == 0:
        return np.zeros((0,2), dtype=float)

    arr = np.array(loop_pixels, dtype=object)
    # (y,x) 튜플 형태 강제
    try:
        arr = np.array([(int(p[0]), int(p[1])) for p in loop_pixels], dtype=int)
    except Exception as e:
        print("[ERROR] pix_to_world: 입력 노드 형식 오류:", e)
        return np.zeros((0,2), dtype=float)

    if arr.ndim != 2 or arr.shape[1] != 2:
        return np.zeros((0,2), dtype=float)

    py = arr[:,0]
    px = arr[:,1]
    wx = origin[0] + px * res
    wy = origin[1] + (H - py) * res   # 이미지 행(y)은 위에서 아래로, 월드 y는 아래에서 위로
    return np.stack([wx, wy], axis=1)

def close_loop(points_xy, tol=0.5):
    """시작/끝이 tol 이하로 가까우면 닫고, 아니면 열린 경로 유지"""
    pts = np.array(points_xy, float)
    if pts.shape[0] < 3:
        return pts
    d = np.linalg.norm(pts[0] - pts[-1])
    if d <= tol:
        pts = np.vstack([pts, pts[0]])
    return pts

def resample_equal_arc(points_xy, spacing=0.2, auto_close=True, close_tol=0.5):
    """균일 호길이 리샘플, 필요시 자동 루프 닫기"""
    pts = np.array(points_xy, float)
    if auto_close:
        pts = close_loop(pts, tol=close_tol)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    L = s[-1]
    if L < 1e-6:
        return pts
    n_new = max(4, int(np.round(L / spacing)))
    s_new = np.linspace(0, L, n_new, endpoint=False)
    fx = interpolate.interp1d(s, pts[:,0], kind="linear")
    fy = interpolate.interp1d(s, pts[:,1], kind="linear")
    return np.stack([fx(s_new), fy(s_new)], axis=1)

def smooth_spline_resample(points_xy, spacing=None, n_points=None,
                           smooth=0.001, auto_close=True, close_tol=0.5):
    """
    B-spline 스무딩 후 리샘플.
    - spacing: 간격 기반 (m)
    - n_points: 개수 기반 (정수)
    둘 중 하나 지정. 둘 다 None이면 원래 개수 유지.

    auto_close=True이면 시작/끝이 close_tol 이내면 주기(per=True)로 처리.
    """
    pts = np.asarray(points_xy, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        return pts

    # 폐곡선 여부 판단
    is_close = np.linalg.norm(pts[0] - pts[-1]) <= close_tol
    per = bool(auto_close and is_close)

    # 호길이 파라미터화
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(t[-1])
    if L < 1e-9:
        return pts

    # 스플라인 적합
    tck, _ = interpolate.splprep([pts[:,0], pts[:,1]],
                                 u=t, s=smooth*len(pts), per=per, k=3)

    # 샘플 개수 결정
    if n_points is not None:
        n_new = int(max(2, n_points))
    elif spacing is not None:
        n_new = max(2, int(round(L / float(spacing))))
    else:
        n_new = len(pts)

    # 파라미터 샘플
    if per:
        t_new = np.linspace(0.0, L, n_new, endpoint=False)  # 주기: 끝점 중복 방지
    else:
        t_new = np.linspace(0.0, L, n_new, endpoint=True)   # 비주기: 끝점 포함

    x_new, y_new = interpolate.splev(t_new, tck)
    return np.stack([x_new, y_new], axis=1)


# ---------- 파이프라인 시작 ----------
img, RES, ORIGIN, NEGATE = load_map_and_meta(YAML_PATH, PGM_PATH)
H, W = img.shape

# 이진화
free = to_binary_free(img, free_is_white=FREE_IS_WHITE, bin_thresh=BIN_THRESH)

# morphology(옵션): 끊김 보정
if APPLY_MORPH:
    k = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    if MORPH_OPEN_ITERS > 0:
        free = cv2.morphologyEx(free, cv2.MORPH_OPEN, k, iterations=MORPH_OPEN_ITERS)
    if MORPH_CLOSE_ITERS > 0:
        free = cv2.morphologyEx(free, cv2.MORPH_CLOSE, k, iterations=MORPH_CLOSE_ITERS)

# 스켈레톤 -> 그래프 -> 가지치기
skel = skeleton_from_free_space(free)
G0 = skeleton_to_graph(skel)
G1 = largest_subgraph(G0)
G2 = prune_spurs(G1, max_iter=MAX_SPUR_PRUNE_ITER)
G2 = largest_subgraph(G2)

# 루프/대안 경로 추출
loop_pix = order_loop_nodes_simple(G2)
if len(loop_pix) < 4:
    print("[WARN] 단순 순회로 루프 추정 실패. cycle_basis 시도.")
    cycle_nodes = pick_longest_cycle(G2)
    if len(cycle_nodes) >= 4:
        loop_pix = cycle_nodes
        print("[INFO] cycle_basis로 루프 복구 성공. 길이:", len(loop_pix))
    else:
        print("[WARN] cycle_basis 실패. 최장 경로 휴리스틱 시도.")
        lp = longest_simple_path_heuristic(G2)
        if len(lp) >= 4:
            loop_pix = lp
            print("[INFO] 최장 경로로 대체. 길이:", len(loop_pix))
        else:
            print("[ERROR] 경로 추출 실패. 수동 작성부터 시작하세요(빈 에디터로 진입).")

# 픽셀 -> 월드 좌표
waypoints_xy = pix_to_world(loop_pix, RES, ORIGIN, H)  # (C) 가드로 안전


# ----- 추가: 멀티 레이어 편집기 (center/inner/outer) -----

LAYER_ORDER = ["center", "inner", "outer"]
LAYER_COLOR = {"center":"tab:blue", "inner":"tab:orange", "outer":"tab:green"}

class LayeredEditor:
    def __init__(self, background_img, extent_xy, init_center=None, init_inner=None, init_outer=None,
                 spacing=0.2, smooth=0.001,
                 resample_equal_arc=None, smooth_spline_resample=None,
                 save_prefix="/home/tony/f1tenth/src/f1tenth_simulator/scripts/outputs"):
        self.bg = background_img
        self.extent = extent_xy
        self.spacing = spacing
        self.smooth = smooth
        self.resample_equal_arc = resample_equal_arc
        self.smooth_spline_resample = smooth_spline_resample
        self.save_prefix = save_prefix

        self.points = {
            "center": np.array(init_center if init_center is not None else np.zeros((0,2)), float),
            "inner":  np.array(init_inner  if init_inner  is not None else np.zeros((0,2)), float),
            "outer":  np.array(init_outer  if init_outer  is not None else np.zeros((0,2)), float),
        }
        self.active = "center"
        self.history = []
        self.drag_idx = None

        self.fig, self.ax = plt.subplots(figsize=(8,8))
        self._draw()

        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_move = self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        plt.title("Layered Waypoint Editor [Active: center]\n"
                  "Left=Add/Drag  Right=Delete | [1]=center [2]=inner [3]=outer\n"
                  "[r]=Resample  [z]=Smooth+Resample  [x]=Center from Borders  [w]=Width CSV\n"
                  "[s]=Save all  [q]=Quit")
        plt.show(block=True)

    def _draw(self):
        self.ax.clear()
        self.ax.imshow(self.bg, cmap='gray', origin='lower', extent=self.extent, alpha=0.6)
        for name in LAYER_ORDER:
            pts = self.points[name]
            if len(pts) > 0:
                loop = np.vstack([pts, pts[0]]) if len(pts) >= 3 else pts
                self.ax.plot(loop[:,0], loop[:,1], '-', lw=2, color=LAYER_COLOR[name], alpha=0.9)
                self.ax.scatter(pts[:,0], pts[:,1], s=16, color=LAYER_COLOR[name], label=name)
        self.ax.legend(loc="upper right")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.fig.canvas.draw_idle()

    def _push_history(self):
        self.history.append({k:v.copy() for k,v in self.points.items()})
        if len(self.history) > 60: self.history.pop(0)

    def _nearest_point_index(self, layer, x, y):
        pts = self.points[layer]
        if len(pts) == 0:
            return None, np.inf
        d2 = np.sum((pts - np.array([x,y]))**2, axis=1)
        idx = int(np.argmin(d2))
        return idx, float(np.sqrt(d2[idx]))

    def _nearest_segment_index(self, layer, x, y):
        pts = self.points[layer]
        if len(pts) < 2:
            return None, np.inf, None
        P = np.array([x,y])
        best_i, best_dist, best_proj = None, np.inf, None
        for i in range(len(pts)):
            A = pts[i]; B = pts[(i+1)%len(pts)]
            AB = B - A
            t = np.dot(P - A, AB) / (np.dot(AB, AB) + 1e-12)
            t = np.clip(t, 0.0, 1.0)
            proj = A + t * AB
            dist = np.linalg.norm(P - proj)
            if dist < best_dist:
                best_dist, best_i, best_proj = dist, i, proj
        return best_i, best_dist, best_proj

    def on_click(self, event):
        if not event.inaxes: return
        x, y = float(event.xdata), float(event.ydata)
        layer = self.active
        if event.button == 1:  # add/drag
            idx, d = self._nearest_point_index(layer, x, y)
            if d < self.spacing * 0.8:
                self.drag_idx = idx
            else:
                seg_i, _, _ = self._nearest_segment_index(layer, x, y)
                self._push_history()
                if seg_i is None or len(self.points[layer]) == 0:
                    self.points[layer] = np.vstack([self.points[layer], [x,y]]) if len(self.points[layer])>0 else np.array([[x,y]])
                else:
                    insert_i = (seg_i + 1) % max(1, len(self.points[layer]))
                    self.points[layer] = np.insert(self.points[layer], insert_i, [x,y], axis=0)
                self._draw()
        elif event.button == 3:  # delete
            idx, d = self._nearest_point_index(layer, x, y)
            if idx is not None and d < self.spacing * 1.2 and len(self.points[layer]) > 1:
                self._push_history()
                self.points[layer] = np.delete(self.points[layer], idx, axis=0)
                self._draw()

    def on_move(self, event):
        if self.drag_idx is None or not event.inaxes: return
        x, y = float(event.xdata), float(event.ydata)
        self.points[self.active][self.drag_idx] = [x, y]
        self._draw()

    def on_release(self, event):
        if self.drag_idx is not None:
            self._push_history()
        self.drag_idx = None

    def on_key(self, event):
        if event.key in ['1','2','3']:
            self.active = LAYER_ORDER[int(event.key)-1]
            self._draw()
        elif event.key == 'u':
            if self.history:
                self.points = self.history.pop()
                self._draw()
        elif event.key == 'r':
            lyr = self.active
            if len(self.points[lyr]) >= 3:
                self._push_history()
                self.points[lyr] = self.resample_equal_arc(self.points[lyr], spacing=self.spacing,
                                                           auto_close=True, close_tol=0.5)
                self._draw()
        elif event.key == 'z':
            lyr = self.active
            if len(self.points[lyr]) >= 3:
                self._push_history()
                self.points[lyr] = self.smooth_spline_resample(self.points[lyr],
                                                               spacing=self.spacing,
                                                               smooth=self.smooth,
                                                               auto_close=True, close_tol=0.5)
                self._draw()
        elif event.key == 'x':
            inner = self.points["inner"]; outer = self.points["outer"]
            if len(inner) >= 2 and len(outer) >= 2:
                self._push_history()
                inner_u = self.resample_equal_arc(inner, spacing=self.spacing, auto_close=True)
                outer_u = self.resample_equal_arc(outer, spacing=self.spacing, auto_close=True)
                L = min(len(inner_u), len(outer_u))
                self.points["center"] = 0.5*(inner_u[:L] + outer_u[:L])
                self._draw()
                print("[INFO] center regenerated from inner/outer.")
        elif event.key == 'w':
            center = self.points["center"]; inner = self.points["inner"]; outer = self.points["outer"]
            if len(center)>=2 and len(inner)>=2 and len(outer)>=2:
                widths=[]
                for p in center:
                    di = np.min(np.linalg.norm(inner-p, axis=1))
                    do = np.min(np.linalg.norm(outer-p, axis=1))
                    widths.append(di+do)
                df = pd.DataFrame({"x":center[:,0], "y":center[:,1], "track_width":widths})
                outp = f"{self.save_prefix}_center_with_width.csv"
                df.to_csv(outp, index=False)
                print(f"[SAVE] {outp}")
        elif event.key == 's':
            self.save_all()
        elif event.key == 'q':
            plt.close(self.fig)

    def save_all(self):
        c = np.array(self.points["center"], float)
        i = np.array(self.points["inner"],  float)
        o = np.array(self.points["outer"],  float)

        lens = [len(c), len(i), len(o)]
        if max(lens) < 3:
            print("[WARN] 유효한 레이어가 부족합니다(3점 미만). NaN 패딩으로 저장합니다.")
            N = max(lens) if max(lens) > 0 else 1
            def pad(arr, n):
                if len(arr) == 0: return np.full((n,2), np.nan)
                if len(arr) < n:  return np.vstack([arr, np.full((n-len(arr),2), np.nan)])
                return arr[:n]
            cN, iN, oN = pad(c, N), pad(i, N), pad(o, N)
        else:
            N = max(len(c), len(i), len(o))
            def resample_or_nan(arr, N):
                if arr is not None and len(arr) >= 3:
                    return smooth_spline_resample(arr, n_points=N,
                                                smooth=self.smooth,
                                                auto_close=True, close_tol=0.5)
                else:
                    return np.full((N,2), np.nan)
            cN = resample_or_nan(c, N)
            iN = resample_or_nan(i, N)
            oN = resample_or_nan(o, N)

        # (N,6) 결합
        combined = np.hstack([cN, iN, oN])

        # ✅ 마지막 행에 첫 행을 추가해서 루프 닫기
        if combined.shape[0] > 1:
            combined = np.vstack([combined, combined[0]])

        # 저장
        np.save(f"{self.save_prefix}.npy", combined)
        print(f"[SAVE] {self.save_prefix}.npy  shape={combined.shape}")

        pd.DataFrame(combined, columns=["cx","cy","ix","iy","ox","oy"]).to_csv(
            f"{self.save_prefix}.csv", index=False
        )
        print(f"[SAVE] {self.save_prefix}.csv")




# ----- 멀티 레이어 편집기 끝 -----


# 월드 좌표 extent 계산 (배경 표시용)
extent = [ORIGIN[0], ORIGIN[0] + W*RES, ORIGIN[1], ORIGIN[1] + H*RES]

# 자동 파이프라인이 만든 center가 있으면 초기값으로 전달(없으면 None)
init_center = waypoints_xy if waypoints_xy is not None and len(waypoints_xy)>0 else None

LayeredEditor(
    background_img=img,
    extent_xy=extent,
    init_center=init_center,      # center 초기값 (없으면 수동으로 찍기 시작)
    init_inner=None,              # 있으면 넘겨도 됨
    init_outer=None,              # 있으면 넘겨도 됨
    spacing=TARGET_SPACING_M,
    smooth=SMOOTHNESS,
    resample_equal_arc=resample_equal_arc,            # 기존 함수 그대로 재사용
    smooth_spline_resample=smooth_spline_resample,    # 기존 함수 그대로 재사용
    save_prefix=OUTPUT_PREFIX                         # 저장 파일 prefix
)
