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
YAML_PATH = "/root/f1_sim/src/ferrari/maps/shortcut.yaml"     # 예: "columbia.yaml"
PGM_PATH = "/root/f1_sim/src/ferrari/maps/shortcut.pgm"       # 예: "columbia.pgm"
OUTPUT_PREFIX = "/root/f1_sim/src/path_planner/path_planner/waypoints"      # 저장 prefix

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
    img = cv2.flip(img, 0)   # 0이면 y축(상하) 반전

    res = float(info.get("resolution", 0.05))
    origin = np.array(info.get("origin", [0.0, 0.0, 0.0]), dtype=float)
    negate = int(info.get("negate", 0))
    return img, res, origin, negate


def close_loop(points_xy, tol=0.5):
    pts = np.array(points_xy, float)
    if pts.shape[0] < 3:
        return pts
    d = np.linalg.norm(pts[0] - pts[-1])
    if d <= tol:
        pts = np.vstack([pts, pts[0]])
    return pts


def resample_equal_arc(points_xy, spacing=0.2, auto_close=True, close_tol=0.5):
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
    fx = interpolate.interp1d(s, pts[:, 0], kind="linear")
    fy = interpolate.interp1d(s, pts[:, 1], kind="linear")
    return np.stack([fx(s_new), fy(s_new)], axis=1)


def smooth_spline_resample(points_xy, spacing=None, n_points=None,
                           smooth=0.001, auto_close=True, close_tol=0.5):
    pts = np.asarray(points_xy, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        return pts

    is_close = np.linalg.norm(pts[0] - pts[-1]) <= close_tol
    per = bool(auto_close and is_close)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(t[-1])
    if L < 1e-9:
        return pts

    tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1]],
                                 u=t, s=smooth * len(pts), per=per, k=3)

    if n_points is not None:
        n_new = int(max(2, n_points))
    elif spacing is not None:
        n_new = max(2, int(round(L / float(spacing))))
    else:
        n_new = len(pts)

    if per:
        t_new = np.linspace(0.0, L, n_new, endpoint=False)
    else:
        t_new = np.linspace(0.0, L, n_new, endpoint=True)

    x_new, y_new = interpolate.splev(t_new, tck)
    return np.stack([x_new, y_new], axis=1)


LAYER_ORDER = ["center", "inner", "outer"]
LAYER_COLOR = {"center": "tab:blue", "inner": "tab:orange", "outer": "tab:green"}


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
            "center": np.array(init_center if init_center is not None else np.zeros((0, 2)), float),
            "inner": np.array(init_inner if init_inner is not None else np.zeros((0, 2)), float),
            "outer": np.array(init_outer if init_outer is not None else np.zeros((0, 2)), float),
        }
        self.active = "center"
        self.history = []
        self.drag_idx = None

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._draw()

        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_move = self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        plt.title("Layered Waypoint Editor [Active: center]\n"
                  "Left=Add/Drag  Right=Delete | [1]=center [2]=inner [3]=outer\n"
                  "[r]=Resample  [z]=Smooth+Resample \n"
                  "[s]=Save all  [q]=Quit")
        plt.show(block=True)

    def _draw(self):
        self.ax.clear()
        self.ax.imshow(self.bg, cmap='gray', origin='lower', extent=self.extent, alpha=0.6)
        for name in LAYER_ORDER:
            pts = self.points[name]
            if len(pts) > 0:
                loop = np.vstack([pts, pts[0]]) if len(pts) >= 3 else pts
                self.ax.plot(loop[:, 0], loop[:, 1], '-', lw=2, color=LAYER_COLOR[name], alpha=0.9)
                self.ax.scatter(pts[:, 0], pts[:, 1], s=16, color=LAYER_COLOR[name], label=name)
        self.ax.legend(loc="upper right")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.fig.canvas.draw_idle()

    def _push_history(self):
        self.history.append({k: v.copy() for k, v in self.points.items()})
        if len(self.history) > 60:
            self.history.pop(0)

    def _nearest_point_index(self, layer, x, y):
        pts = self.points[layer]
        if len(pts) == 0:
            return None, np.inf
        d2 = np.sum((pts - np.array([x, y]))**2, axis=1)
        idx = int(np.argmin(d2))
        return idx, float(np.sqrt(d2[idx]))

    def _nearest_segment_index(self, layer, x, y):
        pts = self.points[layer]
        if len(pts) < 2:
            return None, np.inf, None
        P = np.array([x, y])
        best_i, best_dist, best_proj = None, np.inf, None
        for i in range(len(pts)):
            A = pts[i]
            B = pts[(i + 1) % len(pts)]
            AB = B - A
            t = np.dot(P - A, AB) / (np.dot(AB, AB) + 1e-12)
            t = np.clip(t, 0.0, 1.0)
            proj = A + t * AB
            dist = np.linalg.norm(P - proj)
            if dist < best_dist:
                best_dist, best_i, best_proj = dist, i, proj
        return best_i, best_dist, best_proj

    def on_click(self, event):
        if not event.inaxes:
            return
        x, y = float(event.xdata), float(event.ydata)
        layer = self.active
        if event.button == 1:
            idx, d = self._nearest_point_index(layer, x, y)
            if d < self.spacing * 0.8:
                self.drag_idx = idx
            else:
                seg_i, _, _ = self._nearest_segment_index(layer, x, y)
                self._push_history()
                if seg_i is None or len(self.points[layer]) == 0:
                    if len(self.points[layer]) > 0:
                        self.points[layer] = np.vstack([self.points[layer], [x, y]])
                    else:
                        self.points[layer] = np.array([[x, y]])
                else:
                    insert_i = (seg_i + 1) % max(1, len(self.points[layer]))
                    self.points[layer] = np.insert(self.points[layer], insert_i, [x, y], axis=0)
                self._draw()
        elif event.button == 3:
            idx, d = self._nearest_point_index(layer, x, y)
            if idx is not None and d < self.spacing * 1.2 and len(self.points[layer]) > 1:
                self._push_history()
                self.points[layer] = np.delete(self.points[layer], idx, axis=0)
                self._draw()

    def on_move(self, event):
        if self.drag_idx is None or not event.inaxes:
            return
        x, y = float(event.xdata), float(event.ydata)
        self.points[self.active][self.drag_idx] = [x, y]
        self._draw()

    def on_release(self, event):
        if self.drag_idx is not None:
            self._push_history()
        self.drag_idx = None

    def on_key(self, event):
        if event.key in ['1', '2', '3']:
            self.active = LAYER_ORDER[int(event.key) - 1]
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
        elif event.key == 's':
            self.save_all()
        elif event.key == 'q':
            plt.close(self.fig)

    def save_all(self):
        c = np.array(self.points["center"], float)
        i = np.array(self.points["inner"], float)
        o = np.array(self.points["outer"], float)

        lens = [len(c), len(i), len(o)]
        if max(lens) < 3:
            print("[WARN] 유효한 레이어가 부족합니다(3점 미만). NaN 패딩으로 저장합니다.")
            N = max(lens) if max(lens) > 0 else 1

            def pad(arr, n):
                if len(arr) == 0:
                    return np.full((n, 2), np.nan)
                if len(arr) < n:
                    return np.vstack([arr, np.full((n - len(arr), 2), np.nan)])
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
                    return np.full((N, 2), np.nan)
            cN = resample_or_nan(c, N)
            iN = resample_or_nan(i, N)
            oN = resample_or_nan(o, N)

        combined = np.hstack([cN, iN, oN])

        if combined.shape[0] > 1:
            combined = np.vstack([combined, combined[0]])

        np.save(f"{self.save_prefix}.npy", combined)
        print(f"[SAVE] {self.save_prefix}.npy  shape={combined.shape}")

        pd.DataFrame(combined, columns=["cx", "cy", "ix", "iy", "ox", "oy"]).to_csv(
            f"{self.save_prefix}.csv", index=False
        )
        print(f"[SAVE] {self.save_prefix}.csv")


def main():
    img, res, origin, negate = load_map_and_meta(YAML_PATH, PGM_PATH)
    print(origin)
    H, W = img.shape
    print(H, W)

    extent = [origin[0], origin[0] + W * res, origin[1], origin[1] + H * res]
    print(extent)

    LayeredEditor(
        background_img=img,
        extent_xy=extent,
        init_center=None,
        init_inner=None,
        init_outer=None,
        spacing=TARGET_SPACING_M,
        smooth=SMOOTHNESS,
        resample_equal_arc=resample_equal_arc,
        smooth_spline_resample=smooth_spline_resample,
        save_prefix=OUTPUT_PREFIX
    )


if __name__ == "__main__":
    main()
