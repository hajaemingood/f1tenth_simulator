#!/bin/bash
# vendor.sh : submodule/중첩 git 정리 + .gitignore 예외 + add/commit/push 자동화
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "사용법: $0 <패키지경로1> <패키지경로2> ..."
  echo "예시:  $0 src/robot_localization src/slam_toolbox"
  exit 1
fi

# 0) 레포 루트 보정
repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "${repo_root}" ]]; then
  echo "이 디렉토리는 Git 저장소가 아닙니다. 레포 루트에서 실행하세요."
  exit 1
fi
cd "$repo_root"

echo "Repo root: $repo_root"
echo "Packages : $*"

# 1) 대상 패키지 처리
for PKG in "$@"; do
  echo ">>> Processing $PKG"

  if [[ ! -d "$PKG" ]]; then
    echo "    경고: $PKG 디렉토리가 없음. 건너뜀."
    continue
  fi

  # 서브모듈 해제
  git submodule deinit -f "$PKG" 2>/dev/null || true

  # .gitmodules 정리
  if [[ -f .gitmodules ]]; then
    git config -f .gitmodules --remove-section "submodule.$PKG" 2>/dev/null || true
  fi

  # 인덱스에서 서브모듈 포인터 제거 (실제 파일은 남김)
  git rm --cached "$PKG" 2>/dev/null || true

  # 서브모듈 메타데이터 제거
  rm -rf ".git/modules/$PKG" 2>/dev/null || true

  # 폴더 내부 중첩 .git 제거
  if [[ -e "$PKG/.git" ]]; then
    rm -rf "$PKG/.git"
    echo "    removed nested $PKG/.git"
  fi

  # .gitignore 예외 규칙 보강
  touch .gitignore
  rule="!${PKG}/**"
  if ! grep -qxF "$rule" .gitignore; then
    echo "$rule" >> .gitignore
    echo "    added ignore exception: $rule"
    git add .gitignore
  fi

  # 강제로 add
  git add -f "$PKG"
done

# .gitmodules가 비었으면 제거
if [[ -f .gitmodules && ! -s .gitmodules ]]; then
  git rm --cached .gitmodules 2>/dev/null || true
  rm -f .gitmodules
  echo "    removed empty .gitmodules"
fi

# 변경 여부 확인 후 커밋
if git diff --cached --quiet; then
  echo "스테이지된 변경이 없습니다. 이미 정리됐거나 .gitignore가 다른 규칙으로 막고 있을 수 있습니다."
  echo "필요 시 확인: git check-ignore -v -n <파일경로>"
  exit 0
fi

branch="$(git rev-parse --abbrev-ref HEAD)"
git commit -m "Vendor packages: $*"
git push origin "$branch"

echo "완료: GitHub에서 $* 가 일반 폴더로 보여야 합니다."
