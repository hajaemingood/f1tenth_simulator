#!/bin/bash
set -e

for PKG in src/robot_localization src/slam_toolbox; do
  echo ">>> Processing $PKG"

  # 1) 서브모듈 해제
  git submodule deinit -f "$PKG" 2>/dev/null || true

  # 2) .gitmodules 정리
  if [ -f .gitmodules ]; then
    git config -f .gitmodules --remove-section "submodule.$PKG" 2>/dev/null || true
  fi

  # 3) 서브모듈 포인터만 제거 (실제 파일은 남김)
  git rm --cached "$PKG" 2>/dev/null || true

  # 4) 서브모듈 메타데이터 삭제
  rm -rf ".git/modules/$PKG"

  # 5) 중첩 .git 제거
  if [ -e "$PKG/.git" ]; then
    rm -rf "$PKG/.git"
  fi
done

# .gitmodules 비면 제거
if [ -f .gitmodules ] && [ ! -s .gitmodules ]; then
  git rm --cached .gitmodules 2>/dev/null || true
  rm -f .gitmodules
fi

echo ">>> Done. Now add the packages back with: git add src/robot_localization src/slam_toolbox"
