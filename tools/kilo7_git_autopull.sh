#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/opt/kilo7}"
BRANCH="${BRANCH:-main}"

cd "$REPO_DIR"

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || exit 2

if [ -n "$(git status --porcelain)" ]; then
  echo "REFUSE: dirty tree; not pulling"
  exit 3
fi

git fetch origin "$BRANCH"
git checkout "$BRANCH" >/dev/null 2>&1 || true
git pull --ff-only origin "$BRANCH"
