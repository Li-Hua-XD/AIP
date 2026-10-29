#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
UPSTREAM_COMMIT=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
MODEL_DIR="$PROJECT_ROOT/third_party/unitree_mujoco"
python3 -m venv --without-pip .venv
if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
  BOOTSTRAP_FILE="$(mktemp /tmp/aip-get-pip.XXXXXX.py)"
  trap 'rm -f "$BOOTSTRAP_FILE"' EXIT
  curl --fail --location --retry 3 https://bootstrap.pypa.io/get-pip.py -o "$BOOTSTRAP_FILE"
  .venv/bin/python "$BOOTSTRAP_FILE"
fi
if [[ -f requirements-mujoco.lock.txt ]]; then
  .venv/bin/python -m pip install -r requirements-mujoco.lock.txt
else
  .venv/bin/python -m pip install -r requirements-mujoco.txt
fi
if [[ ! -e "$MODEL_DIR" ]]; then
  mkdir -p "$MODEL_DIR"
  git -C "$MODEL_DIR" init
  git -C "$MODEL_DIR" remote add origin https://github.com/unitreerobotics/unitree_mujoco.git
  git -C "$MODEL_DIR" sparse-checkout init --cone
  git -C "$MODEL_DIR" sparse-checkout set unitree_robots/go2 simulate_python example/python
  git -C "$MODEL_DIR" fetch --depth 1 --filter=blob:none origin "$UPSTREAM_COMMIT"
  git -C "$MODEL_DIR" checkout --detach FETCH_HEAD
elif [[ "$(git -C "$MODEL_DIR" rev-parse HEAD)" != "$UPSTREAM_COMMIT" ]]; then
  echo "已有上游模型版本与锁定提交不同，请检查 $MODEL_DIR，脚本不会覆盖。" >&2
  exit 1
fi
bash scripts/run_mujoco.sh --headless --seconds 10
