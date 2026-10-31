#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
if [[ ! -x .venv/bin/python ]] || [[ ! -f third_party/unitree_mujoco/unitree_robots/go2/scene.xml ]]; then
  bash scripts/setup_mujoco.sh
fi
.venv/bin/python -m pip install -r requirements-go2-control.txt
.venv/bin/python scripts/fetch_go2_policy.py
.venv/bin/python scripts/verify_go2_controller.py
.venv/bin/python scripts/verify_go2_terrain.py
