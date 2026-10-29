#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$PROJECT_ROOT/scripts/run_mujoco.sh" \
  --model "$PROJECT_ROOT/scenes/ruins_rescue/scene.xml" --camera overview "$@"
