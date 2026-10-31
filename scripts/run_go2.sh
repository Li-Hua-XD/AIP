#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
  echo "缺少 .venv，请先运行 bash scripts/setup_mujoco.sh" >&2
  exit 1
fi
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-mesa}"
for arg in "$@"; do
  if [[ "$arg" == "--headless" ]]; then
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
    if [[ -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ]]; then
      export __EGL_VENDOR_LIBRARY_FILENAMES="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/50_mesa.json}"
    fi
  fi
done
if [[ "${AIP_SOFTWARE_RENDERING:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi
exec "$PROJECT_ROOT/.venv/bin/python" "$PROJECT_ROOT/scripts/go2_task.py" "$@"
