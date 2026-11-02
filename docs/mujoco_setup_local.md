# 本机 MuJoCo 环境记录（Ubuntu 22.04）

记录 2026-11-02 在开发机上的实际安装方式与结果，补充
[MuJoCo 环境说明](mujoco_setup.md) 中“Ubuntu 24.04 + Python 3.12”的基线。
上游文档与脚本未改动，重建时按本文步骤执行。

## 与基线的差异

| 项目 | 基线 | 本机 |
| --- | --- | --- |
| 系统 | Ubuntu 24.04 | Ubuntu 22.04.5 LTS |
| 系统 Python | 3.12 | 3.10.12 |
| sudo | 可用 | 无密码不可用 |
| 图形 | 视机器而定 | X11 `:1` + RTX 3050 Laptop 4 GB |

锁定的 `numpy==2.5.4` 要求 Python ≥ 3.12，`onnxruntime==1.31.1` 要求 ≥ 3.11，
系统 Python 3.10 无法安装。因此使用 `uv` 下载独立的 CPython 3.12.16，
不改动系统 Python，也不需要 sudo。

## 一次性准备

```bash
# uv 与独立 Python 3.12；安装到 ~/.local，无需 root
curl -LsSf https://astral.sh/uv/install.sh | sh
~/.local/bin/uv python install 3.12

# 仅用于让仓库脚本里的 python3 指向 3.12
mkdir -p ~/.local/aip-py312/bin
ln -sf ~/.local/bin/python3.12 ~/.local/aip-py312/bin/python3
```

## 安装与自检

```bash
cd ~/桌面/AIP/AIP-main
PATH="$HOME/.local/aip-py312/bin:$PATH" bash scripts/setup_go2_controller.sh
```

`setup_go2_controller.sh` 会在需要时调用 `setup_mujoco.sh`：创建 `.venv`、
安装锁定依赖、以固定提交拉取 `unitree_mujoco` 的 Go2 模型、下载并校验
`go2_rl_lab` 策略，最后运行控制器与地形检查。`.venv` 创建后即为 3.12，
后续运行不再需要 `PATH` 前缀。

## 本机验证结果

| 检查 | 结果 |
| --- | --- |
| `setup_mujoco.sh` 无窗口站立 10 s / 5000 步 | 通过，12 执行器，机身高度 0.263 m，0 警告 |
| `verify_go2_controller.py` | 14 项全部通过 |
| `verify_go2_terrain.py` | 通过（首组 5 cm 台阶、12° 坡、4–12 cm 乱石片段） |
| `verify_rescue_scene.py` | 35 项几何检查通过 |
| `render_rescue_views.py` | 7 视图 + 2 拼图，Mesa EGL 渲染成功 |
| X11 窗口运行（`run_rescue.sh`） | 正常运行并保持站立 |

产物：

- `output/mujoco/validation.json`、`gui-check.json`、`rescue-stand.png`
- `output/controller/validation.json`、`terrain-validation.json`
- `output/rescue-views/`（场地多视角渲染与尺寸复核图）

## 图形后端

脚本默认 `__GLX_VENDOR_LIBRARY_NAME=mesa`，无窗口用 Mesa EGL 软件渲染。
本机 X11 上会出现 `failed to load driver: nouveau` 提示，但不影响运行；
物理计算为 CPU，不需要 CUDA。若某次会话无法自动选到显卡，可加
`AIP_SOFTWARE_RENDERING=1`，或显式覆盖 `MUJOCO_GL` 等环境变量。
