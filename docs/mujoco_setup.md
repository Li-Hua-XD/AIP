# MuJoCo 与 Go2 本地仿真环境

本环境用于“废墟智援”比赛前期开发，实体平台为 Unitree Go2 EDU（100 TOPS）。
当前完成基础物理仿真与 Go2 站立演示，并已按规则第 6 页重建废墟赛场，见 [场景使用说明](rescue_scene.md)。
行走策略、感知系统和实机部署仍需后续开发。

## 环境位置与版本

- 系统：Ubuntu 24.04，x86_64；Python 3.12。
- Python 环境：项目根目录下 `.venv`，不修改系统 Python。
- MuJoCo：3.3.6，采用宇树官方 README 推荐版本，官方 Python wheel 内含引擎和 viewer。
- 直接依赖：`requirements-mujoco.txt`；全部依赖的精确版本：`requirements-mujoco.lock.txt`。
- 官方模型：`third_party/unitree_mujoco/unitree_robots/go2/`。
- 模型仓库：https://github.com/unitreerobotics/unitree_mujoco
- 固定提交：`1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`。
- 下载采用 sparse checkout，仅保留 Go2 模型、Python 仿真源码与相关示例，上游 LICENSE 保留在仓库目录。

## 启动

在项目根目录运行：

```bash
bash scripts/run_mujoco.sh
```

窗口中使用鼠标旋转、缩放视角，关闭窗口退出。脚本用关节 PD 控制维持站立，默认持续运行。
它读取电机与关节的映射，避免把模型关节顺序误当作 SDK 电机顺序。

限定运行时间：

```bash
bash scripts/run_mujoco.sh --seconds 10
```

无窗口运行、保存截图和验证报告：

```bash
bash scripts/run_mujoco.sh --headless --seconds 10 \
  --screenshot output/mujoco/go2.png --report output/mujoco/validation.json
```

也可以直接使用官方模型查看器（不包含本项目的站立控制）：

```bash
source .venv/bin/activate
python -m mujoco.viewer --mjcf=third_party/unitree_mujoco/unitree_robots/go2/scene.xml
```

## 图形后端

安装检查时 `nvidia-smi` 无法连接 NVIDIA 驱动。因此启动脚本默认选择 Mesa GLX；
无窗口渲染使用 Mesa EGL，物理计算使用 CPU。
MuJoCo 基础仿真不需要 CUDA。当前未安装或调整 NVIDIA 驱动、CUDA、PyTorch 或 MJX。

如果当前会话无法自动选择可用的图形设备，可以显式使用软件渲染：

```bash
AIP_SOFTWARE_RENDERING=1 bash scripts/run_mujoco.sh
```

脚本保留调用者设置的 `MUJOCO_GL`、`__GLX_VENDOR_LIBRARY_NAME` 和
`__EGL_VENDOR_LIBRARY_FILENAMES`，后续修复显卡驱动后可覆盖默认后端。
窗口需要可访问的桌面会话；SSH/无显示器环境请使用 `--headless`。

## 重新安装与验证

Python 3.12、git 和 curl 可用后，联网运行以下命令即可重建环境：

```bash
bash scripts/setup_mujoco.sh
```

该脚本创建 `.venv`，必要时从 `bootstrap.pypa.io` 下载官方 pip 引导程序，
安装锁定依赖并取回固定提交的模型，最后执行 10 秒无窗口站立检查。
如果已有模型目录的提交不同，脚本报错，保留已有文件。

验证报告要求 12 个执行器、有限的物理状态、无 MuJoCo 警告，并检查机身高度与姿态。
输出文件保存在 `output/mujoco/`；本地虚拟环境、下载的上游仓库和运行输出均已加入忽略规则。

本机验收结果（2026-10-29）：

- MuJoCo 导入、依赖完整性检查通过。
- `scene.xml` 和 `scene_terrain.xml` 均成功加载，包含 12 个执行器。
- 无窗口运行 10 秒 / 5000 步，机身高度约 0.263 米，保持站立，物理警告为 0。
- Mesa EGL 成功生成 `output/mujoco/go2.png`。
- X11 可视窗口运行约 8 秒，保持站立，物理警告为 0。
- 两次测试的报告分别为 `output/mujoco/validation.json` 与 `output/mujoco/viewer-validation.json`。

在受限执行沙箱中直接打开 X11 会失败；上述窗口验证已在允许访问本机桌面的执行环境中通过。
在本机桌面终端执行启动命令即可。

## 与 Go2 EDU 100 TOPS 的对应关系

当前使用宇树公开的通用 Go2 刚体/关节模型，并非对 EDU 100 TOPS 实物完整数字化标定。
后续应按实机补充计算模块、传感器及载荷的质量、惯量、安装位置和噪声参数。
100 TOPS 描述实机端侧计算能力，不是 MuJoCo 的安装配置项。

本项目演示直接调用 MuJoCo，不使用 DDS，不连接或控制实机。
上游 `simulate_python` 源码已下载，但 Unitree SDK2 Python、CycloneDDS 与 pygame 尚未安装；
后续需要验证实机控制接口时再配置该桥接环境，仿真沿用官方 `domain_id=1`、`interface=lo`。
场地重建参数需在赛题组发布正式配置表后核对更新。

## 官方参考

- MuJoCo 安装与查看器：https://mujoco.readthedocs.io/en/stable/python.html
- 宇树仿真器与 SDK2 接口：https://github.com/unitreerobotics/unitree_mujoco
