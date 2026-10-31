# AIP · 废墟智援

使用 Unitree Go2 完成比赛场景中的自主探索、建图、定位与通行，并在后续阶段加入生命目标识别。项目先在 MuJoCo 中开发和回归，再迁移到真实 Go2 验证。

## 产品路线

硬件选型为单台 **Go2 EDU，16 GB / 100 TOPS 拓展坞版本**。

| 功能阶段 | 交付目标 | 传感器安排 |
| --- | --- | --- |
| **V1：导航与建图** | 从起点探索未知环境，在线建图和机器人定位，处理障碍与死胡同，完成通行并导出地图、轨迹和日志 | 使用本体自带雷达、IMU及可用本体状态；不使用 D435i、MID360，不依赖视觉或生命目标识别 |
| **V2：生命目标识别** | 在 V1 上增加目标识别、目标定位与去重、证据回传、人工复核及弱光评测 | 按验证结果决定是否选装 RealSense D435i；MID360 按建图/定位缺口另行评估 |

多机协同保留为非必要能力拓展，开发和验收以单机为基线。自带雷达的具体型号、数据接口、实际视场，以及拓展坞的系统与驱动环境，按交付设备核验。

### 两个导航版本

| 导航版本 | 运行方式 | 实施顺序 |
| --- | --- | --- |
| **不背图版 `online_unknown`** | 地图和终点位置均未知；每次冷启动建立在线地图，通过前沿及已观测未访问区域探索寻路 | **优先实现，V1 主线** |
| **背图版 `prior_map`** | 加载允许的先验地图，完成重定位，再结合实时观测规划和处理地图变化；终点信息单独配置 | 不背图核心闭环稳定后实现 |

导航版本与功能阶段独立：V2 也可以继续使用不背图导航。不背图自主算法不读取场景内部障碍坐标、预设航点、中心线、历史地图或终点坐标。

**终点验收**：仿真由独立评估器根据机器人真值位置和终点区域判定到达，并核对停稳；终点位置不提供给探索算法。实机如何验收终点尚未确认，先开展探索、建图、定位和分段通行验证，完整实机起终点验收待规则明确后补齐。

## 当前实现状态

| 模块 | 状态 |
| --- | --- |
| MuJoCo 环境与场景 | 已有本地环境及 12×10 m 参数化废墟场地，包含坡道、S 弯、死胡同、五组固定乱石和三类隧道 |
| Go2 仿真运动后端 | 已接入固定版本 `go2_rl_lab` 混合地形策略，支持速度命令、定距行走、转向和简单组合任务 |
| 基础运动验证 | 已有基础及部分确定地形片段的验证记录；完整场地和多次通行能力仍需验收 |
| V1 自主系统 | 自带雷达等效观测、在线 SLAM、未知环境探索、导航闭环、独立终点评估和地图导出待实现 |
| Go2 实机接入 | 硬件已选型，SDK、传感器、停止保护及仿真实机差异待到机验证 |
| V2 目标链路 | 待实现 |

下列命令运行当前基础仿真和运动示例；任务位置反馈使用仿真真值，尚不构成不背图建图定位或完整比赛验收。

## 快速开始

在项目根目录执行。开发机环境基线为 Ubuntu 24.04、Python 3.12；详细依赖见 [MuJoCo 环境说明](docs/mujoco_setup.md)。

### 安装仿真与行走后端

准备 Python 3.12、git 和 curl 后联网执行：

```bash
bash scripts/setup_go2_controller.sh
```

脚本在需要时调用 `setup_mujoco.sh`，创建本地 `.venv`、安装锁定依赖、下载固定版本模型与策略，并运行基础和地形检查。仅安装原始 MuJoCo 站立环境可执行 `bash scripts/setup_mujoco.sh`。下载来源和模型版本见 [Go2 行走与任务开发](docs/go2_controller.md)。

### 运行行走任务

```bash
# 平地前进约 0.5 m，然后停车
bash scripts/run_go2.sh --task forward --distance 0.5

# 左转 90°，然后停车
bash scripts/run_go2.sh --task turn --angle 90

# 前进、转向、再前进的组合任务
bash scripts/run_go2.sh --task demo

# 从废墟场景起点进行短距离行走
bash scripts/run_go2.sh --scene rescue --task forward --distance 0.3
```

无窗口运行并保存报告、轨迹和截图：

```bash
bash scripts/run_go2.sh --headless --task demo \
  --report output/controller/demo.json \
  --screenshot output/controller/demo.png
```

`--speed` 为速度大小，默认 `0.2 m/s`；负 `--distance` 表示后退。当前策略校验范围为前后 `[-4.5, 4.2] m/s`、横移 `±1 m/s`、转向 `±3 rad/s`。这些是固定 checkpoint 的平地训练指令范围，不是全赛道或实机许可速度；分地形限速、制动和包络需单独验证，详见 [控制器说明](docs/go2_controller.md#任务代码如何写)。

### 查看场地与站立演示

```bash
# 废墟赛场及起点站立展示
bash scripts/run_rescue.sh

# 原始 Go2 模型站立展示
bash scripts/run_mujoco.sh
```

场地参数位于 `scenes/ruins_rescue/layout.json`，完整场景为 `scenes/ruins_rescue/scene.xml`。当前场景包含示意图重建假设，最终尺寸以正式场地配置为准，见 [赛场说明](docs/rescue_scene.md)。不背图算法仅通过模拟传感器观察世界，完整场景配置由仿真与评估端持有。

### 验证与重新生成场景

```bash
# 基础运动、停车、命令有效期等检查
.venv/bin/python scripts/verify_go2_controller.py

# 指定台阶、坡道及固定乱石片段检查
.venv/bin/python scripts/verify_go2_terrain.py

# 场景几何验证
.venv/bin/python scripts/verify_rescue_scene.py
```

修改 `layout.json` 后，使用 `.venv/bin/python scripts/build_rescue_scene.py` 重新生成场景，再执行几何验证。既有运动记录涵盖首组 5 cm 台阶、12° 上坡片段和一组 4–12 cm 固定乱石；这些片段不能代替完整全场、多次运行或实机验收。报告默认写入 `output/`，同名文件可能被覆盖。

## 文档入口

| 文档 | 内容 |
| --- | --- |
| [PRD（修订 v0.2）](docs/design/废墟智援_PRD_v0.1.md) | 产品范围、硬件、功能分期、需求及验收门槛 |
| [不背图版导航方案](docs/design/导航方案_不背图版.md) | 未知地图与未知终点探索、在线建图、输入隔离和位置验收 |
| [背图版导航方案](docs/design/导航方案_背图版.md) | 先验地图、重定位、在线修正及地图失配处理 |
| [技术设计（修订 v0.2）](docs/design/废墟智援_技术设计文档_v0.1.md) | 模块、坐标、时钟、消息、控制保护和数据存储 |
| [仿真与实机复现实施方案](docs/design/MuJoCo仿真与Go2实机复现实施方案_v0.1.md) | 工程改造、开发顺序、Go2 接入和迁移验证 |
| [SDK 兼容性与接口缺口](docs/design/宇树SDK兼容性与比赛接口缺口_2026-10-31.md) | 官方接口、本地后端差异及待补能力 |
| [MuJoCo 环境说明](docs/mujoco_setup.md) | 安装、版本、站立与渲染入口 |
| [Go2 行走与任务开发](docs/go2_controller.md) | 行走策略来源、参数、任务接口及验证记录 |
| [赛场说明](docs/rescue_scene.md) | 建模尺寸、假设、生成及几何验证 |
| [比赛规则](docs/references/废墟智援_赛题规则文档.pdf) | 用户提供的赛事依据 |

设计主文档已修订为 v0.2，保留原文件名以兼容链接。文档中的拟建模块、接口和配置不代表已实现，运行能力以代码和对应验证记录为准。

## 项目目录

```text
AIP/
├── aip/                          # 当前 Go2 仿真后端和简单任务
├── configs/                      # 行走策略参数、来源及文件锁定清单
├── docs/
│   ├── design/                   # PRD、两种导航方案、技术设计和实施方案
│   ├── references/               # 比赛规则等参考资料
│   └── *.md                      # 环境、控制器和赛场使用说明
├── scenes/ruins_rescue/           # 场地配置、MJCF、纹理及许可证
├── scripts/                      # 安装、启动、模型下载、场景生成和验证
├── requirements-mujoco.txt        # MuJoCo 直接依赖
├── requirements-mujoco.lock.txt   # MuJoCo 锁定依赖
├── requirements-go2-control.txt   # 行走后端依赖
├── output/                       # 报告、轨迹和截图（Git 忽略）
├── third_party/                  # 下载的上游模型与策略（Git 忽略）
├── .venv/                        # 项目 Python 环境（Git 忽略）
└── tmp/                          # 临时文件（Git 忽略）
```

直接使用当前 `AIP` 目录作为 Git 仓库。代码、文档、依赖清单、配置和场景资源纳入版本管理；本地环境、下载资源、缓存与运行结果按 `.gitignore` 排除。长期说明放在 `docs/`，生成报告和截图放在 `output/`。
