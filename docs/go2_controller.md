# Go2 行走与任务开发：go2_rl_lab 后端

产品范围已按[PRD修订稿](design/废墟智援_PRD_v0.1.md)调整：V1优先不背图且终点未知，使用Go2 EDU 16 GB / 100 TOPS拓展坞及自带雷达/IMU做探索、建图和定位；D435i/MID360不用于V1。仿真终点仅由独立评估器按位置判定，实机验收方式待确认。本文的运动示例使用仿真真值，仅供控制开发，不等于不背图导航验收；生命目标识别在V2实现。


2026-10-30，运动后端已切换为用户指定的
[wty-yy/go2_rl_lab](https://github.com/wty-yy/go2_rl_lab)。当前使用作者发布的
`lab3.0` 混合地形 MoE-CTS 模型，原有任务接口继续使用。

## 直接运行

在项目根目录运行。当前机器已配置好，不需要先训练或重新安装。

```bash
# 前进约半米，按位置反馈停车
bash scripts/run_go2.sh --task forward --distance 0.5

# 左转 90 度，再停车
bash scripts/run_go2.sh --task turn --angle 90

# 前进半米 → 左转 90 度 → 再前进三十厘米
bash scripts/run_go2.sh --task demo

# 在比赛场景起点前进
bash scripts/run_go2.sh --scene rescue --task forward --distance 0.3

# 作者提供的楼梯场景：沿 +x 登上首组 5 cm 台阶
bash scripts/run_go2.sh --scene stairs --task forward --distance 3 --speed 0.4
```

`--speed` 为速度大小，单位 m/s；前进最多 `4.2`，负 `--distance` 表示后退，
后退最多 `4.5`。默认速度仍为 `0.2 m/s`。`demo` 最后一段固定前进，
因此其速度最多 `4.2 m/s`，即使第一段距离为负数。
命令行和 `move()` 共用 `configs/go2_controller.json` 中的速度范围校验。

默认打开窗口；SSH/无桌面加 `--headless`。例如：

```bash
bash scripts/run_go2.sh --headless --task demo \
  --report output/controller/my-demo.json \
  --screenshot output/controller/my-demo.png
```

可选场景：`flat`、`rescue`、`stairs`、`cross_slope`、`cross_stairs`、`race_track`。
除了 `rescue` 使用本项目赛场，其余场景来自指定仓库。场景能加载不代表各条路径都已验证。
`flat` 现在使用作者配套的平地，不含旧官方 `scene.xml` 中的额外阶梯障碍。

报告与同名前缀 CSV 记录位置、姿态、实际速度、指令和足端接触。
默认保存到 `output/controller/{scene}-{task}.json`；重复运行会覆盖。
全部任务超时和指令有效期均按仿真时间计算，无窗口运行速度通常快于现实时间。
前进/后退任务默认在 20 秒仿真时间后超时；增加 `--distance` 不会自动延长超时。
当前任务按位置反馈到达阈值后请求停车，没有提前减速规划；高速下的停车距离、
最终距离误差需单独验证，可能导致报告中的停车或精度检查不通过。

原始 `run_mujoco.sh` 和 `run_rescue.sh` 保留站立演示；行走使用 `run_go2.sh`。

## 任务代码如何写

任务函数：`aip/tasks.py`；
物理/策略后端：`aip/sim_robot.py`；场景组合：`aip/sim_scenes.py`。

```python
from aip.sim_robot import MujocoGo2
from aip.tasks import advance, drive_distance, turn_angle
import math

with MujocoGo2("flat", headless=True) as robot:
    advance(robot, 2)  # 等待机器人落地、零速度策略稳定
    print(drive_distance(robot, 0.5))
    print(turn_angle(robot, math.pi / 2))
    robot.stop()
    advance(robot, 1.5)
    print(robot.get_state())
```

自己写速度循环时：

```python
with MujocoGo2("flat", headless=True) as robot:
    advance(robot, 2)
    end = robot.get_state().time + 3
    try:
        while robot.get_state().time < end and robot.is_running():
            robot.move(0.2, 0.0, 0.0)
            state = robot.step()
            print(state.x, state.y, state.yaw)
    finally:
        robot.stop()
    advance(robot, 1.5)
```

`move()` 提交速度；`step()` 推进 0.02 仿真秒；连续 0.3 秒未续发命令则请求零速度。
因此“move 一次再 wait 3 秒”不会持续前进 3 秒。
便利接口 `robot.run_velocity(0.2, 0, 0, seconds=3)` 会持续更新指令并在结束时请求停止。

| 接口/字段 | 含义 |
| --- | --- |
| `move(vx, vy, vyaw)` | 机身坐标速度，m/s、m/s、rad/s |
| `stop()` | 请求零速度，通过状态确认物理停止 |
| `get_state()` | 当前状态的不可变快照 |
| `state.x/y/z` | 世界坐标位置，m；目前使用仿真真值 |
| `state.yaw` | 世界方向角，rad，范围 [-pi, pi] |
| `state.vx/vy/yaw_rate` | 机身坐标下的实际速度 |
| `state.command` | 当前生效的速度请求 |
| `reset(position=..., yaw=...)` | 新试验的初始位置/方向，同时清空十帧历史 |

自 2026-10-31 起，项目指令范围与当前固定 checkpoint 的平地训练范围对齐，
替换原 `±[0.6, 0.3, 0.8]` 的本地保守限制：

| 指令 | 最小值 | 最大值 | 单位 |
| --- | ---: | ---: | --- |
| 前后 `vx` | -4.5 | 4.2 | m/s |
| 左右 `vy` | -1.0 | 1.0 | m/s |
| 转向 `vyaw` | -3.0 | 3.0 | rad/s |

配置使用 `command_limits.min` 和 `command_limits.max`，按 `[vx, vy, vyaw]` 排列。
边界值允许，超限和非有限数报错；不会静默截断速度。
来源为 `third_party/go2_rl_lab/exported/config.yaml` 中
`training_parameters.environment.commands.base_velocity.terrain_max_command_ranges.flat`。
模型训练课程的最终前后范围同样为 `[-4.5, 4.2]`。

上游部署配置 `deploy/deploy_mujoco/config.yaml` 中的
`max_velocity_command: [2.0, 1.0, 3.0]` 是默认手柄映射范围，
脚本速度指令直接传入策略；本项目采用上述 checkpoint 的训练范围。

上游按地形设置了更低的训练指令范围：

| 训练地形 | 前后速度范围（m/s） | 转向范围（rad/s） |
| --- | --- | --- |
| 平地 | [-4.5, 4.2] | [-3, 3] |
| 波浪、上下坡、粗糙坡面 | [-1.5, 1.5] | [-1.5, 1.5] |
| 上下楼梯、障碍、小柱体 | [-1, 1] | [-1.5, 1.5] |
| 轮胎 | [-2, 1.5] | [-1.5, 1.5] |

这些地形的横移训练范围均为 `[-1, 1] m/s`。
当前适配器统一校验平地范围，不会识别所在赛段并自动切换地形限速。
`rescue` 包含多种地形，不能将平地最大指令视为废墟全程可用速度。
训练范围表示模型训练配置，并非实测可达速度或高速通行验收结果；
本文已有楼梯、坡道和乱石通行结果使用 `0.4 m/s` 指令。

任务逻辑不需要接触关节编号。实机后端仍需另外实现定位、时钟及 SDK 适配；
当前进程不连接真实 Go2，也不模拟完整的 `SportClient` RPC 服务。

## 固定的模型、配置和来源

- 代码仓库：`wty-yy/go2_rl_lab`，分支 `lab3.0`。
- 固定提交：`6c95d9feef4814c25eb6536e61bb38e1d1d718d4`。
- 模型：[作者发布的混合地形 checkpoint](https://huggingface.co/wty-yy/go2_rl_gym_data/tree/d1f19f0a486e171c327d8c3da2d31dba294e6586/go2_rl_lab/2026-10-13_15-35-09_mixed1.3.2resume_82.5k_0.6929/exported)。
- 权重版本：`mixed1.3.2resume_82.5k_0.6929`，训练 iteration 82500。
- 模型仓库修订：`d1f19f0a486e171c327d8c3da2d31dba294e6586`。
- ONNX SHA256：`6ad8e4672301be9d2e3c610b2c9a79cf2833993fd0a6b253622e96faab1a8f95`。
- 本地配置：`configs/go2_controller.json`。
- 所有模型、源码参考、许可证和资源的下载锁定：`configs/go2_rl_lab.lock.json`。

模型页标注 MIT；项目自有代码 MIT，其依赖/派生代码保留各自声明，下载目录内保留完整 LICENSE。
这取代了之前来源为 `Renkunzhao/legged_rl_deploy` 的活动模型；旧权重不再被启动脚本使用。
该模型是论文作者发布的研究运控，仍不是宇树 Go2 出厂运动服务。

## 这次实际迁移了什么

配套 ONNX 接受 **450 维输入**，包含 45 维单帧观测的十帧历史。
历史按每个观测项分块排列，每块由旧到新；不能直接把十个完整帧依次 flatten。
项目按作者 exporter 的布局维护和复位历史。

同步采用作者部署参数：

- 关节按 FL、FR、RL、RR，每腿 hip/thigh/calf 排列，并按名称映射至 MuJoCo 电机。
- 单帧为角速度、重力投影、缩放速度指令、相对关节位置、关节速度、上次动作。
- 物理步长 0.005 秒 / 200 Hz；策略周期 0.02 秒 / 50 Hz。
- `implicitfast` 积分，Newton 求解器，100 次迭代和 50 次线搜索上限。
- Kp=25、Kd=0.5、动作尺度 0.25，以及作者 Go2HV 方向相关力矩/转速包络。
- 使用作者配套机器人碰撞几何、惯量、关节阻尼/摩擦和电机限幅。

作者 TorchScript 模型将历史保存在模块内部；这里使用其 ONNX 导出，历史由本地适配器维护。
推理仍只需要 CPU ONNX Runtime。**没有安装 Isaac Lab/Isaac Sim 训练栈，也无需重新训练。**

组合场景在 `output/controller/scenes/` 生成，保留原始机器人资源与比赛场地。
只在复位时设置位置；运行中完全由电机力矩和接触产生运动。
当前没有向策略提供地形真值；这是作者发布的学生策略，不是需要特权地形输入的教师策略。

## 重建与复验

```bash
# 联网重建依赖与固定版本模型、资源，然后跑基础和地形检查
bash scripts/setup_go2_controller.sh

# 只校验/补齐下载文件，不修改依赖
.venv/bin/python scripts/fetch_go2_policy.py

# 运动、停车、命令有效期、历史复位
.venv/bin/python scripts/verify_go2_controller.py

# 指定楼梯、比赛坡道和固定乱石
.venv/bin/python scripts/verify_go2_terrain.py
```

下载需访问 PyPI、GitHub raw/media 和 Hugging Face 及其模型存储域名。
所有文件在写入前校验 SHA256；已有同名文件不匹配则停止并保留。
下载的权重、上游资源、生成场景和报告均不纳入 Git。

## 2026-10-31 速度范围修改验证

- 配置范围与固定 checkpoint 的平地训练范围逐项核对一致。
- 基础回归 14 项通过，包含非对称前后速度边界、横移/转向边界、
  超限和非有限数拒绝，以及原有运动、停车、指令过期和历史复位检查。
- 命令行 8 组检查通过：`--speed 4`、前进 `4.2`、后退 `4.5` 被接受，
  超限、零速度、非有限速度和 `demo` 后续前进段超限被拒绝。
  接受边界的命令行检查使用停车容差内的距离，只验证参数路径。
- 原有楼梯、比赛坡道、固定乱石 3 组地形回归通过，指令速度仍为 `0.4 m/s`。
- 平地发送 `4 m/s` 指令持续 2 秒，前向位移约 `5.847 m`，
  最后一秒平均实际前向速度约 `3.894 m/s`；随后请求零速度并等待 1.5 秒，
  末态平移速度约 `0.031 m/s`，保持直立且无物理警告。

平地短时试验使用持续速度接口，不验证高速定距停车精度，也不代表废墟地图高速通行能力。
本轮未实测 `4.2 m/s` 前进、`4.5 m/s` 后退、横移或转向的最大实际速度。
报告位于 `output/controller/validation.json`、`terrain-validation.json`、
`command-limit-validation.json` 和 `high-speed-flat-smoke.json`。

## 2026-10-30 无窗口验收结果

基础检查 13 项通过，包含前进、转向、组合任务、比赛起点坐标转换、停车、
指令超时、非法输入，以及物理状态和策略历史共同复位后的轨迹重现。

| 工况 | 结果 |
| --- | --- |
| 平地目标 0.5 m | 实际 0.498 m，误差约 -2 mm |
| 前进后左转 90° | 实际约 86.90°，在当前 0.1 rad 阈值内 |
| 转向后再前进 0.3 m | 实际 0.297 m |
| 比赛起点前进 0.3 m | 实际 0.310 m |
| 作者楼梯的首组 5 cm 台阶 | 前进 3 m，上到高约 0.3 m 的区域，停车检查通过 |
| 比赛 12° 坡道 | 从赛段前平地起步，上坡前进约 2.763 m，无机身接触 |
| 比赛第五组固定乱石，石块高 4–12 cm | 从该赛段前起步穿越，前进约 2.423 m，无机身接触 |

地形检查阈值：距离误差 <8 cm，末态平移速度 <0.07 m/s、角速度 <0.1 rad/s，
无物理警告、无机身接触、未翻倒；楼梯/坡道还检查机身高度确实上升。
单次成功是确定工况的冒烟验证，不代表随机工况成功率或全部比赛通关。

另一次 `cross_slope` 正 x 方向较陡坡道试验保持直立，但 3 m 任务停车后位置误差约
-9.6 cm，超过当前 8 cm 阈值，**该项未通过**。报告为 `output/controller/wty-slope.json`。
未验证完整上下楼、大台阶、可滚动碎石、全场导航，以及实际 Go2 部署。
新增窗口入口本轮未做桌面交互验收，离屏截图已生成。

报告：`output/controller/validation.json`、`output/controller/terrain-validation.json`。
