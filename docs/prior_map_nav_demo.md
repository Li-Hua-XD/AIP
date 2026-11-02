# 背图版（prior_map）导航演示

本分支在原有 MuJoCo 赛场与 Go2 行走后端上，补了一条可运行的背图版导航闭环：

```
先验地图文件 → 校验 SHA-256 → 全局 A* 规划 → 纯跟踪控制 → 仿真行走 → 视频/报告
```

对应赛题“方向一：避障运控”的分段通行验证；不包含生命目标识别（V2）。

## 与不背图版的边界

| 项目 | 本演示 |
| --- | --- |
| 地图 | 离线从静态场地模型生成为 2.5D 栅格，运行前校验哈希 |
| 终点 | `goal_information = goal_provided`，显式给出终点区域 |
| 定位 | **仿真真值占位**，实机/正式评测需替换为雷达 + IMU 定位 |
| 规划 | 膨胀机器人半径后的栅格 A*，按赛道锚点分段规划 |
| 控制 | 纯跟踪（可倒车），速度 0.22–0.50 m/s |
| 特殊段 | 死胡同进入后原路倒车退出；圆管、方涵切换固定机位 |
| 动态障碍/弱光/目标 | 未包含 |

## 复现

```bash
cd ~/桌面/AIP/AIP-main
git checkout feature/prior-map-nav

# 1) 生成先验地图（只需在场地模型变化后重新生成）
.venv/bin/python scripts/build_prior_map.py

# 2) 跑完整背图版导航并录制视频
.venv/bin/python scripts/run_prior_map_nav.py

# 打开实时窗口（可选，实时速度）
.venv/bin/python scripts/run_prior_map_nav.py --viewer --no-video --no-frames

# 只跑控制、不出画面
.venv/bin/python scripts/run_prior_map_nav.py --no-video --no-frames
```

产物：

- `output/video/prior_map_nav.mp4`：追尾视角 + 俯视先验地图/路径/轨迹，25 fps
- `output/nav/prior_map_nav.json`：分段用时、横向误差、终点误差
- `output/nav/prior_map_nav.csv`：0.1 s 采样的位姿轨迹
- `output/prior_map/ruins_rescue_map.png`：先验地图检查图
- `output/prior_map/route_check.png`：全局路径检查图

## 路线

起点 → 坡道底 → 上坡 → 坡顶转弯/平台 → 下坡 → S 弯 → 接近死胡同 →
进入死胡同 → **倒车退出** → 绕过死胡同 → 圆管 → 方涵 → 终点，共 15 段、
规划长度约 31.1 m。

## 代码

| 文件 | 作用 |
| --- | --- |
| `aip/prior_map.py` | 2.5D 先验地图构建/校验、膨胀、A*、路径简化 |
| `aip/prior_nav.py` | 纯跟踪与倒车跟踪、横向误差 |
| `scripts/build_prior_map.py` | 从静态场地模型生成地图与检查图 |
| `scripts/run_prior_map_nav.py` | 分段规划、执行、渲染、视频与报告 |
| `configs/prior_map_route.json` | 分段锚点、速度、死胡同倒车标记 |

## 本机实跑结果

2026-11-02，`feature/prior-map-nav`：

| 项目 | 结果 |
| --- | --- |
| 路线 | 15 段，规划长度 31.1 m |
| 终点判定 | 到达（误差 0.194 m，容差 0.35 m） |
| 仿真用时 | 127.0 s |
| 姿态 | 未翻倒，最小 upright cos 0.96 |
| 视频 | `output/video/prior_map_nav.mp4`，25 fps，2 分 14 秒（片头 + 全程 + 片尾） |
| 报告 | `output/nav/prior_map_nav.json` |

视频由同一次带渲染的完整跑生成，共 3357 帧，包含片头、起点站立、全程、终点停留和片尾结果卡。
