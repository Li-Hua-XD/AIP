# 废墟智援 MuJoCo 场景

依据项目内[《废墟智援赛题规则文档》](references/废墟智援_赛题规则文档.pdf)第 6 页正文与嵌入示意图建模。
这是用于开发验证的参数化重建。示意图为透视图，没有测绘坐标；区域位置、墙体尺寸等假设见配置文件。
赛题组后续提供正式场地配置表时，应更新参数后重新生成。

## 直接运行

在项目根目录运行，Go2 在起点朝北站立，窗口默认展示全场：

```bash
bash scripts/run_rescue.sh
```

使用 MuJoCo 窗口的相机选项切换视角；选择 Free 后可用鼠标旋转、平移与缩放。
也可以在启动时选择俯视相机或机器人附近的自由视角：

```bash
bash scripts/run_rescue.sh --camera top
bash scripts/run_rescue.sh --camera follow
```

启动程序包含站立 PD 控制。自主行走、爬坡、避障和导航策略需要后续接入；当前不自动跑完全程。

无窗口运行并截图：

```bash
bash scripts/run_rescue.sh --headless --seconds 10 \
  --camera overview --width 1600 --height 1200 \
  --screenshot output/mujoco/rescue-overview.png \
  --report output/mujoco/rescue-validation.json
```

## 场景文件

| 文件 | 用途 |
| --- | --- |
| `scenes/ruins_rescue/scene.xml` | 包含 Go2 的完整 MJCF 场景，12 个电机，`home` 关键帧为起点站立姿态 |
| `scenes/ruins_rescue/arena.xml` | 纯场地 MJCF，便于接入其他机器人或检查地图 |
| `scenes/ruins_rescue/layout.json` | 场地尺寸、区域坐标、随机种子、摩擦参数和建模假设 |
| `scenes/ruins_rescue/layout.svg` | 标注区域和坐标的俯视布局图 |
| `scenes/ruins_rescue/geometry_manifest.json` | 坡道/S 弯中心线、起点姿态和石块参数 |
| `scenes/ruins_rescue/assets/` | 本项目生成的地面、砖墙与标记纹理 |

完整场景引用当前项目的 `third_party/unitree_mujoco/unitree_robots/go2/assets/` 网格。
这些模型已在上一阶段安装。搬移项目时需同时保留该目录，或先运行 `scripts/setup_mujoco.sh`。
纯场地 `arena.xml` 仅依赖同目录下的 `assets/`，可以单独复制使用。
`scene.xml` 中的 Go2 结构源于宇树官方模型；随场景保留了 `UNITREE_LICENSE.txt`。

用 Python 直接加载：

```python
import mujoco

model = mujoco.MjModel.from_xml_path("scenes/ruins_rescue/scene.xml")
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
mujoco.mj_resetDataKeyframe(model, data, key)
mujoco.mj_forward(model, data)
# 在自己的控制循环中设置 data.ctrl，再调用 mujoco.mj_step(model, data)。
```

## 原图尺寸与实现

坐标单位为米，西南角为 `(0,0)`，图中向右是 `+x`，向上是 `+y`，高度为 `+z`。

| 区域 | PDF 标注/正文 | 本场景实现 |
| --- | --- | --- |
| 场地 | 12×10 m | 横向 12 m，纵向 10 m，外围墙位于该净场地外侧 |
| 起点 | 1.5×1.5 m | 中心 `(1.0,0.9)`；Go2 朝 `+y` |
| 坡道 | 12°，坡顶 0.45 m，净宽 1.2 m | 上坡、转弯平台、下坡；上下坡水平投影长度各约 2.117 m |
| S 弯 | 净宽 1.2 m，半径 1.0–1.2 m | 取中心线半径 1.1 m，连续反向弯道 |
| 死胡同 | 1.2×2 m | 净宽 1.2 m、内部深度 2 m，入口朝北，需原路退回 |
| 乱石 | 5 处，凸起 4–12 cm | 5 组、每组 24 块固定凸多面体，顶面高度均在给定范围内 |
| 隧道 1 | 过窄 0.2 m，应放弃 | 净宽 0.2 m，接入终点隔墙的狭窄支路 |
| 隧道 2 | 圆管内径 1.0 m，长 1.8–2.0 m | 内径 1.0 m、长度 1.9 m，沿 x 轴，双端贯通 |
| 隧道 3 | 方涵 1.2×0.6×1.5 m | 按净宽×净高×长度解释，沿 y 轴，双端贯通 |
| 终点 | 2×2 m | 中心 `(10.8,8.7)`，从方涵或过窄支路进入 |

方涵三维标注顺序、S 弯半径的定义、精确位置、墙高/厚、坡顶平台长度和地面摩擦等均需以后续官方配置确认。
本场景默认墙高约 0.6 m，内部墙厚 0.12 m；这些是建模取值。

圆管由 48 个独立凸扇形壳段组成。其碰撞形状保留内腔，内接半径为 0.5 m，
不会因将整个圆管当作单个凸网格而堵住管口。坡道使用连续凸分段，段间有 0.05 mm 数值重叠以避免微小接缝。
乱石为固定障碍，未模拟可滚动或坍塌的碎石。起终点标记不参与碰撞。

配置中的 `route_waypoints` 表示粗略任务顺序，供规划系统使用；
坡道和 S 弯的细化中心线在 `geometry_manifest.json` 中。航点连线不是可直接执行的机器人轨迹。
当前地图覆盖静态运控环境；生命目标、动态障碍及传感器模型还需按后续任务要求配置。

## 改图与验证

修改 `layout.json` 后执行：

```bash
.venv/bin/python scripts/build_rescue_scene.py
.venv/bin/python scripts/verify_rescue_scene.py
```

构建脚本会重新生成两个 XML、纹理、布局图和几何说明。
坡道与 S 弯根据参数衔接，其他区域由配置中的坐标定位；调整通道尺度后需同时检查相邻区域的空间关系。
验证脚本针对当前 PDF 的尺寸规格检查编译后的几何，而非仅检查配置文本。

本机已通过 21 项验证：

- 12×10 m 净场地，坡道上下坡角度、顶高与通道净宽。
- S 弯净宽，死胡同内部宽度/深度，坡道和弯道中心线地面连续性。
- 三种隧道的几何开口、圆管内径及方涵净高。
- 动态球体实际穿过圆管和方涵，直径 0.26 m 的球体被 0.2 m 过窄支路阻挡。
- 五处乱石共 120 块的实际网格顶点高度在 4–12 cm 范围内。
- Go2 起点连续 10 秒 / 5000 步站立，无 MuJoCo 物理警告。

报告：`output/mujoco/rescue-geometry-validation.json`。
上述验证确认地图尺寸、碰撞与起点仿真正常；通关能力取决于后续控制和规划程序。
