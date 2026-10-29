# AIP
废墟智援比赛

## 项目目录

```text
AIP/
├── docs/
│   ├── design/                   # PRD 与技术设计文档
│   ├── references/               # 比赛规则等参考资料
│   ├── mujoco_setup.md           # 环境安装与使用说明
│   └── rescue_scene.md           # 赛场使用与尺寸说明
├── scripts/                      # 安装、启动、场景生成与验证脚本
├── scenes/ruins_rescue/           # 场景配置、模型、纹理及许可证
├── requirements-mujoco.txt        # 直接依赖
├── requirements-mujoco.lock.txt   # 锁定依赖版本
├── output/                       # 本地运行结果与导出文件（Git 忽略）
├── third_party/                  # 安装脚本下载的上游模型（Git 忽略）
├── .venv/                        # 本地 Python 环境（Git 忽略）
└── tmp/                          # 临时文件（Git 忽略）
```

项目文档：

- [产品需求文档 PRD v0.1](docs/design/废墟智援_PRD_v0.1.md)
- [技术设计文档 v0.1](docs/design/废墟智援_技术设计文档_v0.1.md)
- [废墟智援赛题规则](docs/references/废墟智援_赛题规则文档.pdf)

两份设计文档为 v0.1 评审稿，当前已配置的仿真环境以
[MuJoCo 配置文档](docs/mujoco_setup.md)和[赛场说明](docs/rescue_scene.md)为准。

## 版本管理

直接使用当前 `AIP` 目录作为 Git 仓库。代码、文档、依赖清单和场景资源应纳入版本控制；
本地环境、下载的上游仓库、运行结果、临时文件、缓存和 MuJoCo 日志由 `.gitignore` 排除。
长期保留的说明文档放在 `docs/`，截图、验证报告和 PDF 导出等生成文件放在 `output/`。

克隆项目后，在 Python 3.12、git 和 curl 可用的环境中联网执行
`bash scripts/setup_mujoco.sh`，重建本地依赖与固定版本的上游模型，详见环境配置文档。

## 废墟救援赛场

已根据规则 PDF 第 6 页重建 12×10 米场地，包含上下坡、S 弯、死胡同、五处乱石、三种隧道和起终点。

```bash
bash scripts/run_rescue.sh
```

详见 [赛场使用与尺寸说明](docs/rescue_scene.md)。地图参数位于
`scenes/ruins_rescue/layout.json`，完整场景为 `scenes/ruins_rescue/scene.xml`。

## MuJoCo / Unitree Go2

已在项目 `.venv` 中配置独立的 MuJoCo 环境。安装说明、验证方法和范围见
[MuJoCo 配置文档](docs/mujoco_setup.md)。

在项目目录启动 Go2 站立窗口：

```bash
bash scripts/run_mujoco.sh
```

无窗口验证并保存渲染图：

```bash
bash scripts/run_mujoco.sh --headless --seconds 10 \
  --screenshot output/mujoco/go2.png --report output/mujoco/validation.json
```
