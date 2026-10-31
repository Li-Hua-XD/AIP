"""组合 go2_rl_lab 配套机器人与已有场景，生成本地可检查的 MJCF。

不改写作者资源或比赛场地。输出只保存在 output/controller/scenes/。
"""

import copy
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE_NAMES = ("flat", "rescue", "stairs", "cross_slope", "cross_stairs", "race_track")


def make_scene(name, config):
    """将作者原样机器人模型放入对应地形；救援起点沿用比赛布局。"""
    assets = ROOT / config["asset_dir"]
    robot = ET.parse(assets / "go2.xml").getroot()
    robot.find("compiler").set("meshdir", str(assets / "go2"))
    body = robot.find("worldbody/body")
    for leg in ("FL", "FR", "RL", "RR"):
        calf = robot.find(f".//body[@name='{leg}_calf']")
        # 仅命名已有足端球体，保留其质量、形状和碰撞参数。
        foot = next(g for g in calf.findall("geom") if g.get("size") == "0.022")
        foot.set("name", leg)
    if name == "rescue":
        scene_path = ROOT / "scenes/ruins_rescue/arena.xml"
        original = ET.parse(ROOT / "scenes/ruins_rescue/scene.xml").getroot()
        key = original.find("keyframe/key[@name='home']")
        pose = list(map(float, key.get("qpos").split()))
        body.set("pos", f"{pose[0]} {pose[1]} 0.445")
        body.set("quat", " ".join(map(str, pose[3:7])))
    else:
        scene_path = assets / f"{name}.xml"
    scene = ET.parse(scene_path).getroot()
    for section in ("asset", "worldbody"):
        destination = robot.find(section)
        source = scene.find(section)
        if source is None:
            continue
        for item in source:
            item = copy.deepcopy(item)
            for resource in item.iter():
                filename = resource.get("file")
                if filename:
                    resource.set("file", str((scene_path.parent / filename).resolve()))
            destination.append(item)
    for section in ("visual", "statistic"):
        source = scene.find(section)
        if source is not None:
            existing = robot.find(section)
            if existing is not None:
                robot.remove(existing)
            robot.append(copy.deepcopy(source))
    payload = ET.tostring(robot, encoding="unicode")
    path = ROOT / f"output/controller/scenes/{name}.xml"
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, suffix=".xml") as f:
        f.write(payload)
        temporary = Path(f.name)
    temporary.replace(path)
    return path, payload
