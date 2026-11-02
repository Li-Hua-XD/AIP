"""从场地模型生成背图版只读先验地图，并输出检查图。

用法：
    .venv/bin/python scripts/build_prior_map.py
    .venv/bin/python scripts/build_prior_map.py --map output/prior_map/ruins_rescue.npz
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aip.prior_map import PriorMap, build_map  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def render(map_: PriorMap, route, img_path, scale=60):
    h, w = map_.free.shape
    canvas = Image.new("RGB", (w * scale // 5, h * scale // 5), "#20242a")
    draw = ImageDraw.Draw(canvas)
    passable = map_.passable()
    for j in range(h):
        y = (h - 1 - j) * scale // 5  # 图上 +y 朝上
        for i in range(w):
            if not map_.free[j, i]:
                color = "#3c4756"
            elif not passable[j, i]:
                color = "#7a5c3a"
            else:
                color = "#d8d2c4"
            draw.rectangle([i * scale // 5, y, (i + 1) * scale // 5 - 1,
                            y + scale // 5 - 1], fill=color)
    def to_px(x, y):
        return (x / map_.resolution * scale // 5,
                (map_.depth_m - y) / map_.resolution * scale // 5)

    for x in range(0, int(map_.width_m) + 1):
        px, _ = to_px(x, 0)
        draw.line([(px, 0), (px, canvas.height)], fill="#2b3038", width=1)
        draw.text((px + 3, 4), str(x), fill="#8a94a3", font=ImageFont.truetype(FONT, 11))
    for y in range(0, int(map_.depth_m) + 1):
        _, py = to_px(0, y)
        draw.line([(0, py), (canvas.width, py)], fill="#2b3038", width=1)
        draw.text((4, py - 13), str(y), fill="#8a94a3", font=ImageFont.truetype(FONT, 11))

    for path in route or []:
        draw.line([to_px(*p) for p in path], fill="#1f7a4d", width=3, joint="curve")
        for p in path:
            px, py = to_px(*p)
            draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill="#1f7a4d")

    draw.text((12, canvas.height - 26), "深灰=墙体/场地外  棕=机器人半径膨胀  浅=可规划自由区  绿=全局路径",
              fill="#cfd6e2", font=ImageFont.truetype(FONT, 13))
    canvas.save(img_path)
    print(f"检查图：{img_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene", type=Path, default=ROOT / "scenes/ruins_rescue/arena.xml")
    p.add_argument("--map", type=Path, default=ROOT / "output/prior_map/ruins_rescue.npz")
    p.add_argument("--resolution", type=float, default=0.05)
    p.add_argument("--png", type=Path, default=ROOT / "output/prior_map/ruins_rescue_map.png")
    a = p.parse_args()
    if a.resolution <= 0 or not math.isfinite(a.resolution):
        p.error("分辨率必须为有限正数")

    data = build_map(a.scene, a.resolution)
    m = PriorMap(free=data["free"], elevation=data["elevation"],
                 resolution=data["resolution"], origin=tuple(data["origin"]),
                 ceiling=data["ceiling"])
    layout = ROOT / "scenes/ruins_rescue/layout.json"
    meta = m.save(a.map, source=a.scene.relative_to(ROOT), extra={
        "scene_layout": str(layout.relative_to(ROOT)),
        "scene_sha256": __import__("hashlib").sha256(a.scene.read_bytes()).hexdigest(),
        "notes": "背图版专用；由静态场地模型离线生成，不含机器人实时观测",
    })
    print(f"地图：{a.map}")
    print(f"栅格：{m.nx}×{m.ny} @ {m.resolution} m，自由格 {int(m.free.sum())}")
    print(f"SHA256：{meta['sha256']}")
    render(m, None, a.png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
