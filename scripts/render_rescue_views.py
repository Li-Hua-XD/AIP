"""Render repeatable engineering views of the measured rescue arena."""

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL","egl")
import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model",type=Path,default=ROOT/"scenes/ruins_rescue/arena.xml")
    parser.add_argument("--output",type=Path,default=ROOT/"output/rescue-views")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((args.model.parent/"layout.json").read_text())
    model=mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    captions={"overview":"01 / 正向全景","reverse":"02 / 反向全景","top":"03 / 俯视布局",
              "tunnels":"04 / 隧道区", "narrow_bore":"隧道 1 / 净宽 0.20 m · 双端贯通",
              "round_bore":"隧道 2 / 内径 1.00 m · 长 1.90 m",
              "box_bore":"隧道 3 / 净宽 1.20 × 净高 0.60 × 长 1.50 m"}
    views={}
    with mujoco.Renderer(model,height=1200,width=1600) as renderer:
        for name in captions:
            renderer.update_scene(data,camera=name)
            views[name]=Image.fromarray(renderer.render())
            views[name].save(args.output/f"{name}.png")
    font_path="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font=ImageFont.truetype(font_path,22); title=ImageFont.truetype(font_path,32)
    def sheet(names,filename,heading):
        canvas=Image.new("RGB",(1648,1440),"#eef0ed"); draw=ImageDraw.Draw(canvas)
        draw.text((24,18),heading,font=title,fill="#172b34")
        draw.text((24,65),"第 6 页尺寸复核 · 同一参数化模型的实际渲染",font=font,fill="#43555b")
        for i,name in enumerate(names):
            x,y=16+(i%2)*816,110+(i//2)*656
            if name is None:
                lines=["文档尺寸与建模取值","场地 12 × 10 m / 五处乱石",
                       "坡道 12° / 坡顶 0.45 m / 净宽 1.20 m",
                       "S 弯净宽 1.20 m / 半径取 1.10 m",
                       "死胡同净宽 1.20 × 深 2.00 m","乱石凸起严格限制在 0.04–0.12 m",
                       "隧道 1 净宽 0.20 m，机器人应放弃",
                       "隧道 2 文档长度 1.8–2.0 m，取 1.90 m",
                       "方涵尺寸按净宽 × 净高 × 长度解释",
                       "墙高、墙厚、精确坐标、石形为建模假设",
                       "实际场地以后续官方配置资料为准"]
                for j,line in enumerate(lines): draw.text((x+24,y+24+j*48),line,font=font,fill="#263c44")
            else:
                canvas.paste(views[name].resize((800,600),Image.Resampling.LANCZOS),(x,y))
                draw.text((x+8,y+610),captions[name],font=font,fill="#172b34")
        canvas.save(args.output/filename)
    sheet(["overview","reverse","top","tunnels"],"multiview.png","废墟智援 / 正确几何多视角参考")
    sheet(["narrow_bore","round_bore","box_bore",None],"tunnel-clearances.png","三条隧道 / 贯通内腔与尺寸核对")
    report={"renderer":"MuJoCo","version":mujoco.__version__,"model":str(args.model.resolve()),
            "model_sha256":hashlib.sha256(args.model.read_bytes()).hexdigest(),"views":list(captions),
            "source":cfg["source"],"layout_sha256":hashlib.sha256((args.model.parent/"layout.json").read_bytes()).hexdigest()}
    (args.output/"render-manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps({"views":len(views),"sheets":2,"output":str(args.output)},ensure_ascii=False))


if __name__=="__main__": main()
