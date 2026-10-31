"""Build a parameterized, physically collidable MuJoCo rescue arena from PDF page 6."""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "scenes/ruins_rescue"
ROBOT = ROOT / "third_party/unitree_mujoco/unitree_robots/go2/go2.xml"


def fmt(values):
    return " ".join(f"{float(x):.9g}" for x in values)


def element(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def save_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


class Arena:
    def __init__(self, cfg):
        self.c = cfg
        self.asset = ET.Element("asset")
        self.world = ET.Element("worldbody")
        self.footprints = []
        self.metadata = {"ramp": {}, "rubble": [], "tunnels": {}, "spawn": {}, "wall_sections": []}
        self.mesh_count = 0

    def box(self, name, center, size, material="rr_wall", collide=True, yaw=0):
        c, s = math.cos(yaw), math.sin(yaw)
        corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]]) * np.array(size[:2]) / 2
        corners = corners @ np.array([[c,s],[-s,c]]) + center[:2]
        if material == "rr_wall" and collide:
            self.prism(name, corners, center[2]-size[2]/2, center[2]+size[2]/2)
            return
        geom = element(self.world, "geom", name=name, type="box", pos=fmt(center),
                       size=fmt(np.array(size) / 2), material=material,
                       euler=fmt([0, 0, yaw]), **self.contact(collide))
        if collide:
            self.footprints.append((corners.tolist(), material))
        return geom

    def contact(self, enabled=True):
        return {"contype": "1" if enabled else "0", "conaffinity": "1" if enabled else "0",
                "condim": "3", "friction": fmt(self.c["surface_friction"]), "group": "0" if enabled else "1"}

    def prism(self, name, xy, bottom, top, material="rr_wall", footprint=True, visible=True):
        xy = np.asarray(xy)
        lo = np.broadcast_to(bottom, (len(xy),))
        hi = np.broadcast_to(top, (len(xy),))
        vertices = np.vstack([np.column_stack([xy, lo]), np.column_stack([xy, hi])])
        self.mesh(name, vertices, material, visible)
        if footprint:
            self.footprints.append((xy.tolist(), material))

    def mesh(self, name, vertices, material, visible=True):
        # Every mesh here is convex; a hollow tube is assembled from separate convex sectors.
        mesh_name = name + "_mesh"
        vertices = np.asarray(vertices)
        attrs = {}
        if material == "rr_wall":
            xy = vertices[:4,:2]
            area = np.sum(xy[:,0]*np.roll(xy[:,1],-1)-xy[:,1]*np.roll(xy[:,0],-1))
            if area < 0:
                vertices = vertices[[3,2,1,0,7,6,5,4]]
            quads = [[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            expanded,uv,faces = [],[],[]
            for f,indices in enumerate(quads):
                face = vertices[indices]
                if f < 2:
                    texture = face[:,:2]/3.2
                else:
                    direction = face[1,:2]-face[0,:2]
                    direction /= np.linalg.norm(direction)
                    texture = np.column_stack([face[:,:2] @ direction,face[:,2]])/3.2
                expanded.extend(face); uv.extend(texture)
                offset=f*4
                faces.extend([offset,offset+1,offset+2,offset,offset+2,offset+3])
            vertices=np.array(expanded)
            attrs={"texcoord":fmt(np.array(uv).ravel()),"face":" ".join(map(str,faces))}
        element(self.asset, "mesh", name=mesh_name, vertex=fmt(vertices.ravel()), **attrs)
        contact = self.contact()
        if not visible:
            contact["group"] = "3"  # Hidden collision inspection layer, still physically active.
        element(self.world, "geom", name=name, type="mesh", mesh=mesh_name,
                material=material, **contact)
        self.mesh_count += 1

    def surface(self, name, quads, material, uv=None):
        """Visible nonconvex shell, independent of the individual convex colliders.

        No internal caps: shared segment seams cannot flicker through the wall.
        """
        vertices, faces, normals, texture = [], [], [], []
        for i,quad in enumerate(quads):
            q = np.asarray(quad,dtype=float)
            normal = np.cross(q[1]-q[0],q[2]-q[0])
            normal /= np.linalg.norm(normal)
            vertices.extend(q); normals.extend([normal]*4)
            faces.extend([4*i,4*i+1,4*i+2,4*i,4*i+2,4*i+3])
            texture.extend(uv[i] if uv is not None else q[:,:2]/3.2)
        element(self.asset,"mesh",name=name+"_mesh",vertex=fmt(np.asarray(vertices).ravel()),
                face=" ".join(map(str,faces)),normal=fmt(np.asarray(normals).ravel()),
                texcoord=fmt(np.asarray(texture).ravel()))
        element(self.world,"geom",name=name,type="mesh",mesh=name+"_mesh",
                material=material,**self.contact(False))
        self.mesh_count += 1

    def wall_segment(self, name, a, b, thickness, height, base=0):
        a, b = np.array(a), np.array(b)
        delta = b - a
        self.box(name, [*((a+b)/2), base+height/2],
                 [np.linalg.norm(delta), thickness, height], yaw=math.atan2(delta[1], delta[0]))

    def corridor(self, prefix, pts, tangents, heights, width, thickness, wall_height, floor=False):
        pts, tangents, heights = np.asarray(pts), np.asarray(tangents), np.asarray(heights)
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
        distances = np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]
        surfaces = {side:([],[]) for side in (-1,1)}
        for i in range(len(pts)-1):
            z = [heights[i], heights[i+1], heights[i+1], heights[i]]
            if floor and max(z) > 1e-6:
                # 0.05 mm overlap prevents numerical cracks at shared convex-mesh edges.
                overlap=0.00005
                a=pts[i]-overlap*tangents[i]; b=pts[i+1]+overlap*tangents[i+1]
                slope=(heights[i+1]-heights[i])/np.linalg.norm(pts[i+1]-pts[i])
                za,zb=heights[i]-overlap*slope,heights[i+1]+overlap*slope
                xy = [a-width/2*normals[i], b-width/2*normals[i+1],
                      b+width/2*normals[i+1], a+width/2*normals[i]]
                self.prism(f"{prefix}_floor_{i:03}", xy, -0.04, [za,zb,zb,za], "rr_ramp")
            for side in (-1, 1):
                near = side * width/2
                far = side * (width/2 + thickness)
                xy = [pts[i]+near*normals[i], pts[i+1]+near*normals[i+1],
                      pts[i+1]+far*normals[i+1], pts[i]+far*normals[i]]
                top = np.array(z)+wall_height
                name = f"{prefix}_wall_{side}_{i:03}"
                self.prism(name,xy,-0.02,top,visible=False)
                self.metadata["wall_sections"].append({"name":name,"xy":np.asarray(xy).tolist(),
                                                      "top_m":top.tolist(),"bottom_m":-0.02})
                vertices = np.vstack([np.column_stack([xy,[-.02]*4]),np.column_stack([xy,top])])
                faces = [[0,1,5,4],[2,3,7,6],[4,5,6,7],[3,2,1,0]]
                if i == 0: faces.append([3,0,4,7])
                if i == len(pts)-2: faces.append([1,2,6,5])
                u = np.array([distances[i],distances[i+1],distances[i+1],distances[i]]*2)
                quads,textures = surfaces[side]
                for j,indices in enumerate(faces):
                    if side == -1: indices = indices[::-1]
                    face = vertices[indices]
                    quads.append(face)
                    textures.append(np.column_stack([u[indices],face[:,2]])/3.2 if j<2 else face[:,:2]/3.2)
        for side,(quads,textures) in surfaces.items():
            self.surface(f"{prefix}_wall_surface_{side}",quads,"rr_wall",textures)

    def materials(self):
        assets = OUT / "assets"
        assets.mkdir(parents=True, exist_ok=True)
        rng = np.random.default_rng(123)
        noise = rng.normal(0, 4, (512, 512, 1))
        brick = Image.fromarray(np.uint8(np.clip(np.array([140,143,139])+noise, 0, 255)))
        draw = ImageDraw.Draw(brick)
        for row in range(16):
            draw.line([(0,row*32),(512,row*32)], fill=(83,86,82), width=2)
            offset = 32 if row % 2 else 0
            for x in range(offset, 513, 64):
                draw.line([(x,row*32),(x,(row+1)*32)], fill=(83,86,82), width=2)
        brick.save(assets / "brick.png")
        sand = np.uint8(np.clip(np.array([177,163,139])+rng.normal(0, 3, (256,256,1)), 0, 255))
        Image.fromarray(sand).save(assets / "sand.png")
        element(self.asset, "texture", name="rr_brick_tex", type="2d", file="assets/brick.png")
        element(self.asset, "texture", name="rr_sand_tex", type="2d", file="assets/sand.png")
        element(self.asset, "texture", name="rr_sky", type="skybox", builtin="gradient", rgb1="0.25 0.3 0.35", rgb2="0.65 0.68 0.68", width="512", height="3072")
        element(self.asset, "material", name="rr_wall", texture="rr_brick_tex", texuniform="false", texrepeat="1 1", reflectance="0", specular="0")
        element(self.asset, "material", name="rr_ground", texture="rr_sand_tex", texuniform="true", texrepeat="2 2", reflectance="0", specular="0")
        colors = {"rr_ramp": "0.64 0.55 0.42 1", "rr_pipe": "0.48 0.51 0.51 1",
                  "rr_start": "0.2 0.43 0.7 1", "rr_finish": "0.22 0.54 0.38 1",
                  "rr_white": "0.92 0.93 0.9 1"}
        for i in range(5):
            v = 0.39 + i * 0.045
            colors[f"rr_rock_{i}"] = fmt([v+0.025, v+0.01, v-0.025, 1])
        for name, rgba in colors.items():
            element(self.asset, "material", name=name, rgba=rgba, reflectance="0", specular="0")
        for label in ("START", "FINISH"):
            img = Image.new("RGB", (512,128), (235,236,228))
            d = ImageDraw.Draw(img)
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 76)
            d.text((256,64), label, font=font, fill=(40,46,44), anchor="mm")
            img.save(assets / f"{label.lower()}.png")
            element(self.asset, "texture", name=f"rr_{label}_tex", type="2d", file=f"assets/{label.lower()}.png")
            element(self.asset, "material", name=f"rr_{label}_label", texture=f"rr_{label}_tex", texrepeat="1 1", texuniform="false", specular="0")

    def bounds(self):
        c = self.c["arena"]
        w,d,h,t = (c[k] for k in ("width", "depth", "wall_height", "wall_thickness"))
        self.box("rr_floor", [w/2,d/2,-0.06], [w,d,0.12], "rr_ground")
        for name,pos,size in [
            ("south", [w/2,-t/2,h/2], [w+2*t,t,h]),
            ("north", [w/2,d+t/2,h/2], [w+2*t,t,h]),
            ("west", [-t/2,d/2,h/2], [t,d,h]),
            ("east", [w+t/2,d/2,h/2], [t,d,h])]:
            self.box("rr_boundary_"+name,pos,size)
        for name in ("start", "finish"):
            zone = self.c[name]
            x,y = zone["center"]
            sx,sy = zone["size"]
            self.box("rr_"+name+"_pad", [x,y,0.001], [sx,sy,0.002], "rr_"+name, False)
            self.box("rr_"+name+"_label", [x,y,0.003], [min(1.25,sx-0.1),0.32,0.002], f"rr_{name.upper()}_label", False)
            element(self.world, "site", name=name, pos=fmt([x,y,0.015]), type="sphere", size="0.035", rgba="1 1 0 0", group="5")
        for i,(x,y) in enumerate(self.c["route_waypoints"]):
            element(self.world,"site",name=f"route_{i:02}",pos=fmt([x,y,0.04]),size="0.04",rgba="1 0.7 0 0",group="5")

    def ramp_and_s(self):
        c = self.c["ramp"]
        x,y0,yt,r = (c[k] for k in ("center_x","start_y","turn_y","turn_radius"))
        run = c["height"] / math.tan(math.radians(c["angle_deg"]))
        ys = [y0,c["ascent_start_y"],c["ascent_start_y"]+run,yt]
        pts = [[x,y] for y in ys]
        tangents = [[0,1]]*len(ys)
        heights = [np.clip((y-c["ascent_start_y"])/run,0,1)*c["height"] for y in ys]
        for angle in np.linspace(math.pi,math.pi/2,19)[1:]:
            pts.append([x+r+r*math.cos(angle),yt+r*math.sin(angle)])
            tangents.append([math.sin(angle),-math.cos(angle)])
            heights.append(c["height"])
        xs = [x+r,x+r+run,c["exit_x"]]
        for xx in xs[1:]:
            pts.append([xx,yt+r]); tangents.append([1,0])
            heights.append(max(0,c["height"]-(xx-x-r)*math.tan(math.radians(c["angle_deg"]))))
        self.corridor("rr_ramp",pts,tangents,heights,c["width"],c["wall_thickness"],c["wall_height"],True)
        self.metadata["ramp"] = {"slope_deg": c["angle_deg"],"height_m":c["height"],"horizontal_run_m":run,
                                  "centerline":[[*p,float(z)] for p,z in zip(pts,heights)]}
        # Curved wall between the start bay and the open rubble field.
        divider=self.c["start_divider"]
        dp = np.asarray(divider["points"],dtype=float)
        directions = np.diff(dp,axis=0)
        directions /= np.linalg.norm(directions,axis=1)[:,None]
        normals = np.column_stack([-directions[:,1],directions[:,0]])
        miters = [normals[0]]
        miters.extend((a+b)/(1+np.dot(a,b)) for a,b in zip(normals[:-1],normals[1:]))
        miters.append(normals[-1])
        offsets = np.asarray(miters)*divider["wall_thickness"]/2
        for i in range(len(dp)-1):
            xy = [dp[i]-offsets[i],dp[i+1]-offsets[i+1],dp[i+1]+offsets[i+1],dp[i]+offsets[i]]
            self.prism(f"rr_start_divider_{i}",xy,-.02,divider["wall_height"])
        s = self.c["s_bend"]
        radius = s["radius"]
        sx,sy = pts[-1]
        spts,st = [],[]
        for angle in np.linspace(math.pi/2,-math.pi/2,37):
            spts.append([sx+radius*math.cos(angle),sy-radius+radius*math.sin(angle)])
            st.append([math.sin(angle),-math.cos(angle)])
        for angle in np.linspace(math.pi/2,math.pi,19)[1:]:
            spts.append([sx+radius*math.cos(angle),sy-3*radius+radius*math.sin(angle)])
            st.append([-math.sin(angle),math.cos(angle)])
        self.corridor("rr_s",spts,st,np.zeros(len(spts)),s["width"],s["wall_thickness"],s["wall_height"])
        self.metadata["s_bend"] = {"clear_width_m":s["width"],"centerline_radius_m":radius,"centerline":spts}

    def dead_end(self):
        c = self.c["dead_end"]
        x,y,w,l = (c[k] for k in ("center_x","entrance_y","clear_width","clear_length"))
        t,h = c["wall_thickness"],c["wall_height"]
        self.box("rr_dead_end_left",[x-w/2-t/2,y-l/2,h/2],[t,l,h])
        self.box("rr_dead_end_right",[x+w/2+t/2,y-l/2,h/2],[t,l,h])
        self.box("rr_dead_end_back",[x,y-l-t/2,h/2],[w+2*t,t,h])

    def tunnel_box(self, prefix, c, axis="y"):
        x,y = c["center"]
        w,h,l,t = (c[k] for k in ("clear_width","clear_height","length","wall_thickness"))
        if axis == "y":
            for side in (-1,1):
                self.box(prefix+f"_side_{side}",[x+side*(w+t)/2,y,h/2],[t,l,h])
            self.box(prefix+"_roof",[x,y,h+t/2],[w+2*t,l,t])
        else:
            for side in (-1,1):
                self.box(prefix+f"_side_{side}",[x,y+side*(w+t)/2,h/2],[l,t,h])
            self.box(prefix+"_roof",[x,y,h+t/2],[l,w+2*t,t])

    def tunnels(self):
        self.tunnel_box("rr_tunnel_narrow", self.c["tunnel_narrow"], "x")
        self.tunnel_box("rr_tunnel_box", self.c["tunnel_box"])
        c = self.c["tunnel_round"]
        x,y = c["center"]
        inner,l,t,n = c["inner_diameter"]/2,c["length"],c["shell_thickness"],c["segments"]
        # inner radius is the apothem: polygon approximation never reduces the stated clearance.
        ri,ro = inner/math.cos(math.pi/n),(inner+t)/math.cos(math.pi/n)
        shell_faces = []
        for i in range(n):
            a,b = (i-0.5)*2*math.pi/n,(i+0.5)*2*math.pi/n
            yz = [(y+r*math.cos(theta),inner+r*math.sin(theta)) for r,theta in [(ri,a),(ri,b),(ro,b),(ro,a)]]
            vertices = [[xx,yy,zz] for xx in [x-l/2,x+l/2] for yy,zz in yz]
            self.mesh(f"rr_tunnel_round_{i:03}",vertices,"rr_pipe",visible=False)
            visual_vertices = np.asarray(vertices)
            # 0.05 mm visual bias avoids coplanar flicker against the ground.
            # The physical tube and its specified 1 m bore are not shifted.
            visual_vertices[np.abs(visual_vertices[:,2])<1e-9,2] = .00005
            for indices in [[0,4,5,1],[3,2,6,7],[0,1,2,3],[4,7,6,5]]:
                shell_faces.append(visual_vertices[indices])
        self.surface("rr_tunnel_round_surface",shell_faces,"rr_pipe")
        self.footprints.append(([[x-l/2,y-ro],[x+l/2,y-ro],[x+l/2,y+ro],[x-l/2,y+ro]],"rr_pipe"))
        p = self.c["finish_partition"]
        ph,pt = p["wall_height"],p["wall_thickness"]
        nx,ny = self.c["tunnel_narrow"]["center"]
        gap = self.c["tunnel_narrow"]["clear_width"]
        for i,(low,high) in enumerate([(p["y"],ny-gap/2),(ny+gap/2,self.c["arena"]["depth"])]):
            self.box(f"rr_finish_partition_vertical_{i}",[p["x"],(low+high)/2,ph/2],[pt,high-low,ph])
        box = self.c["tunnel_box"]
        bx,by = box["center"]
        bw = box["clear_width"]
        for i,(lo,hi) in enumerate([(p["x"],bx-bw/2),(bx+bw/2,self.c["arena"]["width"])]):
            self.box(f"rr_finish_partition_horizontal_{i}",[(lo+hi)/2,p["y"],ph/2],[hi-lo,pt,ph])
        self.metadata["tunnels"] = {"narrow_width_m":gap,"round_inner_diameter_m":inner*2,
                                    "round_length_m":l,"box_clear_size_m":[bw,box["clear_height"],box["length"]],
                                    "all_bores_open_both_ends":True,
                                    "round_polygon_max_radial_error_m":ri-inner,
                                    "visual_floor_bias_m":.00005}

    def rubble(self):
        c = self.c["rubble"]
        rng = np.random.default_rng(c["seed"])
        for pile_idx,pile in enumerate(c["piles"]):
            cx,cy = pile["center"]
            rx,ry = pile["radius"]
            rocks = []
            # Angular stones cluster irregularly, with all peaks measured above ground.
            for i in range(c["rocks_per_pile"]):
                angle = i*math.pi*(3-math.sqrt(5))+rng.uniform(-.3,.3)
                radial = math.sqrt((i+.5)/c["rocks_per_pile"])*.84
                px,py = cx+rx*radial*math.cos(angle), cy+ry*radial*math.sin(angle)
                low,high = c["height_range"]
                height = low+(high-low)*rng.uniform(.35,1)*(1-.45*radial)
                if i == 0: height = high
                if i == c["rocks_per_pile"]-1: height = low
                radius = rng.uniform(.09,.19)*(1-.3*radial)
                yaw = rng.uniform(0,2*math.pi)
                rotation = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
                angles = np.arange(6)*math.pi/3+rng.uniform(-.18,.18,6)
                outline = np.column_stack([np.cos(angles),np.sin(angles)])
                outline *= rng.uniform(.75,1.25,(6,1))*[radius,radius*rng.uniform(.6,1.25)]
                outline = outline @ rotation
                lower = np.column_stack([outline+[px,py],np.full(6,-.012)])
                middle = np.column_stack([outline*rng.uniform(.75,1.05,(6,1))+[px,py],
                                          rng.uniform(.3,.65,6)*height])
                ridge_xy = np.array([[-.35,0],[.22,.18],[.1,-.25]])*radius @ rotation+[px,py]
                ridge = np.column_stack([ridge_xy,height*np.array([1,.82,.70])])
                self.mesh(f"rr_rubble_{pile_idx+1}_{i:02}",np.vstack([lower,middle,ridge]),f"rr_rock_{i%5}")
                self.footprints.append(((outline+[px,py]).tolist(),f"rr_rock_{i%5}"))
                rocks.append({"center":[px,py],"height":height,"radius_m":radius})
            self.metadata["rubble"].append({"pile":pile_idx+1,"rocks":rocks})

    def cameras(self):
        # Clean inspection lighting avoids shadow-map acne resembling damaged walls.
        element(self.world,"light",name="rr_sun",pos="3 2 12",dir="0.2 0.3 -1",directional="true",diffuse="0.55 0.53 0.48",ambient="0.15 0.15 0.15",specular="0 0 0",castshadow="false")
        element(self.world,"light",name="rr_fill",pos="10 8 8",dir="-0.3 -0.2 -1",diffuse="0.15 0.17 0.2",specular="0 0 0",castshadow="false")
        element(self.world,"camera",name="top",pos="6 5 19",xyaxes="1 0 0 0 1 0",fovy="33")
        cameras = {
            "overview":([13,-16,18],[6,5,.25],30),
            "reverse":([-5,25,18],[6,5,.25],30),
            "tunnels":([6,1.8,4],[9.7,7,.35],46),
            "round_bore":([6.15,5.35,.5],[10.5,5.35,.5],48),
            "narrow_bore":([7.1,8.25,.3],[10.5,8.25,.3],52),
            "box_bore":([10.8,4.1,.3],[10.8,8.7,.3],52),
        }
        for name,(eye,target,fovy) in cameras.items():
            eye,target = np.asarray(eye,dtype=float),np.asarray(target,dtype=float)
            z = (eye-target)/np.linalg.norm(eye-target)
            x = np.cross([0,0,1],z); x /= np.linalg.norm(x)
            y = np.cross(z,x)
            element(self.world,"camera",name=name,pos=fmt(eye),xyaxes=fmt([*x,*y]),fovy=str(fovy))

    def root(self, with_robot):
        if with_robot:
            root = ET.parse(ROBOT).getroot()
            root.set("model","ruins_rescue_go2")
            root.find("compiler").set("meshdir",os.path.relpath(ROBOT.parent/"assets",OUT))
            root.find("compiler").set("angle","radian")
            asset,world = root.find("asset"),root.find("worldbody")
            home = root.find("keyframe/key[@name='home']")
            qpos = np.array([float(x) for x in home.get("qpos").split()])
            yaw = math.radians(self.c["start"]["yaw_deg"])
            qpos[:2] = self.c["start"]["center"]
            qpos[3:7] = [math.cos(yaw/2),0,0,math.sin(yaw/2)]
            home.set("qpos",fmt(qpos)); home.set("ctrl",fmt(np.zeros(12)))
            # Generic viewers begin above the ground; the launcher resets the standing home keyframe.
            body = world.find("body[@name='base_link']")
            body.set("pos",fmt([qpos[0],qpos[1],0.445])); body.set("quat",fmt(qpos[3:7]))
            self.metadata["spawn"] = {"qpos":qpos.tolist(),"heading_deg":self.c["start"]["yaw_deg"]}
        else:
            root = ET.Element("mujoco",model="ruins_rescue_arena")
            element(root,"compiler",angle="radian")
            element(root,"option",timestep="0.002",gravity="0 0 -9.81")
            asset,world = element(root,"asset"),element(root,"worldbody")
        for node in self.asset: asset.append(copy.deepcopy(node))
        for node in self.world: world.append(copy.deepcopy(node))
        visual = element(root,"visual")
        element(visual,"global",offwidth="1600",offheight="1200",azimuth="110",elevation="-45")
        element(visual,"headlight",ambient="0.24 0.24 0.24",diffuse="0.25 0.25 0.25",specular="0 0 0")
        element(visual,"map",znear="0.005",zfar="80",shadowscale="0.6")
        element(visual,"quality",shadowsize="4096")
        element(root,"statistic",center="6 5 0.2",extent="8")
        return root

    def blueprint(self):
        from html import escape
        scale,ox,oy = 65,75,90
        def point(x,y): return ox+x*scale,oy+(10-y)*scale
        def polygon(points,color):
            return '<polygon points="'+" ".join(f"{a:.1f},{b:.1f}" for a,b in [point(*p) for p in points])+'" fill="'+color+'" stroke="#56616a" stroke-width="0.4"/>'
        svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1050" height="850" viewBox="0 0 1050 850">',
               '<rect width="1050" height="850" fill="#f4f3ef"/>',
               '<g font-family="sans-serif" fill="#23343d"><text x="75" y="40" font-size="25" font-weight="bold">废墟智援 · MuJoCo 场地重建</text>',
               '<text x="75" y="65" font-size="13">12 × 10 m · 依据规则第 6 页 · 区域坐标为开发用重建值</text>']
        for pts,mat in self.footprints:
            color = '#d9cbb0' if mat=='rr_ground' else '#c8ad7d' if mat=='rr_ramp' else '#7c898b' if mat=='rr_pipe' else '#a6a49c' if mat.startswith('rr_rock') else '#657275'
            svg.append(polygon(pts,color))
        for name,color in [("start","#6fa2d4"),("finish","#83b59e")]:
            c=self.c[name]; x,y=c["center"]; w,d=c["size"]
            svg.append(polygon([[x-w/2,y-d/2],[x+w/2,y-d/2],[x+w/2,y+d/2],[x-w/2,y+d/2]],color))
        path = " ".join(f"{x:.1f},{y:.1f}" for x,y in [point(*p) for p in self.c["route_waypoints"]])
        svg.append(f'<polyline points="{path}" fill="none" stroke="#d75f35" stroke-width="2.5" stroke-dasharray="7 6"/>')
        labels = [(1,.8,"起点 1.5×1.5"),(1.0,5.8,"12°坡道"),(4.4,7.3,"S 弯"),(3.15,2.9,"死胡同"),(8.7,8.6,"隧道1 · 0.2m"),(8.45,5.35,"隧道2 · Ø1m"),(10.8,6.05,"隧道3"),(10.8,8.7,"终点 2×2")]
        labels += [(p["center"][0],p["center"][1]+.4,f"乱石 {i+1}") for i,p in enumerate(self.c["rubble"]["piles"])]
        for x,y,label in labels:
            px,py=point(x,y)
            svg.append(f'<text x="{px}" y="{py}" text-anchor="middle" font-size="12" fill="#182a32" stroke="#f4f3ef" stroke-width="3" paint-order="stroke">{escape(label)}</text>')
        for x in range(13):
            px,py=point(x,0); svg.append(f'<text x="{px}" y="{py+25}" text-anchor="middle" font-size="12">{x}</text>')
        for y in range(11):
            px,py=point(0,y); svg.append(f'<text x="{px-25}" y="{py+4}" text-anchor="middle" font-size="12">{y}</text>')
        svg += ['<text x="430" y="790" font-size="14">x / m →</text>',
                '<text x="75" y="825" font-size="12">虚线为任务顺序参考；走廊转弯须沿中心线，不是可直接执行的运动轨迹。</text></g></svg>']
        (OUT / "layout.svg").write_text("\n".join(svg))


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout",type=Path,default=OUT/"layout.json")
    parser.add_argument("--output",type=Path,default=OUT)
    args = parser.parse_args()
    cfg = json.loads(args.layout.read_text())
    OUT = args.output.resolve()
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"layout.json").write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+"\n")
    arena = Arena(cfg)
    arena.materials(); arena.bounds(); arena.ramp_and_s(); arena.dead_end(); arena.tunnels(); arena.rubble(); arena.cameras()
    save_xml(arena.root(True), OUT / "scene.xml")
    save_xml(arena.root(False), OUT / "arena.xml")
    (OUT / "geometry_manifest.json").write_text(json.dumps(arena.metadata,ensure_ascii=False,indent=2)+"\n")
    (OUT / "UNITREE_LICENSE.txt").write_text((ROOT / "third_party/unitree_mujoco/LICENSE").read_text())
    arena.blueprint()
    print(f"Generated {OUT}/scene.xml and arena.xml ({arena.mesh_count} meshes, visual shells and convex colliders)")


if __name__ == "__main__":
    main()
