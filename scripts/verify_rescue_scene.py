"""Check compiled collision geometry, tunnel openings and Go2 standing dynamics."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/ruins_rescue"


def main():
    global SCENE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene",type=Path,default=SCENE)
    parser.add_argument("--report",type=Path,default=ROOT/"output/mujoco/rescue-geometry-validation.json")
    args = parser.parse_args()
    SCENE = args.scene.resolve()
    config = json.loads((SCENE / "layout.json").read_text())
    manifest = json.loads((SCENE / "geometry_manifest.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(SCENE / "arena.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    checks = []

    def check(name, ok, detail):
        checks.append({"name":name, "passed":bool(ok), "detail":detail})
        if not ok:
            raise AssertionError(f"{name}: {detail}")

    def ray(origin, direction, groups=(1,0,0,1,0,0)):
        geom_id = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, np.array(origin,dtype=float),
                                np.array(direction,dtype=float), np.array(groups,dtype=np.uint8),
                                True, -1, geom_id)
        name = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(geom_id[0])) if geom_id[0]>=0 else None
        return float(distance),name

    def ground_z(x,y):
        dist,name = ray([x,y,2],[0,0,-1])
        assert dist >= 0, (x,y,name)
        return 2-dist

    check("arena_dimensions", np.allclose(model.geom("rr_floor").size[:2]*2,[12,10]), "12 x 10 m")
    ramp = config["ramp"]
    x = ramp["center_x"]
    y0 = ramp["ascent_start_y"]
    dz = ground_z(x,y0+1.4)-ground_z(x,y0+0.4)
    angle = math.degrees(math.atan(dz))
    peak = ground_z(x,ramp["turn_y"]-0.1)
    check("ramp_slope", abs(angle-12)<0.005, {"measured_deg":angle})
    check("ramp_peak", abs(peak-.45)<1e-5, {"measured_height_m":peak})
    down_dz = ground_z(2.3,8.4)-ground_z(3.3,8.4)
    check("downhill_slope", abs(math.degrees(math.atan(down_dz))-12)<0.005, {"measured_drop_over_1m":down_dz})
    left,_ = ray([1,3,0.3],[-1,0,0]); right,_ = ray([1,3,0.3],[1,0,0])
    check("ramp_clear_width", abs(left+right-1.2)<1e-5, {"measured_m":left+right})
    s_manifest = manifest["s_bend"]
    widths = []
    points = np.array(s_manifest["centerline"])
    for i in range(2,len(points)-2,4):
        tangent = points[i+1]-points[i-1]
        normal = np.array([-tangent[1],tangent[0],0]); normal /= np.linalg.norm(normal)
        p = [*points[i],.3]
        a,_ = ray(p,normal); b,_ = ray(p,-normal)
        widths.append(a+b)
    check("s_bend_clear_width", np.allclose(widths,1.2,atol=.003), {"min_m":min(widths),"max_m":max(widths)})
    dead = config["dead_end"]
    dx,ey = dead["center_x"],dead["entrance_y"]
    dist,name = ray([dx,ey,.3],[0,-1,0])
    check("dead_end_length", abs(dist-2)<1e-5 and name=="rr_dead_end_back", {"measured_m":dist,"hit":name})
    a,_ = ray([dx,ey-1,.3],[-1,0,0]); b,_ = ray([dx,ey-1,.3],[1,0,0])
    check("dead_end_width", abs(a+b-1.2)<1e-5, {"measured_m":a+b})
    # Tunnel centerline must be empty along the entire cylinder axis.
    tube = config["tunnel_round"]
    tx,ty = tube["center"]
    dist,name = ray([tx-tube["length"]/2-.1,ty,.5],[1,0,0])
    check("round_tunnel_open_axis", dist>tube["length"]+.15, {"first_hit_distance":dist,"hit":name})
    radial = [ray([tx,ty,.5],d)[0] for d in [[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]]
    check("round_tunnel_inner_diameter", np.allclose(radial,.5,atol=1e-5), {"inner_radii_m":radial})
    narrow = config["tunnel_narrow"]
    nx,ny = narrow["center"]
    a,_ = ray([nx,ny,.3],[0,1,0]); b,_ = ray([nx,ny,.3],[0,-1,0])
    check("narrow_tunnel_clearance",abs(a+b-.2)<1e-5,{"measured_width_m":a+b})
    box = config["tunnel_box"]
    bx,by = box["center"]
    a,_ = ray([bx,by,.3],[-1,0,0]); b,_ = ray([bx,by,.3],[1,0,0])
    up,_ = ray([bx,by,.3],[0,0,1])
    check("box_tunnel_clearance", abs(a+b-1.2)<1e-5 and abs(up+.3-.6)<1e-5,
          {"width_m":a+b,"height_m":up+.3})
    dist,name = ray([bx,by-box["length"]/2-.1,.3],[0,1,0])
    check("box_tunnel_open_axis",dist>box["length"]+.2,{"first_hit_distance":dist,"hit":name})

    def clear_section(label,center,axis,length,offsets):
        failures=[]
        for sign in (-1,1):
            direction=np.zeros(3); direction[axis]=sign
            for offset in offsets:
                origin=np.asarray(center,dtype=float)+offset-direction*(length/2+.02)
                for groups in [(1,0,0,1,0,0),(0,1,0,0,0,0)]:
                    dist,hit=ray(origin,direction,groups)
                    if 0 <= dist < length+.04-1e-5:
                        failures.append({"origin":origin.tolist(),"distance":dist,"hit":hit})
        check(label+"_full_section_bidirectional",not failures,
              {"visual_and_collision_rays":len(offsets)*4,"failures":failures[:5]})

    for label,c,axis in [("narrow",narrow,0),("box",box,1)]:
        offsets=[]
        for lateral in np.linspace(-c["clear_width"]/2+.001,c["clear_width"]/2-.001,9):
            for z in np.linspace(.001,c["clear_height"]-.001,9):
                offset=np.zeros(3); offset[1-axis]=lateral; offset[2]=z
                offsets.append(offset)
        clear_section(label,[*c["center"],0],axis,c["length"],offsets)
        dimensions=[]
        for along in np.linspace(-c["length"]/2+.001,c["length"]/2-.001,7):
            position=np.array([*c["center"],.3]); position[axis]+=along
            lateral=np.zeros(3); lateral[1-axis]=1
            width=ray(position,lateral)[0]+ray(position,-lateral)[0]
            height=ray(position,[0,0,1])[0]+ray(position,[0,0,-1])[0]
            dimensions.append([width,height])
        check(label+"_clearance_full_length",np.allclose(dimensions,[c["clear_width"],c["clear_height"]],atol=1e-5),
              {"measured_width_height_m":dimensions})
    offsets=[np.zeros(3)]
    for radius in [.1,.3,.499]:
        offsets.extend([0,radius*math.cos(t),radius*math.sin(t)]
                       for t in np.linspace(0,2*math.pi,64,endpoint=False))
    clear_section("round",[tx,ty,.5],0,tube["length"],offsets)
    radii=[]
    for xx in np.linspace(tx-tube["length"]/2+.001,tx+tube["length"]/2-.001,7):
        for theta in np.linspace(0,2*math.pi,256,endpoint=False):
            radii.append(ray([xx,ty,.5],[0,math.cos(theta),math.sin(theta)])[0])
    check("round_full_length_inner_radius",min(radii)>=.5-1e-6 and max(radii)<=.500151,
          {"samples":len(radii),"min_radius_m":min(radii),"max_radius_m":max(radii)})

    # Measure lengths from actual compiled geometry, independent of the configuration metadata.
    for label,c,axis,prefix in [("narrow",narrow,0,"rr_tunnel_narrow_"),
                                ("round",tube,0,"rr_tunnel_round_"),("box",box,1,"rr_tunnel_box_")]:
        extents=[]
        for i in range(model.ngeom):
            name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i)
            if not name or not name.startswith(prefix) or not model.geom_contype[i]:
                continue
            mid=model.geom_dataid[i]; start=model.mesh_vertadr[mid]; count=model.mesh_vertnum[mid]
            vertices=model.mesh_vert[start:start+count] @ data.geom_xmat[i].reshape(3,3).T+data.geom_xpos[i]
            extents.extend(vertices[:,axis])
        length=float(max(extents)-min(extents))
        check(label+"_actual_length",abs(length-c["length"])<1e-5,{"measured_m":length})
    # Reconstruct vertices in world coordinates to check actual rock height, not metadata alone.
    heights=[]
    for i in range(model.ngeom):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i)
        if name and name.startswith("rr_rubble_"):
            mid=model.geom_dataid[i]; start=model.mesh_vertadr[mid]; count=model.mesh_vertnum[mid]
            verts=model.mesh_vert[start:start+count] @ data.geom_xmat[i].reshape(3,3).T + data.geom_xpos[i]
            heights.append(float(verts[:,2].max()))
    check("five_rubble_piles",len(manifest["rubble"])==5 and len(heights)==5*config["rubble"]["rocks_per_pile"],{"piles":len(manifest["rubble"]),"rocks":len(heights)})
    check("rubble_heights",min(heights)>=.04-1e-6 and max(heights)<=.12+1e-6,{"min_m":min(heights),"max_m":max(heights)})
    # Exact corridor centerline is open above its walking surface, including floor segment seams.
    for name,pts in [("ramp",manifest["ramp"]["centerline"]),("s_bend",[[*p,0] for p in points])]:
        errors=[abs(ground_z(p[0],p[1])-p[2]) for p in pts[1:-1]]
        check(name+"_centerline_surface",max(errors)<1e-4,{"max_height_error_m":max(errors)})

    errors=[]
    for wall in manifest["wall_sections"]:
        xy=np.array(wall["xy"])
        for t in [.01,.5,.99]:
            near=xy[0]*(1-t)+xy[1]*t
            far=xy[3]*(1-t)+xy[2]*t
            direction=(far-near)/np.linalg.norm(far-near)
            top=wall["top_m"][0]*(1-t)+wall["top_m"][1]*t
            for z in [.01,top/2,top-.01]:
                for side in (-1,1):
                    start=near-direction*.025 if side==1 else far+direction*.025
                    for groups in [(1,0,0,1,0,0),(0,1,0,0,0,0)]:
                        dist,hit=ray([*start,z],[*(direction*side),0],groups)
                        if not 0 <= dist <= .026:
                            errors.append({"wall":wall["name"],"z":z,"distance":dist,"hit":hit})
    check("walls_solid_ground_to_top_both_faces",not errors,
          {"sections":len(manifest["wall_sections"]),"visual_and_collision_rays":len(manifest["wall_sections"])*36,
           "failures":errors[:5]})

    def physical_probe(position, velocity, radius, seconds):
        root=ET.parse(SCENE/"arena.xml").getroot()
        root.find("compiler").set("texturedir",str(SCENE))
        root.find("option").set("gravity","0 0 0")
        body=ET.SubElement(root.find("worldbody"),"body",name="probe",pos=" ".join(map(str,position)))
        ET.SubElement(body,"freejoint")
        ET.SubElement(body,"geom",type="sphere",size=str(radius),mass="0.1",friction="0 0 0")
        pm=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"))
        pd=mujoco.MjData(pm); pd.qvel[:3]=velocity
        for _ in range(round(seconds/pm.opt.timestep)):
            mujoco.mj_step(pm,pd)
        return pd.qpos[:3].copy()

    end=physical_probe([tx-tube["length"]/2-.15,ty,.5],[1,0,0],.12,tube["length"]+.3)
    check("round_tunnel_physical_passage",end[0]>tx+tube["length"]/2+.05,{"probe_end_xyz":end.tolist()})
    end=physical_probe([nx-narrow["length"]/2-.2,ny,.3],[1,0,0],.13,2.1)
    check("narrow_tunnel_physically_blocks_26cm_probe",end[0]<nx-narrow["length"]/2,{"probe_end_xyz":end.tolist()})
    end=physical_probe([bx,by-box["length"]/2-.2,.3],[0,1,0],.12,box["length"]+.4)
    check("box_tunnel_physical_passage",end[1]>by+box["length"]/2+.05,{"probe_end_xyz":end.tolist()})
    for sign in (-1,1):
        end=physical_probe([nx-sign*(narrow["length"]/2+.15),ny,.3],[sign,0,0],.08,narrow["length"]+.3)
        check(f"narrow_16cm_probe_direction_{sign}",sign*(end[0]-nx)>narrow["length"]/2+.05,
              {"probe_end_xyz":end.tolist()})
    end=physical_probe([tx+tube["length"]/2+.15,ty,.5],[-1,0,0],.12,tube["length"]+.3)
    check("round_tunnel_reverse_physical_passage",end[0]<tx-tube["length"]/2-.05,{"probe_end_xyz":end.tolist()})
    end=physical_probe([bx,by+box["length"]/2+.2,.3],[0,-1,0],.12,box["length"]+.4)
    check("box_tunnel_reverse_physical_passage",end[1]<by-box["length"]/2-.05,{"probe_end_xyz":end.tolist()})

    robot = mujoco.MjModel.from_xml_path(str(SCENE / "scene.xml"))
    state = mujoco.MjData(robot)
    mujoco.mj_resetDataKeyframe(robot,state,mujoco.mj_name2id(robot,mujoco.mjtObj.mjOBJ_KEY,"home"))
    joints=robot.actuator_trnid[:,0]
    qi,vi=robot.jnt_qposadr[joints],robot.jnt_dofadr[joints]
    target=state.qpos[qi].copy()
    for _ in range(5000):
        state.ctrl[:]=np.clip(50*(target-state.qpos[qi])-3*state.qvel[vi],robot.actuator_ctrlrange[:,0],robot.actuator_ctrlrange[:,1])
        mujoco.mj_step(robot,state)
    base=robot.body("base_link").id
    stable = np.isfinite(state.qpos).all() and state.xpos[base,2]>.18 and state.xmat[base].reshape(3,3)[2,2]>.9
    check("go2_10_second_stand",stable and not np.any(state.warning.number),
          {"seconds":float(state.time),"height_m":float(state.xpos[base,2]),"warnings":int(sum(state.warning.number))})
    output=args.report
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps({"passed":True,"checks":checks},ensure_ascii=False,indent=2)+"\n")
    print(json.dumps({"passed":True,"checks":len(checks),"report":str(output)},ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
