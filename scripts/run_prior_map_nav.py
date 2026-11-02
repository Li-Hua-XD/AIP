"""背图版（prior_map）导航演示：先验地图 → 全局 A* → 纯跟踪 → 画面录制。

- 地图来自 ``output/prior_map/ruins_rescue.npz``，加载前校验 SHA-256；
  运行中不读取场景几何、不读取实时障碍坐标。
- 定位输入用仿真真值（见报告中的 localization 字段），用于先打通导航闭环；
  实机/正式评测需替换为雷达与 IMU 的定位结果。
- 终点按 ``goal_provided`` 显式给出，属于背图版允许的配置。

用法：
    .venv/bin/python scripts/run_prior_map_nav.py
    .venv/bin/python scripts/run_prior_map_nav.py --viewer
    .venv/bin/python scripts/run_prior_map_nav.py --no-video --max-seconds 60
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# --viewer 需要 GLFW 窗口；录制用 EGL 离屏渲染。两者不能混用同一后端。
if "--viewer" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "glfw")
else:
    if os.environ.get("MUJOCO_GL") is None:
        os.environ["MUJOCO_GL"] = "egl"
        nvidia = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        mesa = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
        if os.environ.get("AIP_EGL_MESA") != "1" and Path(nvidia).is_file():
            os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", nvidia)
        elif Path(mesa).is_file():
            os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", mesa)
os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "mesa")
sys.path.insert(0, str(ROOT))

import argparse  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from aip.prior_map import PriorMap  # noqa: E402
from aip.prior_nav import PathFollower, cross_track_error  # noqa: E402
from aip.sim_robot import MujocoGo2  # noqa: E402
from aip.tasks import advance, wrap_angle  # noqa: E402

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WALL_KEYS = ("wall", "tunnel", "dead_end", "divider", "partition")


_FONT_CACHE = {}


def font(size):
    if size not in _FONT_CACHE:
        _FONT_CACHE[size] = ImageFont.truetype(FONT_PATH, size)
    return _FONT_CACHE[size]


class Navigator:
    def __init__(self, map_path, route):
        self.map = PriorMap.load(map_path)
        self.route = route
        self.legs = []
        current = tuple(route["start"])
        for leg in route["legs"]:
            goal = tuple(leg["to"])
            path = self.map.plan(current, goal)
            if path is None:
                raise RuntimeError(f"先验地图上无法规划：{leg['name']} {current} -> {goal}")
            self.legs.append({
                "name": leg["name"],
                "from": current,
                "to": goal,
                "path": path,
                "speed": float(leg.get("speed", 0.4)),
                "lookahead": float(leg.get("lookahead", route["lookahead_default_m"])),
                "return_to_start": bool(leg.get("return_to_start", False)),
                "planned_m": self.map.path_length(path),
            })
            current = goal
        self.planned_total = sum(l["planned_m"] for l in self.legs) + \
            sum(2 * self.map.path_length(l["path"]) for l in self.legs if l["return_to_start"])
        self.leg_index = 0

    def full_route(self):
        pts = []
        for leg in self.legs:
            pts.extend(leg["path"][1:] if pts else leg["path"])
        return pts


class TopPanel:
    """俯视静态底图 + 路径/轨迹叠加；相机 top: pos(6,5,19), fovy 33。"""

    def __init__(self, background, width=576, height=480, distance=19.0, fovy=33.0):
        self.background = background.convert("RGB")
        self.width, self.height = width, height
        self.span_y = 2 * distance * math.tan(math.radians(fovy / 2))
        self.span_x = self.span_y * width / height
        self.center = (6.0, 5.0)
        self.font = font(14)

    def to_px(self, x, y):
        px = (x - self.center[0]) / self.span_x * self.width + self.width / 2
        py = self.height / 2 - (y - self.center[1]) / self.span_y * self.height
        return px, py

    def image(self, *, full_route, leg_path, trail, pose, goal, leg_name):
        img = self.background.copy()
        draw = ImageDraw.Draw(img)
        if len(full_route) > 1:
            draw.line([self.to_px(*p) for p in full_route], fill=(40, 180, 110), width=2)
        if len(leg_path) > 1:
            draw.line([self.to_px(*p) for p in leg_path], fill=(90, 255, 150), width=3)
        if len(trail) > 1:
            draw.line([self.to_px(*p) for p in trail], fill=(255, 150, 40), width=3)
        gx, gy = self.to_px(*goal)
        draw.ellipse([gx - 7, gy - 7, gx + 7, gy + 7], outline=(60, 255, 140), width=3)
        x, y, yaw = pose
        px, py = self.to_px(x, y)
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 70, 60))
        draw.line([px, py, px + 18 * math.cos(yaw), py - 18 * math.sin(yaw)],
                  fill=(255, 70, 60), width=3)
        draw.rectangle([0, 0, self.width, 26], fill=(16, 20, 26))
        draw.text((8, 4), f"先验地图 12×10 m ｜ {leg_name}", fill=(220, 230, 240), font=self.font)
        return img


VIDEO_W, VIDEO_H = 720 + 576, 480 + 34


def intro_card(top_panel, planned_full, planned_m, leg_count, start_pose):
    canvas = Image.new("RGB", (VIDEO_W, VIDEO_H), (14, 18, 24))
    draw = ImageDraw.Draw(canvas)
    draw.text((54, 74), "废墟智援 · 背图版导航演示", fill=(240, 245, 250), font=font(36))
    draw.text((54, 132), "prior_map ｜ 先验地图 + A* 全局规划 + 纯跟踪",
              fill=(70, 220, 140), font=font(21))
    draw.line([(54, 178), (620, 178)], fill=(60, 70, 82), width=2)
    lines = [
        "路线：起点 → 坡道 → S 弯 → 死胡同 → 圆管 → 方涵 → 终点",
        f"规划长度 {planned_m:.1f} m ｜ {leg_count} 段 ｜ 速度 0.22–0.50 m/s",
        "死胡同进入后原路倒车退出；圆管、方涵切换固定机位",
        "定位输入：仿真真值（占位，实机替换为雷达 / IMU）",
        "画面：左追尾视角 ｜ 右先验地图、全局路径与实时轨迹",
    ]
    for k, line in enumerate(lines):
        draw.text((56, 208 + 40 * k), line, fill=(200, 210, 222), font=font(18))
    draw.text((56, 448), "开始", fill=(70, 220, 140), font=font(22))
    mini = top_panel.image(full_route=planned_full, leg_path=[], trail=[],
                           pose=start_pose, goal=(10.8, 8.7), leg_name="先验地图与全局路径")
    canvas.paste(mini, (VIDEO_W - 576 - 17, 17))
    return canvas


def outro_card(top_panel, planned_full, trail, pose, result, planned_m):
    canvas = Image.new("RGB", (VIDEO_W, VIDEO_H), (14, 18, 24))
    draw = ImageDraw.Draw(canvas)
    draw.text((54, 86), "到达终点", fill=(70, 220, 140), font=font(44))
    rows = [
        ("终点误差", f"{result['goal_error_m']:.3f} m（容差 {result['finish_tolerance_m']:.2f} m）"),
        ("仿真时间", f"{result['sim_seconds']:.1f} s"),
        ("规划长度", f"{planned_m:.1f} m"),
        ("最小直立余弦", f"{result['min_upright_cos']:.2f}"),
    ]
    for k, (key, value) in enumerate(rows):
        y = 178 + 52 * k
        draw.text((58, y), key, fill=(150, 162, 176), font=font(18))
        draw.text((210, y), value, fill=(235, 240, 245), font=font(20))
    draw.text((58, 430), "背图版 prior_map ｜ 先验地图 + A* + 纯跟踪",
              fill=(120, 132, 148), font=font(16))
    mini = top_panel.image(full_route=planned_full, leg_path=[], trail=trail,
                           pose=pose, goal=(10.8, 8.7), leg_name="终点与轨迹")
    canvas.paste(mini, (VIDEO_W - 576 - 17, 17))
    return canvas


class VideoWriter:
    """优先 imageio；缺失时用系统 GStreamer(x264enc) 通过 stdin 编码。"""

    def __init__(self, path, width, height, fps):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.width, self.height, self.fps = width, height, fps
        self._imageio = None
        self._proc = None
        try:
            import imageio.v2 as imageio
            self._imageio = imageio.get_writer(str(path), fps=fps, codec="libx264",
                                               quality=7, macro_block_size=1)
            return
        except ImportError:
            pass
        cmd = [
            "gst-launch-1.0", "-q", "fdsrc", "fd=0", "!", "rawvideoparse",
            f"width={width}", f"height={height}", "format=rgb",
            f"framerate={int(round(fps))}/1",
            "!", "videoconvert", "!", "video/x-raw,format=I420", "!",
            "x264enc", "speed-preset=veryfast", "tune=zerolatency",
            "bitrate=6000", "key-int-max=40", "!", "mp4mux", "!",
            "filesink", f"location={self.path}",
        ]
        self._proc = __import__("subprocess").Popen(
            cmd, stdin=__import__("subprocess").PIPE,
            stderr=__import__("subprocess").PIPE)

    def append(self, frame):
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        if self._imageio is not None:
            self._imageio.append_data(frame)
            return
        assert self._proc is not None and self._proc.stdin is not None
        try:
            self._proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            err = ""
            if self._proc.stderr is not None:
                err = self._proc.stderr.read().decode(errors="replace")[-800:]
            raise RuntimeError(f"GStreamer 编码管道提前退出：{err}") from None

    def close(self):
        if self._imageio is not None:
            self._imageio.close()
            return
        if self._proc is None:
            return
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.wait(timeout=180)
        except Exception:
            self._proc.kill()
        if self._proc.returncode not in (0, None):
            err = self._proc.stderr.read().decode(errors="replace")[-800:]
            raise RuntimeError(f"GStreamer 编码失败：{err}")


def compose(chase_img, top_img, state, leg_name, elapsed, planned_m):
    width = chase_img.width + top_img.width
    height = chase_img.height + 34
    canvas = Image.new("RGB", (width, height), (16, 20, 26))
    canvas.paste(chase_img, (0, 34))
    canvas.paste(top_img, (chase_img.width, 34))
    draw = ImageDraw.Draw(canvas)
    draw.text((10, 7), "废墟智援 · 背图版 prior_map ｜ 先验地图 + A* 全局规划 + 纯跟踪 ｜ "
                       "定位：仿真真值（占位）", fill=(225, 235, 245), font=font(17))
    draw.rectangle([0, 34, chase_img.width, 66], fill=(0, 0, 0))
    draw.text((10, 40),
              f"t={elapsed:5.1f}s   v={math.hypot(state.vx, state.vy):4.2f} m/s   "
              f"leg={leg_name}   规划长度 {planned_m:4.1f} m",
              fill=(235, 240, 245), font=font(15))
    return canvas


def body_contacts(robot):
    """只统计机身（base）与墙体/隧道结构的接触；足端踩管壁不算碰撞。"""
    count = 0
    d, m = robot.data, robot.model
    for c in d.contact[:d.ncon]:
        names = []
        for g in (c.geom1, c.geom2):
            gid = int(g)
            name = m.geom(gid).name or ""
            body = m.body(m.geom_bodyid[gid]).name or ""
            names.append((name, body))
        wall = any(any(k in n for k in WALL_KEYS) for n, _ in names)
        trunk = any(b == "base" for _, b in names)
        if wall and trunk:
            count += 1
    return count


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--route", type=Path, default=ROOT / "configs/prior_map_route.json")
    p.add_argument("--map", type=Path)
    p.add_argument("--video", type=Path, default=ROOT / "output/video/prior_map_nav.mp4")
    p.add_argument("--report", type=Path, default=ROOT / "output/nav/prior_map_nav.json")
    p.add_argument("--frame-dir", type=Path, default=ROOT / "output/nav/frames")
    p.add_argument("--fps", type=float, default=25.0)
    p.add_argument("--max-seconds", type=float, default=240.0)
    p.add_argument("--viewer", action="store_true", help="打开 MuJoCo 窗口（实时速度）")
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--no-frames", action="store_true")
    a = p.parse_args()

    route = json.loads(a.route.read_text())
    map_path = a.map or (ROOT / route["map"])
    nav = Navigator(map_path, route)
    print(f"先验地图：{map_path}")
    print(f"地图 SHA256：{nav.map.meta['sha256']}")
    print(f"规划总长：{nav.planned_total:.2f} m（{len(nav.legs)} 段）")
    for leg in nav.legs:
        print(f"  {leg['name']:<16s} {leg['planned_m']:5.2f} m  speed={leg['speed']:.2f}")

    fps = a.fps
    render_dt = 1.0 / fps if fps > 0 else 0.05
    robot = MujocoGo2("rescue", headless=not a.viewer)
    report = {
        "mode": route["navigation_mode"],
        "goal_information": route["goal_information"],
        "localization": "sim ground truth (placeholder for lidar/IMU localization)",
        "map": {
            "file": str(map_path),
            "sha256": nav.map.meta["sha256"],
            "resolution_m": nav.map.resolution,
            "origin_m": list(nav.map.origin),
        },
        "start": route["start"],
        "goal": route["goal"],
        "planned_total_m": nav.planned_total,
        "legs": [],
        "video": None if a.no_video else str(a.video),
        "result": {},
    }
    planned_full = nav.full_route()
    trail = []
    step_count = 0
    total_steps = 0
    t_start = None
    frames = 0
    renderer = None
    top_panel = None
    chase_cam = None
    writer = None
    csv_file = None
    csv_writer = None
    outcome = {"reached": False, "error": None}

    try:
        if not a.no_video or not a.no_frames:
            if not a.no_frames:
                a.frame_dir.mkdir(parents=True, exist_ok=True)
            renderer = mujoco.Renderer(robot.model, height=480, width=720)
            top_cam = mujoco.MjvCamera()
            top_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            top_cam.fixedcamid = mujoco.mj_name2id(
                robot.model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
            with mujoco.Renderer(robot.model, height=480, width=576) as top_renderer:
                top_renderer.update_scene(robot.data, camera=top_cam)
                background = Image.fromarray(top_renderer.render())
            top_panel = TopPanel(background)
            fixed_cams = {}
            for cam_name in ("round_bore", "box_bore"):
                cam = mujoco.MjvCamera()
                cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                cam.fixedcamid = mujoco.mj_name2id(
                    robot.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
                fixed_cams[cam_name] = cam
            chase_cam = mujoco.MjvCamera()
            chase_cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            chase_cam.trackbodyid = robot.base_id
            chase_cam.distance, chase_cam.azimuth, chase_cam.elevation = 3.6, 145, -35
        if not a.no_video:
            writer = VideoWriter(a.video, 720 + 576, 480 + 34, fps)
        a.report.parent.mkdir(parents=True, exist_ok=True)
        csv_file = (a.report.with_suffix(".csv")).open("w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=[
            "time", "x", "y", "z", "yaw", "vx", "vy", "yaw_rate", "leg", "cross_track_m"])
        csv_writer.writeheader()

        last_render = -1e9
        t_start = robot.get_state().time

        def append_canvas(canvas):
            nonlocal frames
            if writer is None:
                return
            writer.append(np.asarray(canvas))
            frames += 1

        def render_frame(state, leg_name, leg_path, planned_m):
            nonlocal last_render
            if renderer is None:
                return None
            if "穿圆管" in leg_name:
                cam = fixed_cams["round_bore"]
            elif "穿方涵" in leg_name:
                cam = fixed_cams["box_bore"]
            else:
                chase_cam.lookat[:] = [state.x, state.y, state.z]
                cam = chase_cam
            renderer.update_scene(robot.data, camera=cam)
            chase = Image.fromarray(renderer.render())
            top = top_panel.image(full_route=planned_full, leg_path=leg_path,
                                  trail=trail, pose=(state.x, state.y, state.yaw),
                                  goal=tuple(route["goal"]), leg_name=leg_name)
            canvas = compose(chase, top, state, leg_name, state.time - t_start, planned_m)
            last_render = state.time
            append_canvas(canvas)
            if not a.no_frames and frames > 0 and frames % 100 == 0:
                canvas.save(a.frame_dir / f"frame_{frames:05d}.png")
            if not a.no_frames and frames > 0 and frames % 20 == 0:
                canvas.save(ROOT / "output/nav/current.png")
            return canvas

        def run_leg(leg, *, follower, reverse=False, timeout_extra=10.0):
            nonlocal step_count, total_steps
            leg_path = leg["path"][::-1] if reverse else leg["path"]
            timeout = max(20.0, leg["planned_m"] / max(leg["speed"], 0.05) * 3.0 + timeout_extra)
            start_time = robot.get_state().time
            max_ct = 0.0
            contact = 0
            while True:
                state = robot.get_state()
                vx, vyaw, done = follower.update(state)
                robot.move(vx, 0.0, vyaw)
                state = robot.step()
                step_count += 1
                total_steps += 1
                ct = cross_track_error(leg_path, state.x, state.y)
                max_ct = max(max_ct, ct)
                contact = max(contact, body_contacts(robot))
                if csv_writer is not None and step_count % 5 == 0:
                    csv_writer.writerow({
                        "time": round(state.time, 3), "x": round(state.x, 4),
                        "y": round(state.y, 4), "z": round(state.z, 4),
                        "yaw": round(state.yaw, 4), "vx": round(state.vx, 4),
                        "vy": round(state.vy, 4), "yaw_rate": round(state.yaw_rate, 4),
                        "leg": leg["name"], "cross_track_m": round(ct, 4)})
                if state.time - last_render >= render_dt - 1e-9:
                    render_frame(state, leg["name"], leg_path, nav.planned_total)
                if done:
                    break
                if state.time - start_time > timeout:
                    raise TimeoutError(f"{leg['name']} 超时")
                if state.time - t_start > a.max_seconds:
                    raise TimeoutError("超过总时长上限")
            robot.stop()
            settle_end = robot.get_state().time + 0.4
            while robot.get_state().time < settle_end:
                state = robot.step()
                if state.time - last_render >= render_dt - 1e-9:
                    render_frame(state, leg["name"], leg_path, nav.planned_total)
            final = robot.get_state()
            return {
                "name": leg["name"], "from": leg["from"], "to": leg["to"],
                "planned_m": round(leg["planned_m"], 3),
                "sim_seconds": round(final.time - start_time, 2),
                "max_cross_track_m": round(max_ct, 3),
                "trunk_contact_events": contact,
                "reverse": reverse,
                "final_xy": [round(final.x, 3), round(final.y, 3)],
            }

        state = robot.get_state()
        trail.append((state.x, state.y))
        if writer is not None:
            intro = intro_card(top_panel, planned_full, nav.planned_total,
                               len(nav.legs), (state.x, state.y, state.yaw))
            for _ in range(int(round(3.0 * fps))):
                append_canvas(intro)
        settle_end = state.time + 1.5
        while robot.get_state().time < settle_end:
            state = robot.step()
            if state.time - last_render >= render_dt - 1e-9:
                render_frame(state, "起点准备", nav.legs[0]["path"], nav.planned_total)
        state = robot.get_state()
        robot.stop()
        print(f"起始位姿：x={state.x:.2f} y={state.y:.2f} yaw={math.degrees(state.yaw):.0f}°")
        render_frame(state, "起点", nav.legs[0]["path"], nav.planned_total)

        for i, leg in enumerate(nav.legs):
            nav.leg_index = i
            follower = PathFollower(leg["path"], lookahead=leg["lookahead"],
                                    speed=leg["speed"], goal_tolerance=0.16)
            print(f"[{i + 1:02d}/{len(nav.legs)}] {leg['name']} "
                  f"({leg['planned_m']:.2f} m, {leg['speed']:.2f} m/s)", flush=True)
            result = run_leg(leg, follower=follower)
            report["legs"].append(result)
            state = robot.get_state()
            trail.append((state.x, state.y))
            if leg["return_to_start"]:
                back = PathFollower(leg["path"][::-1], lookahead=0.25, speed=0.22,
                                    reverse=True, goal_tolerance=0.16)
                print(f"    倒车退出死胡同（{leg['planned_m']:.2f} m）", flush=True)
                back_result = run_leg(leg, follower=back, reverse=True)
                back_result["name"] = leg["name"] + "（倒车退出）"
                report["legs"].append(back_result)
                state = robot.get_state()
                trail.append((state.x, state.y))

        final = robot.get_state()
        robot.stop()
        settle_end = final.time + 1.8
        last_canvas = None
        while robot.get_state().time < settle_end:
            state = robot.step()
            if state.time - last_render >= render_dt - 1e-9:
                last_canvas = render_frame(state, "终点", nav.legs[-1]["path"],
                                           nav.planned_total)
        final = robot.get_state()
        goal = tuple(route["goal"])
        error = math.hypot(final.x - goal[0], final.y - goal[1])
        outcome["reached"] = error <= route["finish_tolerance_m"]
        report["result"] = {
            "reached": outcome["reached"],
            "goal_error_m": round(error, 3),
            "finish_tolerance_m": route["finish_tolerance_m"],
            "final_pose": {"x": round(final.x, 3), "y": round(final.y, 3),
                           "z": round(final.z, 3), "yaw": round(final.yaw, 3)},
            "sim_seconds": round(final.time - t_start, 2),
            "trunk_contacts": sum(l["trunk_contact_events"] for l in report["legs"]),
            "min_upright_cos": round(robot.min_upright, 3),
            "min_height_m": round(robot.min_height, 3),
            "policy_steps": total_steps,
            "video_frames": frames,
        }
        state = final
        last_canvas = render_frame(state, "终点", nav.legs[-1]["path"],
                                   nav.planned_total) or last_canvas
        if writer is not None:
            if last_canvas is not None:
                for _ in range(int(round(1.2 * fps))):
                    append_canvas(last_canvas)
            outro = outro_card(top_panel, planned_full, trail,
                               (state.x, state.y, state.yaw), report["result"],
                               nav.planned_total)
            for _ in range(int(round(3.0 * fps))):
                append_canvas(outro)
        report["result"]["video_frames"] = frames
        print(json.dumps(report["result"], indent=2, ensure_ascii=False))
    except Exception as exc:  # 保留已录画面与报告，便于复盘
        outcome["error"] = f"{type(exc).__name__}: {exc}"
        report["result"] = {"reached": False, "error": outcome["error"]}
        print(f"运行中断：{outcome['error']}", file=sys.stderr)
    finally:
        if csv_file is not None:
            try:
                csv_file.close()
            except Exception:
                pass
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(f"报告：{a.report}")
        if writer is not None:
            try:
                writer.close()
            except Exception as exc:
                print(f"视频收尾失败：{exc}", file=sys.stderr)
        try:
            robot.close()
        except Exception:
            pass
        if renderer is not None:
            try:
                renderer.close()
            except Exception:
                pass
        if not a.no_video and a.video.exists():
            print(f"视频：{a.video}")
    return 0 if report.get("result", {}).get("reached") else 1


if __name__ == "__main__":
    raise SystemExit(main())
