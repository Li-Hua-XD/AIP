"""Local Go2 standing demo and MuJoCo installation smoke test (no robot network)."""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"


def main():
    parser = argparse.ArgumentParser(description="Go2 MuJoCo 本地站立演示")
    parser.add_argument("--headless", action="store_true", help="不打开窗口")
    parser.add_argument("--seconds", type=float, help="运行的仿真秒数；窗口模式默认持续运行")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--screenshot", type=Path, help="保存最终状态渲染图")
    parser.add_argument("--report", type=Path, help="保存验证结果 JSON")
    parser.add_argument("--camera", default="follow", help="follow 或模型中的固定相机名称（overview/top）")
    parser.add_argument("--width", type=int, default=640, help="截图宽度")
    parser.add_argument("--height", type=int, default=480, help="截图高度")
    args = parser.parse_args()
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds 必须是有限正数")
    if args.width <= 0 or args.height <= 0:
        parser.error("截图宽高必须是正整数")
    seconds = args.seconds if args.seconds is not None else (10 if args.headless else float("inf"))
    if not args.model.is_file():
        parser.error(f"模型不存在：{args.model}；请按 docs/mujoco_setup.md 下载官方模型")

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key < 0:
        parser.error("模型必须提供 home 关键帧")
    mujoco.mj_resetDataKeyframe(model, data, key)
    # 官方模型的关节顺序与电机顺序不同，按 transmission 查找，不能直接切片。
    joints = model.actuator_trnid[:, 0]
    qpos_indices = model.jnt_qposadr[joints]
    qvel_indices = model.jnt_dofadr[joints]
    target = data.qpos[qpos_indices].copy()
    data.ctrl[:] = 0  # 上游 home.ctrl 是角度；本模型使用力矩电机。
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    camera.lookat[:] = data.xpos[base_id]
    camera.distance = 1.7
    camera.azimuth = 135
    camera.elevation = -20
    if args.camera != "follow":
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
        if camera_id < 0:
            parser.error(f"模型中没有名为 {args.camera} 的相机")
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        camera.fixedcamid = camera_id
    started = time.perf_counter()
    steps = 0

    def step():
        nonlocal steps
        torque = 50.0 * (target - data.qpos[qpos_indices]) - 3.0 * data.qvel[qvel_indices]
        data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        mujoco.mj_step(model, data)
        steps += 1
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError("仿真状态出现非有限值")

    if args.headless:
        while data.time < seconds:
            step()
    else:
        from mujoco import viewer as mj_viewer

        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.type = camera.type
            viewer.cam.fixedcamid = camera.fixedcamid
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
            while viewer.is_running() and data.time < seconds:
                # 每帧推进约 1/60 秒物理时间。
                frame_end = min(data.time + 1 / 60, seconds)
                while data.time < frame_end:
                    step()
                viewer.sync()
                delay = data.time - (time.perf_counter() - started)
                if delay > 0:
                    time.sleep(delay)

    elapsed = time.perf_counter() - started
    if args.screenshot:
        from PIL import Image

        args.screenshot.parent.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(args.screenshot)

    warnings = {mujoco.mjtWarning(i).name: int(w.number)
                for i, w in enumerate(data.warning) if w.number}
    standing = bool(data.xpos[base_id, 2] > 0.18 and data.xmat[base_id].reshape(3, 3)[2, 2] > 0.9)
    report = {
        "mujoco": mujoco.__version__, "model": str(args.model.resolve()),
        "actuators": model.nu, "nq": model.nq, "nv": model.nv,
        "steps": steps, "simulation_seconds": float(data.time),
        "wall_seconds": round(elapsed, 4), "base_height_m": float(data.qpos[2]),
        "standing": standing, "warnings": warnings,
        "screenshot": str(args.screenshot.resolve()) if args.screenshot else None,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if warnings or not standing:
        raise RuntimeError("站立验收未通过，详见上方状态")


if __name__ == "__main__":
    main()
