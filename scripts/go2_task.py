"""启动现成 Go2 行走策略，演示只使用高层接口的任务算法。"""

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aip.sim_robot import CONFIG, MujocoGo2, validate_velocity_command
from aip.sim_scenes import SCENE_NAMES
from aip.tasks import advance, drive_distance, turn_angle


def main():
    limits = json.loads(CONFIG.read_text())["command_limits"]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--scene", choices=SCENE_NAMES, default="flat")
    p.add_argument("--model", type=Path, help="自定义 Go2 场景，覆盖 --scene")
    p.add_argument("--task", choices=["forward", "turn", "demo", "idle"], default="forward")
    p.add_argument("--distance", type=float, default=0.5, help="前进距离，m；负数为后退")
    p.add_argument("--speed", type=float, default=0.2,
                   help=f"行走速度大小，m/s；前进≤{limits['max'][0]}，后退≤{-limits['min'][0]}")
    p.add_argument("--angle", type=float, default=90, help="转向角度，度；正数为左转")
    p.add_argument("--seconds", type=float, default=5, help="idle 持续的仿真时间")
    p.add_argument("--report", type=Path, help="JSON 报告；同前缀保存 CSV")
    p.add_argument("--screenshot", type=Path)
    p.add_argument("--camera", default="follow", help="截图相机：follow 或场景固定相机")
    a = p.parse_args()
    for name in ["distance", "speed", "angle", "seconds"]:
        if not math.isfinite(getattr(a, name)):
            p.error(f"--{name} 必须有限")
    if a.speed <= 0 or not -180 <= a.angle <= 180 or a.seconds <= 0:
        p.error("速度和时间需大于零，角度在 [-180,180]")
    try:
        validate_velocity_command([a.speed if a.distance >= 0 else -a.speed, 0, 0], limits)
        if a.task == "demo":
            # demo 最后一段固定前进，后退首段也必须满足前进速度上限。
            validate_velocity_command([a.speed, 0, 0], limits)
    except ValueError as exc:
        p.error(str(exc))
    report_path = a.report or ROOT / f"output/controller/{a.scene}-{a.task}.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    trace_path = report_path.with_suffix(".csv")
    results = []
    error = None
    with MujocoGo2(a.model or a.scene, headless=a.headless) as robot, trace_path.open("w", newline="") as f:
        fields = list(asdict(robot.get_state()))
        logger = csv.DictWriter(f, fieldnames=fields)
        logger.writeheader()

        def log(state):
            row = asdict(state)
            row["contacting_feet"] = "|".join(state.contacting_feet)
            row["command"] = "|".join(map(str, state.command))
            logger.writerow(row)

        try:
            # 从配套出生高度落地后，等待零速度策略稳定。
            advance(robot, 2, log)
            if a.task in ("forward", "demo"):
                results.append(drive_distance(robot, a.distance, a.speed, on_step=log))
            if a.task in ("turn", "demo"):
                results.append(turn_angle(robot, math.radians(a.angle), on_step=log))
            if a.task == "demo":
                results.append(drive_distance(robot, 0.3, a.speed, on_step=log))
            if a.task == "idle":
                robot.stop()
                advance(robot, a.seconds, log)
        except (RuntimeError, TimeoutError, InterruptedError) as exc:
            error = str(exc)
            robot.stop()
        final = robot.get_state()
        checks = {
            "completed": error is None,
            "no_physics_warnings": not robot.warnings,
            "upright_throughout": robot.min_upright > 0.85,
            "stopped": math.hypot(final.vx, final.vy) < 0.07 and abs(final.yaw_rate) < 0.1,
            "task_accuracy": all(abs(r.get("error_m", 0)) < 0.08 and abs(r.get("error_rad", 0)) < 0.1
                                 for r in results),
        }
        report = {
            "passed": all(checks.values()), "checks": checks, "error": error,
            "controller": robot.config["name"], "policy_sha256": robot.config["policy_sha256"],
            "source_commit": robot.config["commit"], "checkpoint": robot.config["checkpoint"],
            "model": str(robot.model_path), "results": results,
            "final_state": asdict(final), "warnings": robot.warnings,
            "min_upright_cos": robot.min_upright, "min_base_height_m": robot.min_height,
            "max_abs_torque_Nm": robot.max_abs_torque, "trace": str(trace_path.resolve()),
        }
        if a.screenshot:
            robot.save_screenshot(a.screenshot, camera=a.camera)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
