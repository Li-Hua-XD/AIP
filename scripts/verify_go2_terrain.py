"""验收 go2_rl_lab 在具体楼梯、比赛坡道和固定乱石上的通行。

项目根目录运行：.venv/bin/python scripts/verify_go2_terrain.py
初始复位可选择赛段起点；运动过程始终由策略与足地接触产生。
"""

import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aip.sim_robot import MujocoGo2
from aip.tasks import advance, drive_distance


def main():
    results = []
    cases = [
        ("author_stairs_5cm", "stairs", None, None, 3.0, 0.4, 0.25),
        ("rescue_ramp_12deg", "rescue", (1.0, 4.0, 0.445), math.pi / 2, 2.7, 0.4, 0.25),
        ("rescue_fixed_rubble_4to12cm", "rescue", (8.9, 1.35, 0.445), 0.0, 2.4, 0.4, None),
    ]
    for name, scene, position, yaw, distance, speed, rise in cases:
        with MujocoGo2(scene) as robot:
            if position is not None:
                robot.reset(position=position, yaw=yaw)
            torso_contact = False
            torso_ids = {robot.model.geom(n).id for n in ("base_box", "base_cyl1", "base_sphere1")}

            def observe(_):
                nonlocal torso_contact
                torso_contact |= any(int(g) in torso_ids for contact in robot.data.contact[:robot.data.ncon]
                                     for g in (contact.geom1, contact.geom2))

            error, task = None, None
            try:
                advance(robot, 2, observe)
                start = robot.get_state()
                task = drive_distance(robot, distance, speed, timeout=30, on_step=observe)
            except (RuntimeError, TimeoutError) as exc:
                error = str(exc)
            final = robot.get_state()
            checks = {
                "completed": error is None,
                "distance_error_below_8cm": task is not None and abs(task["error_m"]) < 0.08,
                "no_torso_contact": not torso_contact,
                "no_overturn": robot.min_upright > 0.7,
                "no_physics_warnings": not robot.warnings,
                "stopped": math.hypot(final.vx, final.vy) < 0.07 and abs(final.yaw_rate) < 0.1,
            }
            if rise is not None:
                checks["climbed_above_start"] = error is None and final.z - start.z > rise
            results.append({"name": name, "passed": all(checks.values()), "checks": checks,
                            "error": error, "task": task, "final_state": asdict(final),
                            "min_upright_cos": robot.min_upright, "warnings": robot.warnings})
            print(name, "PASS" if results[-1]["passed"] else "FAIL", flush=True)
    report = {"controller": robot.config["name"], "policy_sha256": robot.config["policy_sha256"],
              "passed": all(r["passed"] for r in results), "cases": results}
    path = ROOT / "output/controller/terrain-validation.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
