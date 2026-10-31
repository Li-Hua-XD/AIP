"""检查真实物理行走、方向转换、反馈任务、指令过期与停车；不连接实机。"""

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aip.sim_robot import MujocoGo2
from aip.tasks import advance, drive_distance, turn_angle


def main():
    checks = []

    def check(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    with MujocoGo2() as robot:
        advance(robot, 2)
        patterns = set()

        def observe(state):
            patterns.add(state.contacting_feet)

        result = drive_distance(robot, 0.5, on_step=observe)
        check("forward_0.5m", abs(result["error_m"]) < 0.08, result)
        check("feet_lift_and_contact", any(0 < len(p) < 4 for p in patterns),
              {"contact_patterns": [list(p) for p in sorted(patterns)]})
        state = robot.get_state()
        check("forward_stopped", math.hypot(state.vx, state.vy) < 0.07,
              {"vx": state.vx, "vy": state.vy})
        # 保留实际策略历史，验证组合任务，而非只验证从同一初态开始的孤立动作。
        result = turn_angle(robot, math.pi / 2)
        check("left_turn_90deg", abs(result["error_rad"]) < 0.1, result)
        state = robot.get_state()
        check("turn_stopped", abs(state.yaw_rate) < 0.1, {"yaw_rate": state.yaw_rate})
        result = drive_distance(robot, 0.3, on_step=observe)
        check("forward_after_turn", abs(result["error_m"]) < 0.08, result)
        check("flat_no_warnings", not robot.warnings, robot.warnings)
        check("flat_upright", robot.min_upright > 0.85, {"min_upright_cos": robot.min_upright})

    with MujocoGo2("rescue") as robot:
        advance(robot, 2)
        start = robot.get_state()
        result = drive_distance(robot, 0.3)
        end = robot.get_state()
        check("rescue_body_frame_forward", abs(result["error_m"]) < 0.08
              and end.y - start.y > 0.2 and abs(end.x - start.x) < 0.1,
              {"result": result, "dx": end.x - start.x, "dy": end.y - start.y})
        check("rescue_no_warnings", not robot.warnings, robot.warnings)

    with MujocoGo2() as robot:
        advance(robot, 2)
        robot.move(0.2, 0, 0)
        advance(robot, 2)  # 故意不续发，不能无限执行旧速度。
        state = robot.get_state()
        check("command_expiry", state.command == (0.0, 0.0, 0.0)
              and math.hypot(state.vx, state.vy) < 0.07,
              {"command": state.command, "vx": state.vx, "vy": state.vy})
        # 上游平地训练范围允许非对称前后速度，边界命令只提交、不推进仿真。
        accepted = 0
        for command in [(4.2, 0, 0), (-4.5, 0, 0), (0, 1, 0), (0, -1, 0),
                        (0, 0, 3), (0, 0, -3), (4.0, 0, 0), (1.0, 0, 0)]:
            robot.move(*command)
            accepted += robot.get_state().command == command
        check("policy_command_boundaries_accepted", accepted == 8, {"accepted": accepted})
        robot.stop()
        invalid_commands = [(float("nan"), 0, 0), (float("inf"), 0, 0),
                            (0, float("-inf"), 0), (4.21, 0, 0), (-4.51, 0, 0),
                            (0, 1.01, 0), (0, -1.01, 0), (0, 0, 3.01), (0, 0, -3.01)]
        rejected = 0
        for command in invalid_commands:
            try:
                robot.move(*command)
            except ValueError:
                rejected += 1
        check("invalid_commands_rejected", rejected == len(invalid_commands)
              and robot.get_state().command == (0.0, 0.0, 0.0), {"rejected": rejected})

        # 隐状态复位必须与物理复位一致：同一命令序列再次运行应得到相同轨迹。
        trajectory = []
        for _ in range(2):
            robot.reset()
            advance(robot, 2)
            robot.run_velocity(0.2, 0, 0, 0.5)
            s = robot.get_state()
            trajectory.append([s.x, s.y, s.z, s.yaw])
        check("policy_history_reset", max(abs(a - b) for a, b in zip(*trajectory)) < 1e-8,
              {"poses": trajectory})

    report = {"controller": robot.config["name"], "source_commit": robot.config["commit"],
              "policy_sha256": robot.config["policy_sha256"],
              "passed": all(c["passed"] for c in checks), "checks": checks}
    path = ROOT / "output/controller/validation.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
