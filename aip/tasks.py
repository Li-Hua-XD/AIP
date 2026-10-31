"""可复用的任务逻辑，只依赖 move/stop/step/get_state 接口。

这里使用后端提供的位置/方向；仿真为真值，实机需要有效的定位反馈。
不包含避障、路线规划或整场比赛通关策略。
"""

import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def advance(robot, seconds, on_step=None):
    end = robot.get_state().time + seconds
    while robot.get_state().time < end - 1e-9:
        if not robot.is_running():
            raise InterruptedError("仿真窗口已关闭")
        state = robot.step()
        if on_step:
            on_step(state)


def drive_distance(robot, distance=0.5, speed=0.2, timeout=20, on_step=None):
    """沿起始机身前向行走指定距离，靠位置反馈停车；允许负距离后退。"""
    if not all(map(math.isfinite, [distance, speed, timeout])) or speed <= 0 or timeout <= 0:
        raise ValueError("距离必须有限，速度和超时必须为有限正数")
    start = robot.get_state()
    direction = 1 if distance >= 0 else -1
    progress = 0.0
    try:
        while abs(distance) - direction * progress > 0.025:
            state = robot.get_state()
            if state.time - start.time >= timeout:
                raise TimeoutError("行走未在限定仿真时间内到达目标")
            if not robot.is_running():
                raise InterruptedError("仿真窗口已关闭")
            # 使用位置反馈停车，沿初始朝向行走，不依赖预估的速度×时间。
            vx = direction * speed
            yaw_rate = max(-0.4, min(0.4, 1.5 * wrap_angle(start.yaw - state.yaw)))
            robot.move(vx, 0.0, yaw_rate)
            state = robot.step()
            progress = ((state.x - start.x) * math.cos(start.yaw)
                        + (state.y - start.y) * math.sin(start.yaw))
            if on_step:
                on_step(state)
    finally:
        robot.stop()
    advance(robot, 1.5, on_step)
    final = robot.get_state()
    projection = ((final.x - start.x) * math.cos(start.yaw)
                  + (final.y - start.y) * math.sin(start.yaw))
    return {"task": "drive_distance", "requested_m": distance, "actual_m": projection,
            "error_m": projection - distance, "duration_s": final.time - start.time}


def turn_angle(robot, angle=math.pi / 2, max_rate=0.5, timeout=20, on_step=None):
    """相对当前方向转动（-pi 到 pi），用方向反馈停车。"""
    if not all(map(math.isfinite, [angle, max_rate, timeout])) or abs(angle) > math.pi or max_rate <= 0 or timeout <= 0:
        raise ValueError("角度需在 [-pi, pi]，角速度上限和超时须为有限正数")
    start = robot.get_state()
    goal = wrap_angle(start.yaw + angle)
    try:
        while True:
            state = robot.get_state()
            error = wrap_angle(goal - state.yaw)
            if abs(error) < 0.04:
                break
            if state.time - start.time >= timeout:
                raise TimeoutError("转向未在限定仿真时间内完成")
            if not robot.is_running():
                raise InterruptedError("仿真窗口已关闭")
            # 以限定角速度闭环转动，达到目标后请求零速度。
            rate = math.copysign(max_rate, error)
            robot.move(0.0, 0.0, rate)
            state = robot.step()
            if on_step:
                on_step(state)
    finally:
        robot.stop()
    advance(robot, 1.5, on_step)
    final = robot.get_state()
    return {"task": "turn_angle", "requested_rad": angle,
            "actual_rad": wrap_angle(final.yaw - start.yaw),
            "error_rad": wrap_angle(final.yaw - goal), "duration_s": final.time - start.time}
