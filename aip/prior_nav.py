"""背图版导航的路径跟踪控制器。

只依赖 move/stop/step/get_state 接口；定位输入为状态估计（本演示用仿真真值，
实机需替换为雷达/IMU 定位）。包含：

- 纯跟踪（pure pursuit）：沿全局路径给出 vx/vyaw
- 倒车跟踪：死胡同退出时以后向为运动方向，保持低速度原路退回
- 进度/超时/碰撞与终止判定
"""

from __future__ import annotations

import math

import numpy as np

from aip.tasks import wrap_angle


class PathFollower:
    """在一条折线路径上做纯跟踪，可前进或倒车。"""

    def __init__(self, path, *, lookahead=0.45, speed=0.4, reverse=False,
                 goal_tolerance=0.18, yaw_gain=2.2, max_yaw_rate=0.9):
        if len(path) < 2:
            raise ValueError("路径至少需要两个点")
        self.path = np.asarray(path, dtype=float)
        self.lookahead = float(lookahead)
        self.speed = float(speed)
        self.reverse = bool(reverse)
        self.goal_tolerance = float(goal_tolerance)
        self.yaw_gain = float(yaw_gain)
        self.max_yaw_rate = float(max_yaw_rate)
        seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.total = float(self.cum[-1])
        self.index = 0
        self.reached = False
        self.last_progress_s = 0.0

    def _project(self, x, y, start_index):
        """在窗口内找最近点，返回(弧长, 最近点, 段索引, 段内比例)。"""
        best = (math.inf, 0.0, start_index, 0.0)
        hi = min(len(self.path) - 1, start_index + 120)
        for k in range(start_index, hi):
            a, b = self.path[k], self.path[k + 1]
            ab = b - a
            denom = float(ab @ ab)
            t = 0.0 if denom <= 1e-12 else float(np.clip(((x - a[0]) * ab[0] +
                                                         (y - a[1]) * ab[1]) / denom, 0, 1))
            px, py = a + t * ab
            dist = math.hypot(x - px, y - py)
            if dist < best[0]:
                best = (dist, float(self.cum[k] + t * math.sqrt(denom)), k, t)
        return best

    def target_point(self, s):
        s = min(max(s, 0.0), self.total)
        k = int(np.searchsorted(self.cum, s, side="right") - 1)
        k = min(max(k, 0), len(self.path) - 2)
        span = self.cum[k + 1] - self.cum[k]
        t = 0.0 if span <= 1e-9 else (s - self.cum[k]) / span
        return self.path[k] + t * (self.path[k + 1] - self.path[k])

    def update(self, state):
        """返回 (vx, vyaw, done)。state 需含 x/y/yaw。"""
        x, y = state.x, state.y
        dist, s, k, _ = self._project(x, y, self.index)
        self.index = max(self.index, k)
        remaining = self.total - s
        if remaining <= max(self.goal_tolerance, 0.12) and dist < 0.45:
            self.reached = True
            return 0.0, 0.0, True
        look_s = min(self.total, s + self.lookahead)
        tx, ty = self.target_point(look_s)
        direction = state.yaw + (math.pi if self.reverse else 0.0)
        err = wrap_angle(math.atan2(ty - y, tx - x) - direction)
        speed = self.speed
        if remaining < 1.2:
            speed = max(0.16, self.speed * (0.35 + 0.65 * remaining / 1.2))
        if abs(err) > 1.2:
            vx = 0.0
            vyaw = math.copysign(min(self.max_yaw_rate, 0.7), err)
        else:
            vx = speed * (1.0 - 0.55 * min(1.0, abs(err) / 1.2))
            vyaw = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_gain * err))
        vx = -vx if self.reverse else vx
        return vx, vyaw, False


def cross_track_error(path, x, y):
    """点到折线路径的最短距离，用于报告。"""
    pts = np.asarray(path, dtype=float)
    if len(pts) < 2:
        return math.hypot(x - pts[0, 0], y - pts[0, 1])
    best = math.inf
    for a, b in zip(pts, pts[1:]):
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom <= 1e-12 else float(np.clip(((x - a[0]) * ab[0] +
                                                     (y - a[1]) * ab[1]) / denom, 0, 1))
        px, py = a + t * ab
        best = min(best, math.hypot(x - px, y - py))
    return best
