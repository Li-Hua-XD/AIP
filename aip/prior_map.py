"""先验地图（prior_map）的构建、校验与全局规划。

背图版使用：地图由场地勘测/建模离线生成为只读文件，导航进程只加载该文件，
不读取实时场景几何。地图为 2.5D 占据栅格：

- ``free``      0=障碍，1=可通行（未膨胀，机器人中心不可进入膨胀后的障碍）
- ``elevation`` 每个格子的可行走面高度，m
- 起点/终点区域属于显式任务配置，不写进地图文件。

生成时用球形探针在机身高度做碰撞检测：墙体占据格点，隧道内腔、坡道面与
低矮乱石保持可通行；生成结果带 SHA-256，导航进程校验后才加载。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROBOT_RADIUS = 0.28  # 机身半宽加余量，m
PROBE_RADIUS = 0.08  # 机身高度碰撞探针半径，m


def _ray_down(model, data, x, y, z0, geomid, geomgroup=None):
    """从 (x, y, z0) 向下投射，返回命中点高度；未命中返回 None。"""
    import mujoco

    dist = mujoco.mj_ray(
        model, data,
        np.array([x, y, z0], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
        geomgroup, 1, -1, geomid,
    )
    return None if dist < 0 else z0 - float(dist)


def build_map(model_path, resolution=0.05, *, width=None, depth=None):
    """从 MJCF（纯场地模型）离线生成先验地图。

    调用方应保证 model_path 是对应赛场的静态场地模型；生成结果落盘后，
    导航进程不再读取几何文件。

    障碍判定：在机身高度放一个半径 0.08 m 的球形探针做碰撞检测。探针中心
    取局部可行走面以上 0.18 m，因此墙体（厚 0.12 m 以上）必然与探针相交，
    而隧道内腔、坡道面和低矮乱石保持可通行；隧道顶板不会被当作地面。
    """
    import mujoco

    spec = mujoco.MjSpec.from_file(str(model_path))
    probe_body = spec.worldbody.add_body(name="__aip_probe", pos=[0.0, 0.0, 1.0])
    probe_body.add_freejoint()
    probe_geom = probe_body.add_geom()
    probe_geom.name = "__aip_probe"
    probe_geom.type = mujoco.mjtGeom.mjGEOM_SPHERE
    probe_geom.size = [0.08, 0.0, 0.0]
    probe_geom.contype = 1
    probe_geom.conaffinity = 1
    probe_geom.group = 5
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    probe_gid = model.geom("__aip_probe").id

    floor_ids = [i for i in range(model.ngeom)
                 if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX
                 and float(model.geom_size[i][2]) <= 0.07
                 and abs(float(model.geom_pos[i][2])) < 0.1]
    if not floor_ids:
        raise ValueError("场地模型中找不到地面 geom")
    floor = floor_ids[0]
    sx, sy = model.geom_size[floor][:2]
    cx, cy = model.geom_pos[floor][:2]
    width = width or 2 * float(sx)
    depth = depth or 2 * float(sy)
    origin = (float(cx - sx), float(cy - sy))

    nx = int(round(width / resolution))
    ny = int(round(depth / resolution))
    free = np.zeros((ny, nx), dtype=bool)
    elevation = np.full((ny, nx), np.nan, dtype=np.float32)
    ceiling = np.full((ny, nx), np.nan, dtype=np.float32)
    geomid = np.zeros(1, dtype=np.int32)
    groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)  # 排除探针所在组
    qadr = model.jnt_qposadr[model.body("__aip_probe").jntadr[0]]

    for j in range(ny):
        y = origin[1] + (j + 0.5) * resolution
        for i in range(nx):
            x = origin[0] + (i + 0.5) * resolution
            ground = _ray_down(model, data, x, y, 0.5, geomid, groups)
            base = 0.0 if ground is None else ground
            data.qpos[qadr:qadr + 3] = [x, y, base + 0.18]
            data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
            mujoco.mj_forward(model, data)
            blocked = any(probe_gid in (c.geom1, c.geom2)
                          for c in data.contact[:data.ncon])
            if blocked:
                continue
            free[j, i] = True
            elevation[j, i] = max(0.0, base)
            top = _ray_down(model, data, x, y, 3.0, geomid, groups)
            if top is not None and top > elevation[j, i] + 0.05:
                ceiling[j, i] = top

    return {
        "free": free,
        "elevation": elevation,
        "ceiling": ceiling,
        "resolution": float(resolution),
        "origin": [float(origin[0]), float(origin[1])],
        "width_m": float(width),
        "depth_m": float(depth),
    }


def inflate(free, resolution, radius=ROBOT_RADIUS):
    """按机器人半径膨胀障碍；返回可通行掩码。"""
    steps = int(math.ceil(radius / resolution))
    offsets = []
    for dj in range(-steps, steps + 1):
        for di in range(-steps, steps + 1):
            if di == 0 and dj == 0:
                continue
            if (di * resolution) ** 2 + (dj * resolution) ** 2 <= radius ** 2:
                offsets.append((dj, di))
    blocked = ~free
    out = free.copy()
    ny, nx = free.shape
    for dj, di in offsets:
        src = blocked[max(0, -dj):ny - max(0, dj), max(0, -di):nx - max(0, di)]
        dst = out[max(0, dj):ny - max(0, -dj), max(0, di):nx - max(0, -di)]
        dst &= ~src
    return out


def _bresenham_cells(a, b):
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def line_of_sight(passable, a, b):
    return all(passable[j, i] for i, j in _bresenham_cells(a, b))


def simplify_path(passable, cells):
    """在可通行栅格上做视线简化，保留首尾与必要拐点。"""
    if len(cells) <= 2:
        return list(cells)
    out = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        nxt = anchor + 1
        for probe in range(len(cells) - 1, anchor, -1):
            if line_of_sight(passable, cells[anchor], cells[probe]):
                nxt = probe
                break
        out.append(cells[nxt])
        anchor = nxt
    return out


def astar(passable, start, goal):
    """8 邻接 A*，禁止贴角穿越；返回格子坐标列表。"""
    import heapq

    h, w = passable.shape
    if not (0 <= start[1] < h and 0 <= start[0] < w):
        return None
    if not (0 <= goal[1] < h and 0 <= goal[0] < w):
        return None
    if not passable[start[1], start[0]] or not passable[goal[1], goal[0]]:
        return None

    def octile(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    openh = [(octile(start, goal), 0.0, start)]
    best = {start: 0.0}
    parent = {start: None}
    closed = set()
    while openh:
        _, cost, cur = heapq.heappop(openh)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < w and 0 <= nxt[1] < h):
                continue
            if not passable[nxt[1], nxt[0]]:
                continue
            if dx and dy:
                if not (passable[cur[1], nxt[0]] and passable[nxt[1], cur[0]]):
                    continue  # 不切墙角
                step = math.sqrt(2)
            else:
                step = 1.0
            new_cost = cost + step
            if new_cost < best.get(nxt, math.inf):
                best[nxt] = new_cost
                parent[nxt] = cur
                heapq.heappush(openh, (new_cost + octile(nxt, goal), new_cost, nxt))
    return None


@dataclass
class PriorMap:
    free: np.ndarray
    elevation: np.ndarray
    resolution: float
    origin: tuple[float, float]
    ceiling: np.ndarray | None = None
    meta: dict = field(default_factory=dict)
    inflated: np.ndarray | None = None

    @property
    def nx(self):
        return self.free.shape[1]

    @property
    def ny(self):
        return self.free.shape[0]

    @property
    def width_m(self):
        return self.nx * self.resolution

    @property
    def depth_m(self):
        return self.ny * self.resolution

    def world_to_grid(self, x, y):
        i = int(math.floor((x - self.origin[0]) / self.resolution))
        j = int(math.floor((y - self.origin[1]) / self.resolution))
        return i, j

    def grid_to_world(self, i, j):
        return (self.origin[0] + (i + 0.5) * self.resolution,
                self.origin[1] + (j + 0.5) * self.resolution)

    def passable(self):
        if self.inflated is None:
            self.inflated = inflate(self.free, self.resolution)
        return self.inflated

    def plan(self, start_xy, goal_xy):
        """返回世界坐标路径；不可达返回 None。"""
        start = self.world_to_grid(*start_xy)
        goal = self.world_to_grid(*goal_xy)
        cells = astar(self.passable(), start, goal)
        if cells is None:
            return None
        centers = [self.grid_to_world(*c) for c in cells]
        keep = simplify_path(self.passable(), cells)
        return [self.grid_to_world(*c) for c in keep]

    def path_length(self, path):
        return sum(math.dist(a, b) for a, b in zip(path, path[1:]))

    def save(self, path, *, source, extra=None):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = {
            "free": self.free.astype(np.uint8),
            "elevation": np.nan_to_num(self.elevation, nan=-1.0).astype(np.float32),
            "resolution": np.array(self.resolution),
            "origin": np.array(self.origin),
        }
        if self.ceiling is not None:
            arrays["ceiling"] = np.nan_to_num(self.ceiling, nan=-1.0).astype(np.float32)
        np.savez_compressed(path, **arrays)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        meta = {
            "format": "aip-prior-map-2.5d/v1",
            "sha256": digest,
            "resolution_m": self.resolution,
            "origin_m": list(self.origin),
            "size_cells": [self.nx, self.ny],
            "free_cells": int(self.free.sum()),
            "robot_radius_m": ROBOT_RADIUS,
            "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "source": str(source),
        }
        if extra:
            meta.update(extra)
        path.with_suffix(".json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
        return meta

    @classmethod
    def load(cls, path, *, verify_sha256=True):
        path = Path(path)
        meta_path = path.with_suffix(".json")
        meta = json.loads(meta_path.read_text())
        if verify_sha256:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != meta["sha256"]:
                raise ValueError("先验地图哈希与元数据不符，拒绝加载")
        z = np.load(path)
        m = cls(
            free=z["free"].astype(bool),
            elevation=z["elevation"].astype(np.float64),
            resolution=float(z["resolution"]),
            origin=tuple(float(v) for v in z["origin"]),
            ceiling=z["ceiling"].astype(np.float64) if "ceiling" in z else None,
            meta=meta,
        )
        m.elevation[m.elevation < -0.5] = np.nan
        if m.ceiling is not None:
            m.ceiling[m.ceiling < -0.5] = np.nan
        return m
