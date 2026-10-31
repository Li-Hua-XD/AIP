"""Go2 的本地速度控制适配器：任务层不需要访问电机或策略网络。

使用 wty-yy/go2_rl_lab 作者发布的混合地形 MoE-CTS 策略；并非出厂 sport 服务。
网络协议不参与本模块；此后端只驱动自己的 MuJoCo MjData。
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

from aip.sim_scenes import SCENE_NAMES, make_scene

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/go2_controller.json"


def validate_velocity_command(command, limits):
    """校验机身速度指令；上下界允许前进与后退范围不同。"""
    command = np.asarray(command, dtype=float)
    if command.shape != (3,) or not np.all(np.isfinite(command)):
        raise ValueError("速度指令必须为三个有限数 [vx, vy, vyaw]")
    lower = np.asarray(limits["min"], dtype=float)
    upper = np.asarray(limits["max"], dtype=float)
    if np.any(command < lower) or np.any(command > upper):
        raise ValueError(f"速度超出策略指令范围：min={lower.tolist()}，max={upper.tolist()}"
                         "（依次为前后 m/s、左右 m/s、转向 rad/s）")
    return command


@dataclass(frozen=True)
class RobotState:
    time: float
    x: float
    y: float
    z: float
    yaw: float
    vx: float  # 机身坐标系，m/s
    vy: float
    yaw_rate: float
    upright_cos: float
    contacting_feet: tuple[str, ...]
    command: tuple[float, float, float]


class MujocoGo2:
    """move/stop/get_state/step 接口，供任务算法调用。

    step() 推进 20 ms 仿真；move() 必须持续更新，超过 0.3 s 自动转零速度。
    get_state() 是仿真真值，实机后端需替换为有效的传感器/状态估计。
    """

    def __init__(self, scene="flat", *, headless=True, config=CONFIG):
        self.config = json.loads(Path(config).read_text())
        c = self.config
        policy = ROOT / c["policy_path"]
        if not policy.is_file():
            raise FileNotFoundError("缺少行走模型；先运行 bash scripts/setup_go2_controller.sh")
        if hashlib.sha256(policy.read_bytes()).hexdigest() != c["policy_sha256"]:
            raise ValueError("行走模型 SHA256 与配置不符，请重新运行安装脚本或核查模型来源")
        ort.disable_telemetry_events()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy), sess_options=options,
                                          providers=["CPUExecutionProvider"])
        history_dim = sum(c["feature_dims"]) * c["history_length"]
        if self.policy.get_inputs()[0].shape != [1, history_dim] or self.policy.get_outputs()[0].shape != [1, 12]:
            raise ValueError(f"策略必须为配套的 {history_dim} 维历史观测、12 维动作模型")
        self.input_name = self.policy.get_inputs()[0].name
        if scene in SCENE_NAMES:
            self.model_path, payload = make_scene(scene, c)
            self.model = mujoco.MjModel.from_xml_string(payload)
        else:
            self.model_path = Path(scene).resolve()
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        m = self.model
        m.opt.timestep = c["physics_dt"]
        m.opt.integrator = getattr(mujoco.mjtIntegrator, "mjINT_" + c["integrator"].upper())
        m.opt.solver = getattr(mujoco.mjtSolver, "mjSOL_" + c["solver"].upper())
        m.opt.iterations = c["solver_iterations"]
        m.opt.ls_iterations = c["line_search_iterations"]
        self.data = mujoco.MjData(m)
        self.base_id = m.body(c["base_body"]).id
        joints = np.array([m.joint(n).id for n in c["policy_joint_names"]])
        if m.nu != 12 or np.any(m.jnt_type[joints] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("仅支持 12 个铰链关节的 Go2 模型")
        self.qi = m.jnt_qposadr[joints]
        self.vi = m.jnt_dofadr[joints]
        self.ai = np.array([self._actuator_for(j) for j in joints])
        if np.any(m.actuator_trntype[self.ai] != mujoco.mjtTrn.mjTRN_JOINT) or not np.allclose(m.actuator_gear[self.ai, 0], 1):
            raise ValueError("需要直接关节、gear=1 的 Go2 力矩电机")
        self.joint_ranges = m.jnt_range[joints]
        self.default = np.array(c["default_joint_pos"], dtype=np.float64)
        ratio = c["policy_dt"] / c["physics_dt"]
        if not np.isclose(ratio, round(ratio)) or ratio < 1:
            raise ValueError("策略周期必须是物理步长的正整数倍")
        self.decimation = round(ratio)
        self.warmup_steps = round(c["warmup_seconds"] / c["physics_dt"])
        self.feet = {m.geom(leg).id: leg for leg in ("FR", "FL", "RR", "RL")}
        self.viewer = None
        self._closed = False
        self.reset()
        self._wall_start = time.perf_counter()
        if not headless:
            from mujoco import viewer

            self.viewer = viewer.launch_passive(m, self.data)
            self.viewer.cam.distance = 2.2
            self.viewer.cam.azimuth = 135
            self.viewer.cam.elevation = -20

    def _actuator_for(self, joint):
        ids = np.flatnonzero(self.model.actuator_trnid[:, 0] == joint)
        if len(ids) != 1:
            raise ValueError("每个策略关节必须由唯一电机驱动")
        return int(ids[0])

    def reset(self, *, position=None, yaw=None):
        """开始新试验并清空策略历史；可选出生位置/方向用于分段地形验收。"""
        if position is not None and (np.shape(position) != (3,) or not np.all(np.isfinite(position))):
            raise ValueError("出生位置必须包含三个有限数")
        if yaw is not None and not math.isfinite(yaw):
            raise ValueError("出生方向必须有限")
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        if position is not None:
            d.qpos[:3] = position
        if yaw is not None:
            d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[self.qi] = self.default
        d.ctrl[:] = 0
        self.target = self.default.copy()
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.observation_history = np.zeros((self.config["history_length"], 45), dtype=np.float32)
        self._command = np.zeros(3)
        self._expires_at = 0.0
        self.steps = 0
        self.max_abs_torque = 0.0
        self.min_upright = 1.0
        self.min_height = float(d.qpos[2])
        self._wall_start = time.perf_counter()
        mujoco.mj_forward(m, d)

    def move(self, vx, vy=0.0, vyaw=0.0):
        """提交机身坐标速度；限幅范围见 configs/go2_controller.json。"""
        command = validate_velocity_command([vx, vy, vyaw], self.config["command_limits"])
        self._command = command
        self._expires_at = self.data.time + self.config["command_timeout"]

    def stop(self):
        """请求零速度；通过 get_state() 确认实际停稳。"""
        self.move(0.0, 0.0, 0.0)

    def _effective_command(self):
        if self.steps < self.warmup_steps or self.data.time >= self._expires_at:
            return np.zeros(3)
        return self._command.copy()

    def _body_velocity(self):
        velocity = np.zeros(6)
        # MuJoCo 提供机身坐标系的角速度、线速度，避免混用 freejoint 的坐标约定。
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.base_id, velocity, 1)
        return velocity

    def _update_policy(self):
        d, c = self.data, self.config
        gravity = d.xmat[self.base_id].reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
        # 配套 checkpoint 的固定顺序：omega, gravity, command, q_rel, dq, last_action。
        observation = np.concatenate([
            d.qvel[3:6] * c["angular_velocity_scale"], gravity,
            self._effective_command() * c["command_scale"],
            d.qpos[self.qi] - self.default,
            d.qvel[self.vi] * c["joint_velocity_scale"], self.previous_action,
        ]).astype(np.float32)
        # 对齐作者 exporter：按观测项分块，每块保存由旧到新的十帧历史。
        # 不能直接把十个完整 45 维帧依次 flatten，那会改变网络输入的顺序。
        self.observation_history[:-1] = self.observation_history[1:]
        self.observation_history[-1] = observation
        blocks = []
        offset = 0
        for dimension in c["feature_dims"]:
            blocks.append(self.observation_history[:, offset:offset + dimension].reshape(-1))
            offset += dimension
        history = np.concatenate(blocks)
        action = self.policy.run(None, {self.input_name: history[None]})[0][0]
        if not np.all(np.isfinite(action)):
            raise RuntimeError("策略输出无效")
        action = np.clip(action, -c["action_clip"], c["action_clip"])
        self.previous_action = action.copy()
        self.target = self.default + c["action_scale"] * action

    def _motor_effort(self, torque, velocity):
        """作者 Go2HV 电机模型：同向/反向分支，以及高速力矩衰减。"""
        c = self.config["motor"]
        cap = np.where(velocity * torque > 0, c["Y1"], c["Y2"])
        cap *= np.clip((c["X2"] - np.abs(velocity)) / (c["X2"] - c["X1"]), 0, 1)
        return np.clip(torque, -cap, cap)

    def step(self):
        """推进一个任务/策略周期（默认 0.02 仿真秒），返回新状态。"""
        if self._closed:
            raise RuntimeError("仿真实例已关闭")
        m, d, c = self.model, self.data, self.config
        for _ in range(self.decimation):
            torque = c["kp"] * (self.target - d.qpos[self.qi]) - c["kd"] * d.qvel[self.vi]
            effort = self._motor_effort(torque, d.qvel[self.vi])
            lo = np.where(m.actuator_ctrllimited[self.ai], m.actuator_ctrlrange[self.ai, 0], -np.inf)
            hi = np.where(m.actuator_ctrllimited[self.ai], m.actuator_ctrlrange[self.ai, 1], np.inf)
            d.ctrl[self.ai] = np.clip(effort, lo, hi)
            self.max_abs_torque = max(self.max_abs_torque, float(np.abs(d.ctrl).max()))
            previous_time = d.time
            mujoco.mj_step(m, d)
            if d.time <= previous_time or not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))):
                raise RuntimeError("物理时间重置或状态无效")
            mujoco.mj_forward(m, d)
            self.steps += 1
            if self.steps >= self.warmup_steps and self.steps % self.decimation == 0:
                self._update_policy()
            self.min_upright = min(self.min_upright, float(d.xmat[self.base_id, 8]))
            self.min_height = min(self.min_height, float(d.xpos[self.base_id, 2]))
            if self.warnings:
                raise RuntimeError(f"MuJoCo 物理警告：{self.warnings}")
            if d.xmat[self.base_id, 8] < 0.5 or d.xpos[self.base_id, 2] < 0.12:
                raise RuntimeError("检测到翻倒或机身过低，停止本次试验")
        if self.viewer is not None:
            self.viewer.cam.lookat[:] = d.xpos[self.base_id]
            self.viewer.sync()
            delay = d.time - (time.perf_counter() - self._wall_start)
            if delay > 0:
                time.sleep(delay)
        return self.get_state()

    def get_state(self):
        d = self.data
        rotation = d.xmat[self.base_id].reshape(3, 3)
        velocity = self._body_velocity()
        contacts = set()
        for contact in d.contact[:d.ncon]:
            for geom in (contact.geom1, contact.geom2):
                if int(geom) in self.feet:
                    contacts.add(self.feet[int(geom)])
        return RobotState(float(d.time), *map(float, d.xpos[self.base_id]),
                          math.atan2(rotation[1, 0], rotation[0, 0]),
                          float(velocity[3]), float(velocity[4]), float(velocity[2]),
                          float(rotation[2, 2]), tuple(sorted(contacts)),
                          tuple(map(float, self._effective_command())))

    def wait(self, seconds):
        """推进仿真但不刷新运动命令；旧命令仍按 0.3 s 失效。"""
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("等待时间必须为有限非负数")
        end = self.data.time + seconds
        while self.data.time < end - 1e-9 and self.is_running():
            self.step()

    def run_velocity(self, vx, vy, vyaw, seconds):
        """持续更新一段速度指令；结束时发出停止请求。"""
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("运行时间必须为有限正数")
        end = self.data.time + seconds
        try:
            while self.data.time < end - 1e-9 and self.is_running():
                self.move(vx, vy, vyaw)
                self.step()
        finally:
            self.stop()

    @property
    def warnings(self):
        return {mujoco.mjtWarning(i).name: int(w.number)
                for i, w in enumerate(self.data.warning) if w.number}

    def is_running(self):
        return not self._closed and (self.viewer is None or self.viewer.is_running())

    def save_screenshot(self, path, *, camera="follow"):
        from PIL import Image

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        view = camera
        if camera == "follow":
            view = mujoco.MjvCamera()
            view.lookat[:] = self.data.xpos[self.base_id]
            view.distance, view.azimuth, view.elevation = 2.2, 135, -20
        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            renderer.update_scene(self.data, camera=view)
            Image.fromarray(renderer.render()).save(path)

    def close(self):
        self.stop()
        if self.viewer is not None:
            self.viewer.close()
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
