import os
import time
import copy
from typing import Tuple, Optional

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import robosuite as suite
from dataclasses import dataclass

@dataclass
class ForceTorqueLimits:
    # Soft and hard wrench limits
    f_soft: float
    f_hard: float
    t_soft: float
    t_hard: float

@dataclass
class CostConfig:
    # Weights for different cost terms
    pos: float
    ori: float
    hinge: float
    force: float
    torque: float
    hard: float

@dataclass
class ControlConfig:
    # Parameters controlling servo, grasp logic.
    servo_gain: float
    ori_gain: float
    servo_dist: float
    approach_dist: float
    pre_grip_level: float
    full_grip_level: float
    contact_timeout: float
    mpc_horizon: int
    mpc_samples: int
    noise_scale: float

os.environ.setdefault("MUJOCO_GL", "glfw")
ENV = suite.make(
    "Door",
    robots="Sawyer",
    use_latch=False,
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=True,
    horizon=10_000,
)
SIM   = ENV.sim
MODEL = SIM.model

# Locate force-torque sensor indices
sensor_names = MODEL.sensor_names
FT_FORCE_IDX  = next(i for i,n in enumerate(sensor_names) if "force" in n.lower()  and "ee" in n.lower())
FT_TORQUE_IDX = next(i for i,n in enumerate(sensor_names) if "torque" in n.lower() and "ee" in n.lower())

# Action dimensions
ACT_LOW, ACT_HIGH = ENV.action_spec
ACTION_DIM = ACT_LOW.shape[0]
GRIP_IDX   = ACTION_DIM - 1

class ContactDetector:
    # Monitors contact between fingers and handle
    def __init__(self, model):
        geom_names = model.geom_names
        self.finger_ids = [i for i,n in enumerate(geom_names)
                           if "finger" in n.lower() and ("tip" in n.lower() or "pad" in n.lower())]
        self.handle_ids = [i for i,n in enumerate(geom_names)
                           if "handle" in n.lower() or "latch" in n.lower()]
        if not self.finger_ids or not self.handle_ids:
            raise RuntimeError("Missing required geometries in model.")

    def touching(self, sim) -> bool:
        for ci in range(sim.data.ncon):
            c = sim.data.contact[ci]
            if ((c.geom1 in self.finger_ids and c.geom2 in self.handle_ids)
             or (c.geom2 in self.finger_ids and c.geom1 in self.handle_ids)):
                return True
        return False


def read_wrench(sim) -> Tuple[np.ndarray, np.ndarray]:
    # Return (force, torque) arrays from sensors
    d = sim.data.sensordata
    return d[FT_FORCE_IDX:FT_FORCE_IDX+3].copy(), d[FT_TORQUE_IDX:FT_TORQUE_IDX+3].copy()


def orientation_mismatch(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    # Small-angle error between gripper z-axis and target vector
    z = Rot.from_quat(quat).as_matrix()[:,2]
    u = vec / (np.linalg.norm(vec) + 1e-8)
    return 0.5 * np.cross(z, u)

class DoorController:
    def __init__(
        self,
        env,
        ft_limits: ForceTorqueLimits,
        cost_cfg: CostConfig,
        ctrl_cfg: ControlConfig,
        seed: Optional[int] = None
    ):
        self.env                 = env
        self.sim                 = env.sim
        self.ft                  = ft_limits
        self.cost_w              = cost_cfg
        self.ctrl_p              = ctrl_cfg
        self.rand                = np.random.default_rng(seed)
        self.contact             = ContactDetector(self.sim.model)
        self.in_grasp            = False
        self.last_contact_time   = -np.inf
        self.grip_value          = -1.0
        self.prev_plan           = None

    def _wrench_cost(self, f: np.ndarray, t: np.ndarray) -> float:
        fm, tm = np.linalg.norm(f), np.linalg.norm(t)
        soft = (self.cost_w.force  * max(0, fm - self.ft.f_soft)**2
              + self.cost_w.torque * max(0, tm - self.ft.t_soft)**2)
        hard = self.cost_w.hard * (fm + tm) if (fm > self.ft.f_hard or tm > self.ft.t_hard) else 0.0
        return soft + hard

    def _stage_cost(self, obs: dict, f: np.ndarray, t: np.ndarray) -> float:
        p = obs['robot0_eef_pos']
        q = obs['robot0_eef_quat']
        h = obs['handle_pos']
        hinge = float(obs['hinge_qpos'])

        dist = np.linalg.norm(h - p)
        ori_err = np.linalg.norm(orientation_mismatch(q, h - p))
        grip_mult = 0.15 if self.in_grasp else 1.0

        cost_pose   = grip_mult * (self.cost_w.pos * dist**2 + self.cost_w.ori * ori_err**2)
        cost_hinge  = self.cost_w.hinge * hinge
        cost_wrench = self._wrench_cost(f, t)
        return cost_pose + cost_hinge + cost_wrench

    def _simulate_sequence(self, state0, seq, f0, t0) -> float:
        self.sim.set_state(state0)
        self.sim.forward()
        obs = self.env._get_observations()
        total = self._stage_cost(obs, f0, t0)

        for action in seq:
            self.sim.data.ctrl[:ACTION_DIM] = action
            for _ in range(int(self.env.control_timestep)):
                self.sim.step()
            obs = self.env._get_observations()
            f, t = read_wrench(self.sim)
            total += self._stage_cost(obs, f, t)
            if np.linalg.norm(f) > self.ft.f_hard or np.linalg.norm(t) > self.ft.t_hard:
                total += self.cost_w.hard * 1e2
                break
        return total

    def act(self, obs: dict) -> np.ndarray:
        now = time.time()
        # update contact/grasp state
        if self.contact.touching(self.sim):
            self.last_contact_time = now
            if not self.in_grasp:
                self.in_grasp = True
                self.grip_value = self.ctrl_p.full_grip_level
        elif self.in_grasp and (now - self.last_contact_time) > self.ctrl_p.contact_timeout:
            self.in_grasp = False
            self.grip_value = self.ctrl_p.pre_grip_level

        # compute delta
        pos    = obs['robot0_eef_pos']
        handle = obs['handle_pos']
        delta  = handle - pos
        d      = np.linalg.norm(delta)

        # simple servo
        if d > self.ctrl_p.servo_dist and not self.in_grasp:
            cmd = np.zeros(ACTION_DIM)
            cmd[:3]  = np.clip(self.ctrl_p.servo_gain * delta, ACT_LOW[:3], ACT_HIGH[:3])
            cmd[3:6] = np.clip(
                self.ctrl_p.ori_gain * orientation_mismatch(obs['robot0_eef_quat'], delta),
                ACT_LOW[3:6], ACT_HIGH[3:6]
            )
            cmd[GRIP_IDX] = -1.0 if d > self.ctrl_p.approach_dist else self.ctrl_p.pre_grip_level
            self.grip_value = cmd[GRIP_IDX]
            return cmd

        f0, t0 = read_wrench(self.sim)
        base = (np.zeros((self.ctrl_p.mpc_horizon, ACTION_DIM))
                if self.prev_plan is None
                else np.vstack([self.prev_plan[1:], np.zeros((1, ACTION_DIM))]))

        # drift
        if not self.in_grasp:
            step = (delta / max(d, 1e-6)) * min(0.03, d)
            base[:, :3] += step
            self.grip_value = self.ctrl_p.pre_grip_level
        else:
            base[:, :3] += np.array([0, 0.08, 0]) / self.ctrl_p.mpc_horizon

        base[:, GRIP_IDX] = self.grip_value

        # sample
        seqs = self.rand.normal(
            base,
            self.ctrl_p.noise_scale,
            (self.ctrl_p.mpc_samples, self.ctrl_p.mpc_horizon, ACTION_DIM)
        )
        seqs = np.clip(seqs, ACT_LOW, ACT_HIGH)

        state_snapshot = copy.deepcopy(self.sim.get_state())
        costs = np.array([self._simulate_sequence(state_snapshot, s, f0, t0) for s in seqs])
        best = int(np.argmin(costs))
        self.prev_plan = seqs[best]
        return self.prev_plan[0]

def main(
    ft_limits: ForceTorqueLimits,
    cost_cfg:   CostConfig,
    ctrl_cfg:   ControlConfig,
    hinge_cutoff: float =50,
    max_steps:    Optional[int] = None
):
    controller = DoorController(ENV, ft_limits, cost_cfg, ctrl_cfg)
    obs = ENV.reset()
    for i in range(max_steps or ENV.horizon):
        action = controller.act(obs)
        obs, _, _, _ = ENV.step(action)
        if i % 150 == 0:
            f, t = read_wrench(ENV.sim)
            print(
                f"Step {i:5d} | Hinge {obs['hinge_qpos']:+.3f}"
                f" | |F|{np.linalg.norm(f):.1f}N | |T|{np.linalg.norm(t):.1f}Nm"
                + (" | GRASP" if controller.in_grasp else "")
            )
        
        if obs['hinge_qpos'] > hinge_cutoff:
            break
    ENV.close()

if __name__ == "__main__":
    # Example instantiation with default parameters
    ft = ForceTorqueLimits(f_soft=30., f_hard=60., t_soft=6., t_hard=12.)
    cost = CostConfig(pos=12., ori=1.5, hinge=-60., force=2., torque=0.8, hard=1e3)
    ctrl = ControlConfig(
        servo_gain=4., ori_gain=1.5, servo_dist=0.04,
        approach_dist=0.18, pre_grip_level=0.30, full_grip_level=0.60,
        contact_timeout=0.4, mpc_horizon=15, mpc_samples=256, noise_scale=0.4
    )
    main(ft, cost, ctrl)
