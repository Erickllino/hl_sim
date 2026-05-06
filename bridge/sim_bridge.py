"""
bridge/sim_bridge.py — MuJoCo simulation bridge (3v3, 6 robots)

Kinematic locomotion:
  • Joint positions animated by a sinusoidal CPG gait based on velocity commands.
  • Trunk free-joint integrated each control tick from agent velocity commands.
  • Trunk z fixed at TRUNK_HEIGHT — no fall physics in base (no locomotion policy).
  • Shoot command applies a brief xfrc_applied impulse on the ball.

Robot order in scene (matches <include> order in soccer_scene.xml):
  idx 0 → robot1  (team 1, gray)    pos (-1,  0.0)
  idx 1 → robot3  (team 1, gray)    pos (-2,  1.5)
  idx 2 → robot5  (team 1, gray)    pos (-2, -1.5)
  idx 3 → robot2  (team 2, red)     pos ( 1,  0.0)
  idx 4 → robot4  (team 2, red)     pos ( 2,  1.5)
  idx 5 → robot6  (team 2, red)     pos ( 2, -1.5)

Usage:
    uv run python -m bridge.sim_bridge
    uv run python -m bridge.sim_bridge --viewer
    uv run python -m bridge.sim_bridge --duration 60 --speed 2.0
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


import numpy as np

try:
    import mujoco
except ImportError:
    print("ERRO: mujoco não instalado.  Execute: uv sync")
    sys.exit(1)

from bridge.agent_interface import AgentInterface, ActionCmd, SensorState
from bridge.default_agent import DefaultAgent

# ── paths ──────────────────────────────────────────────────────────────────────
SCENE_PATH = Path(__file__).parent.parent / "scenes" / "soccer_scene.xml"

# ── timing ─────────────────────────────────────────────────────────────────────
CONTROL_HZ      = 50
SIM_HZ          = 200
STEPS_PER_CTRL  = SIM_HZ // CONTROL_HZ   # = 4

# ── scene constants ────────────────────────────────────────────────────────────
BALL_BODY_ID            = 3
NUM_ACTUATORS_PER_ROBOT = 23
SENSOR_BLOCK            = 7   # framequat(4) + gyro(3) per robot
TRUNK_HEIGHT            = 0.679   # matches keyframe home pose

# ── per-robot layout (6 robots in scene order: 1,3,5,2,4,6) ───────────────────
#
#  Body IDs: world(0) goal_right(1) goal_left(2) ball(3)
#            then 24 bodies per robot (Trunk + 23 links)
#
#  qpos: ball(7) + per robot: trunk_free(7) + joints(23) = 30
#  qvel: ball(6) + per robot: trunk_free(6) + joints(23) = 29
#
_BODIES_BEFORE_ROBOTS = 4   # world, goal_right, goal_left, ball
_BODIES_PER_ROBOT     = 24
_QPOS_BALL            = 7
_QPOS_PER_ROBOT       = 30  # 7 free + 23 joints
_QVEL_BALL            = 6
_QVEL_PER_ROBOT       = 29  # 6 free + 23 joints

NUM_ROBOTS = 6

TRUNK_BODY_IDS  = [_BODIES_BEFORE_ROBOTS + i * _BODIES_PER_ROBOT for i in range(NUM_ROBOTS)]
TRUNK_QPOS_ADRS = [_QPOS_BALL            + i * _QPOS_PER_ROBOT   for i in range(NUM_ROBOTS)]
TRUNK_DOF_ADRS  = [_QVEL_BALL            + i * _QVEL_PER_ROBOT   for i in range(NUM_ROBOTS)]
SENSOR_STARTS   = [i * SENSOR_BLOCK                               for i in range(NUM_ROBOTS)]

# ── default joint targets (23 actuators) ──────────────────────────────────────
HOME_CTRL = np.array([
    0.0,  0.0,                          # head
    0.2, -1.3,  0.0, -0.5,             # L arm
    0.2,  1.3,  0.0,  0.5,             # R arm
    0.0,                                # waist
   -0.2,  0.0,  0.0,  0.4, -0.2, 0.0, # L leg
   -0.2,  0.0,  0.0,  0.4, -0.2, 0.0, # R leg
], dtype=np.float64)

# ── CPG gait parameters ────────────────────────────────────────────────────────
_GAIT_FREQ   = 1.8   # Hz — steps per second per leg

# Actuator indices in ctrl array (per robot, offset = i * NUM_ACTUATORS_PER_ROBOT):
# 0  AAHead_yaw       1  Head_pitch
# 2  L_Shoulder_Pitch 3  L_Shoulder_Roll  4  L_Elbow_Pitch  5  L_Elbow_Yaw
# 6  R_Shoulder_Pitch 7  R_Shoulder_Roll  8  R_Elbow_Pitch  9  R_Elbow_Yaw
# 10 Waist
# 11 L_Hip_Pitch     12 L_Hip_Roll       13 L_Hip_Yaw      14 L_Knee_Pitch
# 15 L_Ankle_Pitch   16 L_Ankle_Roll
# 17 R_Hip_Pitch     18 R_Hip_Roll       19 R_Hip_Yaw      20 R_Knee_Pitch
# 21 R_Ankle_Pitch   22 R_Ankle_Roll


def _gait_ctrl(cmd: ActionCmd, tick: int) -> np.ndarray:
    """Sinusoidal CPG gait — returns 23-element ctrl array."""
    ctrl = HOME_CTRL.copy()

    speed  = math.hypot(cmd.vx, cmd.vy)
    motion = min(1.0, (speed + abs(cmd.vyaw) * 0.25) * 3.5)
    if motion < 0.05:
        return ctrl  # standing still

    phase = 2.0 * math.pi * _GAIT_FREQ * tick / CONTROL_HZ

    STRIDE = 0.28 * motion   # hip pitch swing (rad)
    LIFT   = 0.22 * motion   # extra knee flex during swing (rad)
    ANKLE  = 0.10 * motion   # ankle push-off (rad)
    ARM    = 0.14 * motion   # shoulder swing (rad)

    # (phase_offset, hip_pitch_i, hip_roll_i, knee_i, ankle_i, arm_pitch_i)
    legs = (
        (0.0,      11, 12, 14, 15, 2),   # left  leg / left  arm
        (math.pi,  17, 18, 20, 21, 6),   # right leg / right arm
    )
    for ph_off, hip_i, roll_i, knee_i, ankle_i, arm_i in legs:
        ph      = phase + ph_off
        sin_ph  = math.sin(ph)

        ctrl[hip_i]   = HOME_CTRL[hip_i]   + STRIDE * sin_ph
        ctrl[knee_i]  = HOME_CTRL[knee_i]  + LIFT   * max(0.0, -sin_ph)  # flex on swing
        ctrl[ankle_i] = HOME_CTRL[ankle_i] + ANKLE  * sin_ph
        ctrl[arm_i]   = HOME_CTRL[arm_i]   - ARM    * sin_ph   # arms swing opposite

    return ctrl


# ── terminal helpers ───────────────────────────────────────────────────────────
BOLD = "\033[1m";  CYAN = "\033[96m";  GREEN = "\033[92m"
RED  = "\033[91m"; DIM  = "\033[2m";   RESET = "\033[0m"

def _ok(msg):   print(f"  {GREEN}✓{RESET} {msg}")
def _info(msg): print(f"  {DIM}·{RESET} {msg}")
def _err(msg):  print(f"  {RED}✗{RESET} {msg}")


# ── per-robot kinematic state ──────────────────────────────────────────────────

@dataclass
class _KinState:
    x:   float = 0.0
    y:   float = 0.0
    yaw: float = 0.0


# ── SimBridge ──────────────────────────────────────────────────────────────────

class SimBridge:
    SHOOT_FORCE = 10.0
    SHOOT_TICKS = 5

    def __init__(self, agents: list[AgentInterface]) -> None:
        if not SCENE_PATH.exists():
            raise FileNotFoundError(f"Scene not found: {SCENE_PATH}")
        if len(agents) > NUM_ROBOTS:
            raise ValueError(f"Scene supports at most {NUM_ROBOTS} robots.")

        self.m = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.d = mujoco.MjData(self.m)
        self.agents = agents

        self._kin:             list[_KinState] = [_KinState() for _ in agents]
        self._shoot_remaining: list[int]        = [0] * len(agents)
        self._tick = 0

        _ok(f"Modelo carregado  nbody={self.m.nbody}  nu={self.m.nu}  "
            f"nq={self.m.nq}  nv={self.m.nv}")
        _ok(f"Agentes ({len(agents)}): {[a.robot_name for a in agents]}")

    # ── episode reset ──────────────────────────────────────────────────────────

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        mujoco.mj_forward(self.m, self.d)

        for i, ks in enumerate(self._kin):
            adr = TRUNK_QPOS_ADRS[i]
            q   = self.d.qpos[adr: adr + 7]
            ks.x   = float(q[0])
            ks.y   = float(q[1])
            ks.yaw = _quat_to_yaw(q[3:7])

        self._shoot_remaining = [0] * len(self.agents)
        self._tick = 0

        for agent in self.agents:
            agent.reset()

    # ── sensor / action ────────────────────────────────────────────────────────

    def _sensor(self, i: int) -> SensorState:
        return SensorState(
            qpos       = self.d.qpos.copy(),
            qvel       = self.d.qvel.copy(),
            trunk_pos  = self.d.xpos[TRUNK_BODY_IDS[i]].copy(),
            trunk_quat = self.d.xquat[TRUNK_BODY_IDS[i]].copy(),
            ball_pos   = self.d.xpos[BALL_BODY_ID].copy(),
            tick       = self._tick,
            sensordata = self.d.sensordata[
                SENSOR_STARTS[i]: SENSOR_STARTS[i] + SENSOR_BLOCK
            ].copy(),
        )

    def _apply(self, i: int, cmd: ActionCmd) -> None:
        ks         = self._kin[i]
        dt         = 1.0 / CONTROL_HZ
        ctrl_start = i * NUM_ACTUATORS_PER_ROBOT
        qposadr    = TRUNK_QPOS_ADRS[i]
        dofadr     = TRUNK_DOF_ADRS[i]
        trunk_id   = TRUNK_BODY_IDS[i]

        if cmd.joint_pos is not None:
            # ROS2/deploy mode: use explicit joint targets from /joint_ctrl
            ctrl = HOME_CTRL.copy()
            ctrl[0] = cmd.head_yaw
            ctrl[1] = cmd.head_pitch
            n = min(len(cmd.joint_pos), NUM_ACTUATORS_PER_ROBOT - 2)
            ctrl[2: 2 + n] = cmd.joint_pos[:n]
            self.d.ctrl[ctrl_start: ctrl_start + NUM_ACTUATORS_PER_ROBOT] = ctrl
        else:
            # Kinematic mode: integrate trunk pose, animate joints via CPG gait
            c, s    = math.cos(ks.yaw), math.sin(ks.yaw)
            ks.x   += (cmd.vx * c - cmd.vy * s) * dt
            ks.y   += (cmd.vx * s + cmd.vy * c) * dt
            ks.yaw += cmd.vyaw * dt
            ks.yaw  = (ks.yaw + math.pi) % (2.0 * math.pi) - math.pi

            h = ks.yaw * 0.5
            self.d.qpos[qposadr + 0] = ks.x
            self.d.qpos[qposadr + 1] = ks.y
            self.d.qpos[qposadr + 2] = TRUNK_HEIGHT
            self.d.qpos[qposadr + 3] = math.cos(h)
            self.d.qpos[qposadr + 4] = 0.0
            self.d.qpos[qposadr + 5] = 0.0
            self.d.qpos[qposadr + 6] = math.sin(h)
            self.d.qvel[dofadr: dofadr + 6] = 0.0

            ctrl = _gait_ctrl(cmd, self._tick)
            ctrl[0] = cmd.head_yaw
            ctrl[1] = cmd.head_pitch
            self.d.ctrl[ctrl_start: ctrl_start + NUM_ACTUATORS_PER_ROBOT] = ctrl

        if cmd.shoot and self._shoot_remaining[i] == 0:
            self._shoot_remaining[i] = self.SHOOT_TICKS

        if self._shoot_remaining[i] > 0:
            trunk_yaw = _quat_to_yaw(self.d.xquat[trunk_id])
            fx = self.SHOOT_FORCE * math.cos(trunk_yaw)
            fy = self.SHOOT_FORCE * math.sin(trunk_yaw)
            self.d.xfrc_applied[BALL_BODY_ID, 0] += fx
            self.d.xfrc_applied[BALL_BODY_ID, 1] += fy
            self._shoot_remaining[i] -= 1

    # ── control tick ───────────────────────────────────────────────────────────

    def _ctrl_tick(self) -> None:
        self.d.xfrc_applied[BALL_BODY_ID] = 0.0
        for i, agent in enumerate(self.agents):
            cmd = agent.step(self._sensor(i))
            self._apply(i, cmd)
        for _ in range(STEPS_PER_CTRL):
            mujoco.mj_step(self.m, self.d)
        self._tick += 1

    # ── run ────────────────────────────────────────────────────────────────────

    def run(
        self,
        duration: float = 30.0,
        viewer:   bool  = False,
        speed:    float = 1.0,
    ) -> None:
        self.reset()
        steps_total = int(duration / self.m.opt.timestep) if duration > 0 else None

        if viewer:
            self._loop_viewer(steps_total, speed)
        else:
            self._loop_headless(steps_total, speed)

    # ── headless loop ──────────────────────────────────────────────────────────

    def _loop_headless(self, steps_total: Optional[int], speed: float) -> None:
        print(f"\n{BOLD}SimBridge headless{RESET}  (Ctrl-C para parar)\n")

        step    = 0
        t_wall0 = time.perf_counter()

        try:
            while steps_total is None or step < steps_total:
                self._ctrl_tick()
                step += STEPS_PER_CTRL

                if speed > 0:
                    t_sim  = step * self.m.opt.timestep
                    t_wall = time.perf_counter() - t_wall0
                    lag    = t_sim / speed - t_wall
                    if lag > 0:
                        time.sleep(lag)

                if step % (CONTROL_HZ * STEPS_PER_CTRL * 5) == 0:
                    t_sim   = step * self.m.opt.timestep
                    trunk_z = self.d.xpos[TRUNK_BODY_IDS[0]][2]
                    ball    = self.d.xpos[BALL_BODY_ID]
                    phase   = getattr(self.agents[0], '_phase', None)
                    phase_str = phase.name if phase is not None else 'ros2'
                    _info(f"t={t_sim:6.1f}s  trunk_z={trunk_z:.3f}  "
                          f"ball=({ball[0]:.2f},{ball[1]:.2f})  "
                          f"phase={phase_str}")

        except KeyboardInterrupt:
            print(f"\n{DIM}interrompido pelo usuário{RESET}")

    # ── viewer loop ────────────────────────────────────────────────────────────

    def _loop_viewer(self, steps_total: Optional[int], speed: float) -> None:
        try:
            import mujoco.viewer as mjv
        except ImportError:
            _err("mujoco.viewer não disponível — usando headless")
            self._loop_headless(steps_total, speed)
            return

        print(f"\n{BOLD}SimBridge viewer{RESET}  (feche a janela para encerrar)\n")

        step    = 0
        t_wall0 = time.perf_counter()

        with mjv.launch_passive(self.m, self.d) as v:
            v.cam.distance  = 18.0
            v.cam.elevation = -30.0

            while v.is_running():
                if steps_total is not None and step >= steps_total:
                    break

                self._ctrl_tick()
                step += STEPS_PER_CTRL
                v.sync()

                if speed > 0:
                    t_sim  = step * self.m.opt.timestep
                    t_wall = time.perf_counter() - t_wall0
                    lag    = t_sim / speed - t_wall
                    if lag > 0:
                        time.sleep(lag)


# ── helpers ────────────────────────────────────────────────────────────────────

def _quat_to_yaw(q: np.ndarray) -> float:
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ── entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="SimBridge — MuJoCo 3v3 (6 robots, standalone)"
    )
    parser.add_argument("--viewer",   action="store_true")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--speed",    type=float, default=1.0)
    args = parser.parse_args()

    print(f"\n{BOLD}T1 Soccer Sim — 3v3{RESET}")

    agents = [
        DefaultAgent(robot_name="T1_0"),
        DefaultAgent(robot_name="T1_1"),
        DefaultAgent(robot_name="T1_2"),
        DefaultAgent(robot_name="T1_3"),
        DefaultAgent(robot_name="T1_4"),
        DefaultAgent(robot_name="T1_5"),
    ]

    bridge = SimBridge(agents=agents)
    bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)


if __name__ == "__main__":
    main()
