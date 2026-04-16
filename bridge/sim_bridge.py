"""
bridge/sim_bridge.py — MuJoCo simulation bridge (Phase 2, no ROS2)

Kinematic locomotion:
  • Joint positions held at HOME_CTRL (static standing pose).
  • Trunk free-joint integrated each control tick from agent velocity commands.
  • Trunk z fixed at TRUNK_HEIGHT — no fall physics in base (no locomotion policy).
  • Shoot command applies a brief xfrc_applied impulse on the ball.

Usage:
    uv run python -m bridge.sim_bridge
    uv run python -m bridge.sim_bridge --viewer
    uv run python -m bridge.sim_bridge --duration 60 --speed 2.0
    uv run sim-bridge --viewer
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

from bridge.agent_interface import AgentInterface, ActionCmd, SensorState, StandaloneAgent

# ── paths ──────────────────────────────────────────────────────────────────────
SCENE_PATH = Path(__file__).parent.parent / "scenes" / "soccer_scene.xml"

# ── timing ─────────────────────────────────────────────────────────────────────
CONTROL_HZ      = 50
SIM_HZ          = 200
STEPS_PER_CTRL  = SIM_HZ // CONTROL_HZ   # = 4

# ── scene body indices (verified against soccer_scene.xml) ─────────────────────
#   0=world  1=goal_right  2=goal_left  3=ball  4=Trunk  5..28=T1 links
BALL_BODY_ID  = 3
TRUNK_BODY_ID = 4

# ── Trunk free-joint in qpos/qvel (world_joint, qposadr=7, dofadr=6) ──────────
TRUNK_QPOSADR = 7    # qpos[7:14] = [x, y, z, qw, qx, qy, qz]
TRUNK_DOFADR  = 6    # qvel[6:12] = [vx, vy, vz, wx, wy, wz]

# ── kinematic height ───────────────────────────────────────────────────────────
TRUNK_HEIGHT = 0.75  # m  — fixed z in kinematic mode

# ── default joint targets (23 actuators) ──────────────────────────────────────
#   [0]  AAHead_yaw          [1]  Head_pitch
#   [2]  LShoulderPitch      [3]  LShoulderRoll   [4]  LElbowPitch  [5]  LElbowYaw
#   [6]  RShoulderPitch      [7]  RShoulderRoll   [8]  RElbowPitch  [9]  RElbowYaw
#   [10] Waist
#   [11] LHipPitch [12] LHipRoll [13] LHipYaw [14] LKneePitch [15] LAnklePitch [16] LAnkleRoll
#   [17] RHipPitch [18] RHipRoll [19] RHipYaw [20] RKneePitch [21] RAnklePitch [22] RAnkleRoll
HOME_CTRL = np.array([
    0.0,  0.0,                          # head
    0.2, -1.3,  0.0, -0.5,             # L arm
    0.2,  1.3,  0.0,  0.5,             # R arm
    0.0,                                # waist
   -0.2,  0.0,  0.0,  0.4, -0.2, 0.0, # L leg
   -0.2,  0.0,  0.0,  0.4, -0.2, 0.0, # R leg
], dtype=np.float64)

# ── terminal helpers ───────────────────────────────────────────────────────────
BOLD = "\033[1m";  CYAN = "\033[96m";  GREEN = "\033[92m"
RED  = "\033[91m"; DIM  = "\033[2m";   RESET = "\033[0m"

def _ok(msg):   print(f"  {GREEN}✓{RESET} {msg}")
def _info(msg): print(f"  {DIM}·{RESET} {msg}")
def _err(msg):  print(f"  {RED}✗{RESET} {msg}")


# ── per-robot kinematic state ──────────────────────────────────────────────────

@dataclass
class _KinState:
    """Integrated pose for one robot's base (trunk free joint)."""
    x:   float = 0.0
    y:   float = 0.0
    yaw: float = 0.0


# ── SimBridge ──────────────────────────────────────────────────────────────────

class SimBridge:
    """
    Orchestrates MuJoCo physics + N agent instances.

    Current limitation: scene has 1 robot (1 free joint for Trunk).
    Multi-robot support requires duplicating the robot body in the scene
    and extending the joint-index map — see ROADMAP_ROS2.md.
    """

    SHOOT_FORCE = 80.0   # N — applied to ball during shoot
    SHOOT_TICKS = 5      # ctrl ticks the force is applied

    def __init__(self, agents: list[AgentInterface]) -> None:
        if not SCENE_PATH.exists():
            raise FileNotFoundError(f"Scene not found: {SCENE_PATH}")

        self.m = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.d = mujoco.MjData(self.m)

        if len(agents) > 1:
            raise NotImplementedError(
                "Multi-robot: duplicate robot body in scene first. "
                "See ROADMAP_ROS2.md §Multi-robot."
            )
        self.agents = agents

        self._kin:              list[_KinState] = [_KinState() for _ in agents]
        self._shoot_remaining:  list[int]        = [0] * len(agents)
        self._tick = 0

        _ok(f"Modelo carregado  nbody={self.m.nbody}  nu={self.m.nu}  "
            f"nq={self.m.nq}  nv={self.m.nv}")
        _ok(f"Agentes: {[a.robot_name for a in agents]}")

    # ── episode reset ──────────────────────────────────────────────────────────

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        mujoco.mj_forward(self.m, self.d)

        # seed kinematic state from keyframe trunk position
        q0 = self.d.qpos[TRUNK_QPOSADR: TRUNK_QPOSADR + 7]
        for ks in self._kin:
            ks.x   = float(q0[0])
            ks.y   = float(q0[1])
            ks.yaw = _quat_to_yaw(q0[3:7])

        self._shoot_remaining = [0] * len(self.agents)
        self._tick = 0

        for agent in self.agents:
            agent.reset()

    # ── sensor / action ────────────────────────────────────────────────────────

    def _sensor(self, i: int) -> SensorState:
        return SensorState(
            qpos       = self.d.qpos.copy(),
            qvel       = self.d.qvel.copy(),
            trunk_pos  = self.d.xpos[TRUNK_BODY_ID].copy(),
            trunk_quat = self.d.xquat[TRUNK_BODY_ID].copy(),
            ball_pos   = self.d.xpos[BALL_BODY_ID].copy(),
            tick       = self._tick,
        )

    def _apply(self, i: int, cmd: ActionCmd) -> None:
        ks  = self._kin[i]
        dt  = 1.0 / CONTROL_HZ

        # integrate base pose (body frame → world frame)
        c, s    = math.cos(ks.yaw), math.sin(ks.yaw)
        ks.x   += (cmd.vx * c - cmd.vy * s) * dt
        ks.y   += (cmd.vx * s + cmd.vy * c) * dt
        ks.yaw += cmd.vyaw * dt
        ks.yaw  = (ks.yaw + math.pi) % (2.0 * math.pi) - math.pi  # wrap

        # write trunk free-joint — xyz + quaternion
        h = ks.yaw * 0.5
        self.d.qpos[TRUNK_QPOSADR + 0] = ks.x
        self.d.qpos[TRUNK_QPOSADR + 1] = ks.y
        self.d.qpos[TRUNK_QPOSADR + 2] = TRUNK_HEIGHT
        self.d.qpos[TRUNK_QPOSADR + 3] = math.cos(h)   # w
        self.d.qpos[TRUNK_QPOSADR + 4] = 0.0            # x
        self.d.qpos[TRUNK_QPOSADR + 5] = 0.0            # y
        self.d.qpos[TRUNK_QPOSADR + 6] = math.sin(h)   # z

        # zero trunk velocity (kinematic — prevent drift accumulation)
        self.d.qvel[TRUNK_DOFADR: TRUNK_DOFADR + 6] = 0.0

        # joint ctrl
        ctrl = HOME_CTRL.copy()
        ctrl[0] = cmd.head_yaw
        ctrl[1] = cmd.head_pitch
        self.d.ctrl[:] = ctrl

        # shoot impulse
        if cmd.shoot and self._shoot_remaining[i] == 0:
            self._shoot_remaining[i] = self.SHOOT_TICKS

        self.d.xfrc_applied[BALL_BODY_ID] = 0.0
        if self._shoot_remaining[i] > 0:
            fx = self.SHOOT_FORCE * math.cos(ks.yaw)
            fy = self.SHOOT_FORCE * math.sin(ks.yaw)
            self.d.xfrc_applied[BALL_BODY_ID, 0] = fx
            self.d.xfrc_applied[BALL_BODY_ID, 1] = fy
            self._shoot_remaining[i] -= 1

    # ── control tick ───────────────────────────────────────────────────────────

    def _ctrl_tick(self) -> None:
        """One 50 Hz tick: sense → think → actuate → step physics × 4."""
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
        """
        Run the bridge loop.

        Args:
            duration: simulated seconds (0 = run indefinitely until Ctrl-C)
            viewer:   open MuJoCo interactive viewer window
            speed:    wall-clock speedup (1.0 = real-time, 0 = as fast as possible)
        """
        self.reset()
        steps_total = int(duration / self.m.opt.timestep) if duration > 0 else None

        if viewer:
            self._loop_viewer(steps_total, speed)
        else:
            self._loop_headless(steps_total, speed)

    # ── headless loop ──────────────────────────────────────────────────────────

    def _loop_headless(self, steps_total: Optional[int], speed: float) -> None:
        print(f"\n{BOLD}SimBridge headless{RESET}  "
              f"(Ctrl-C para parar)\n")

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

                # status line every 5 s of simulated time
                if step % (CONTROL_HZ * STEPS_PER_CTRL * 5) == 0:
                    t_sim   = step * self.m.opt.timestep
                    trunk_z = self.d.xpos[TRUNK_BODY_ID][2]
                    ball    = self.d.xpos[BALL_BODY_ID]
                    phase   = self.agents[0]._phase.name
                    _info(f"t={t_sim:6.1f}s  trunk_z={trunk_z:.3f}  "
                          f"ball=({ball[0]:.2f},{ball[1]:.2f})  "
                          f"phase={phase}")

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

        print(f"\n{BOLD}SimBridge viewer{RESET}  "
              f"(feche a janela para encerrar)\n")

        step    = 0
        t_wall0 = time.perf_counter()

        with mjv.launch_passive(self.m, self.d) as v:
            v.cam.distance  = 14.0
            v.cam.elevation = -25.0

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
    """MuJoCo quaternion (w, x, y, z) → yaw angle (rad)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ── entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="SimBridge — MuJoCo + StandaloneAgent (Fase 2)"
    )
    parser.add_argument("--viewer",   action="store_true",
                        help="Abre o viewer interativo MuJoCo")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Segundos de simulação (0 = infinito)  [default: 30]")
    parser.add_argument("--speed",    type=float, default=1.0,
                        help="Fator de velocidade real (0 = máximo)  [default: 1.0]")
    args = parser.parse_args()

    print(f"\n{BOLD}T1 Soccer Sim — Fase 2: SimBridge{RESET}")

    agent  = StandaloneAgent(robot_name="T1_1")
    bridge = SimBridge(agents=[agent])
    bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)


if __name__ == "__main__":
    main()
