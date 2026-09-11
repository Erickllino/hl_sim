"""
hl_sim/sim/bridge.py — MuJoCo simulation bridge (3v3, 6 robots)

Kinematic locomotion:
  • Joint positions held at HOME_CTRL (static standing pose).
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

Usage (standalone, sem ROS2 e sem GameController):
    python -m hl_sim.sim.bridge --force-playing
    python -m hl_sim.sim.bridge --viewer --force-playing

Para a cadeia de competição completa (brain + GameController oficial),
use hl_sim/cli/run_ros2.py.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Optional


import numpy as np

try:
    import mujoco
except ImportError:
    print("ERRO: mujoco não instalado.  Execute: uv sync")
    sys.exit(1)

from hl_sim import paths
from hl_sim.agents.base import (
    AgentInterface,
    ActionCmd,
    SensorState,
    GameState,
    STATE_PLAYING,
)
from hl_sim.agents.scripted import ScriptedAgent
from hl_sim.sim import layout as scene_layout
from hl_sim.sim.kick import KickConfig

# ── paths ──────────────────────────────────────────────────────────────────────
SCENE_PATH = paths.DEFAULT_SCENE

# ── timing ─────────────────────────────────────────────────────────────────────
CONTROL_HZ      = 50
SIM_HZ          = 200
STEPS_PER_CTRL  = SIM_HZ // CONTROL_HZ   # = 4

# ── constantes da cena ────────────────────────────────────────────────────────
# Os índices de qpos/qvel/ctrl/sensor NÃO moram mais aqui: são resolvidos por
# nome, a partir do modelo carregado, em hl_sim/sim/layout.py.  Só sobram os
# números que são escolha de controle, não estrutura da cena.
TRUNK_HEIGHT = 0.75

NUM_ROBOTS              = scene_layout.NUM_ROBOTS
NUM_ACTUATORS_PER_ROBOT = scene_layout.JOINTS_PER_ROBOT

# ── default joint targets (23 actuators) ──────────────────────────────────────
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
    x:   float = 0.0
    y:   float = 0.0
    yaw: float = 0.0


# ── SimBridge ──────────────────────────────────────────────────────────────────

class SimBridge:
    def __init__(self, agents: list[AgentInterface], kick: Optional[KickConfig] = None) -> None:
        paths.require(SCENE_PATH, "cena do MuJoCo")
        if len(agents) > NUM_ROBOTS:
            raise ValueError(f"A cena suporta no máximo {NUM_ROBOTS} robôs.")

        self.m = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.d = mujoco.MjData(self.m)
        self.agents = agents
        # Índices derivados do modelo por nome; estoura aqui, com mensagem clara,
        # se a cena mudar de forma incompatível.
        self.layout = scene_layout.resolve(self.m)
        self.kick = kick if kick is not None else KickConfig.load()
        self._disable_robot_ball_contacts()

        self._kin:             list[_KinState] = [_KinState() for _ in agents]
        self._shoot_remaining: list[int]        = [0] * len(agents)
        self._kick_consumed = [False] * len(self.agents)
        self._last_kick_id = [0] * len(self.agents)
        self._kick_force = [np.zeros(2) for _ in self.agents]
        self._tick = 0

        _ok(f"Modelo carregado  nbody={self.m.nbody}  nu={self.m.nu}  "
            f"nq={self.m.nq}  nv={self.m.nv}")
        _ok(f"Agentes ({len(agents)}): {[a.robot_name for a in agents]}")

    # ── episode reset ──────────────────────────────────────────────────────────

    def _disable_robot_ball_contacts(self) -> None:
        # Bit exclusivo: mantém bola–chão/gols sem bola–robôs.
        # Geometrias visuais (máscaras zero) continuam sem colisão.
        ball_bit = 1 << 29
        roots = {r.trunk_body_id for r in self.layout.robots}
        for geom in range(self.m.ngeom):
            body = int(self.m.geom_bodyid[geom])
            if not (self.m.geom_contype[geom] or self.m.geom_conaffinity[geom]):
                continue
            if body == self.layout.ball_body_id:
                self.m.geom_contype[geom] = ball_bit
                self.m.geom_conaffinity[geom] = ball_bit
                continue
            while body and body not in roots:
                body = int(self.m.body_parentid[body])
            if body not in roots:
                self.m.geom_contype[geom] |= ball_bit
                self.m.geom_conaffinity[geom] |= ball_bit

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        mujoco.mj_forward(self.m, self.d)

        for i, ks in enumerate(self._kin):
            adr = self.layout.robots[i].trunk_qpos_adr
            q   = self.d.qpos[adr: adr + 7]
            ks.x   = float(q[0])
            ks.y   = float(q[1])
            ks.yaw = _quat_to_yaw(q[3:7])

        self._shoot_remaining = [0] * len(self.agents)
        self._kick_consumed = [False] * len(self.agents)
        self._last_kick_id = [0] * len(self.agents)
        self._kick_force = [np.zeros(2) for _ in self.agents]
        self._tick = 0

        for agent in self.agents:
            agent.reset()

    # ── sensor / action ────────────────────────────────────────────────────────

    def _sensor(self, i: int) -> SensorState:
        rl = self.layout.robots[i]
        # framequat(4) seguido de gyro(3), na ordem em que a cena os declara.
        sens_end = rl.gyro_sensor_adr + 3
        return SensorState(
            qpos       = self.d.qpos.copy(),
            qvel       = self.d.qvel.copy(),
            trunk_pos  = self.d.xpos[rl.trunk_body_id].copy(),
            trunk_quat = self.d.xquat[rl.trunk_body_id].copy(),
            ball_pos   = self.d.xpos[self.layout.ball_body_id].copy(),
            tick       = self._tick,
            sensordata = self.d.sensordata[rl.quat_sensor_adr:sens_end].copy(),
        )

    def _apply(self, i: int, cmd: ActionCmd) -> None:
        ks         = self._kin[i]
        dt         = 1.0 / CONTROL_HZ
        rl         = self.layout.robots[i]
        ctrl_slice = rl.ctrl_slice
        qposadr    = rl.trunk_qpos_adr
        dofadr     = rl.trunk_dof_adr
        trunk_id   = rl.trunk_body_id
        ball_id    = self.layout.ball_body_id

        if cmd.joint_pos is not None:
            ctrl = HOME_CTRL.copy()
            ctrl[0] = cmd.head_yaw
            ctrl[1] = cmd.head_pitch
            n = min(len(cmd.joint_pos), NUM_ACTUATORS_PER_ROBOT - 2)
            ctrl[2: 2 + n] = cmd.joint_pos[:n]
            self.d.ctrl[ctrl_slice] = ctrl
        else:
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

            ctrl = HOME_CTRL.copy()
            ctrl[0] = cmd.head_yaw
            ctrl[1] = cmd.head_pitch
            self.d.ctrl[ctrl_slice] = ctrl

        if not cmd.shoot or cmd.kick_id != self._last_kick_id[i]:
            self._kick_consumed[i] = False
        self._last_kick_id[i] = cmd.kick_id

        ball_pos = self.d.xpos[ball_id]
        delta = ball_pos[:2] - self.d.qpos[qposadr:qposadr + 2]
        trunk_yaw = _quat_to_yaw(self.d.qpos[qposadr + 3:qposadr + 7])
        angle = math.atan2(delta[1], delta[0]) - trunk_yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        can_kick = (
            np.linalg.norm(delta) <= self.kick.distance_m
            and abs(angle) <= math.radians(self.kick.max_angle_degrees)
            and ball_pos[2] <= self.kick.max_ball_height_m
        )
        if (cmd.shoot and not self._kick_consumed[i] and can_kick
                and not any(self._shoot_remaining)
                and not np.any(self.d.xfrc_applied[ball_id, :2])):
            self._kick_consumed[i] = True
            self._shoot_remaining[i] = max(1, math.ceil(self.kick.duration_seconds * CONTROL_HZ))
            self._kick_force[i] = self.kick.force_newtons * np.array([
                math.cos(trunk_yaw), math.sin(trunk_yaw),
            ])

        if self._shoot_remaining[i] > 0:
            fx, fy = self._kick_force[i]
            self.d.xfrc_applied[ball_id, 0] += fx
            self.d.xfrc_applied[ball_id, 1] += fy
            self._shoot_remaining[i] -= 1

    # ── control tick ───────────────────────────────────────────────────────────

    def _ctrl_tick(self) -> None:
        self.d.xfrc_applied[self.layout.ball_body_id] = 0.0
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
                    trunk_z = self.d.xpos[self.layout.robots[0].trunk_body_id][2]
                    ball    = self.d.xpos[self.layout.ball_body_id]
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
    """
    Execução standalone, SEM ROS2 e SEM GameController — só agentes scriptados.

    Para rodar a cadeia de competição completa (brain + GameController oficial),
    use hl_sim/cli/run_ros2.py.
    """
    parser = argparse.ArgumentParser(
        description="SimBridge standalone — MuJoCo 3v3, agentes scriptados"
    )
    parser.add_argument("--viewer",   action="store_true")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--speed",    type=float, default=1.0)
    parser.add_argument("--force-playing", action="store_true",
                        help="Injeta STATE_PLAYING nos agentes. Atalho de "
                             "desenvolvimento: sem GameController os robôs ficam "
                             "parados em INITIAL, que é o comportamento correto.")
    args = parser.parse_args()

    print(f"\n{BOLD}T1 Soccer Sim — 3v3 standalone{RESET}")

    # Ordem da cena (soccer_scene.xml): robots 1, 3, 5 | 2, 4, 6
    agents = [
        ScriptedAgent("T1_1", team_id=56, player_id=1),
        ScriptedAgent("T1_3", team_id=56, player_id=2),
        ScriptedAgent("T1_5", team_id=56, player_id=3),
        ScriptedAgent("T1_2", team_id=57, player_id=1),
        ScriptedAgent("T1_4", team_id=57, player_id=2),
        ScriptedAgent("T1_6", team_id=57, player_id=3),
    ]

    if args.force_playing:
        playing = GameState(state=STATE_PLAYING, received=True)
        for agent in agents:
            agent.on_game_state(playing)
        _info("--force-playing: agentes em STATE_PLAYING (atalho de dev)")
    else:
        _info("sem GameController: agentes ficam em INITIAL. "
              "Use --force-playing para movê-los.")

    bridge = SimBridge(agents=agents)
    bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)


if __name__ == "__main__":
    main()
