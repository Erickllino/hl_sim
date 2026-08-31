"""
hl_sim/agents/base.py — contrato abstrato entre o SimBridge e os agentes.

O simulador **não produz** estado de jogo.  Na competição o caminho é:

    [GameController oficial] ──UDP 3838──▶ [game_controller_node] ──ROS2──▶ brain
                             ◀──UDP 3939── [brain_communication]

O sim entra nesse fluxo como **consumidor**: assina /robocup/game_controller
(publicado pelo game_controller_node) e distribui o GameState para os agentes,
exatamente como o brain faz.  Ver hl_sim/ros2/game_controller_link.py.

Implementações:
  • ScriptedAgent (hl_sim.agents.scripted)  — lógica em Python puro, sem ROS2
  • ROS2Agent    (hl_sim.ros2.agent)        — ponte para o brain do hsl-player

Este módulo NÃO importa rclpy.  Nada em hl_sim.agents nem em hl_sim.sim importa.
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

# O protocolo do árbitro tem dono próprio; aqui só reexportamos o que os agentes
# usam, para não espalhar `from ...protocol.gamecontroller import` por toda parte.
from hl_sim.protocol.gamecontroller import (  # noqa: F401
    GameState,
    HL_MAX_NUM_PLAYERS,
    KICKING_TEAM_NONE,
    PENALTY_NONE,
    STATE_INITIAL, STATE_READY, STATE_SET, STATE_PLAYING, STATE_FINISHED,
    GAME_PHASE_NORMAL, GAME_PHASE_PENALTY_SHOOT_OUT,
    GAME_PHASE_EXTRA_TIME, GAME_PHASE_TIMEOUT,
    SET_PLAY_NONE, SET_PLAY_DIRECT_FREE_KICK, SET_PLAY_INDIRECT_FREE_KICK,
    SET_PLAY_PENALTY_KICK, SET_PLAY_THROW_IN, SET_PLAY_GOAL_KICK,
    SET_PLAY_CORNER_KICK,
)


# ── contratos de dados do loop de simulação ────────────────────────────────────

@dataclass
class SensorState:
    """Snapshot de sensores publicado pelo SimBridge a cada control tick."""
    qpos:       np.ndarray            # (nq,) qpos completo do modelo
    qvel:       np.ndarray            # (nv,) qvel completo do modelo
    trunk_pos:  np.ndarray            # (3,)  XYZ do Trunk no mundo
    trunk_quat: np.ndarray            # (4,)  quaternion do Trunk (w x y z)
    ball_pos:   np.ndarray            # (3,)  XYZ da bola no mundo
    tick:       int = 0               # contador de control ticks
    sensordata: Optional[np.ndarray] = None  # framequat(4) + gyro(3)


@dataclass
class ActionCmd:
    """Comando emitido pelo agente → consumido pelo SimBridge a cada control tick."""
    vx:         float = 0.0   # frente   (m/s, frame do corpo)
    vy:         float = 0.0   # lateral  (m/s, frame do corpo)
    vyaw:       float = 0.0   # rotação  (rad/s)
    head_pitch: float = 0.0   # rad
    head_yaw:   float = 0.0   # rad
    shoot:      bool  = False
    joint_pos:  Optional[np.ndarray] = None  # 21 alvos de junta (/joint_ctrl)


# ── interface abstrata ─────────────────────────────────────────────────────────

class AgentInterface(ABC):
    """
    Uma instância por robô.  O SimBridge chama:
        agent.reset()             — no início do episódio
        cmd = agent.step(state)   — a cada control tick (50 Hz)

    O GameControllerLink chama:
        agent.on_game_state(gs)   — quando chega pacote do GameController
    """

    def __init__(
        self,
        robot_name: str = "robot",
        team_id:    int = 0,
        player_id:  int = 1,
    ) -> None:
        self.robot_name = robot_name
        self.team_id    = team_id
        self.player_id  = player_id
        # Protege a troca de referência do GameState entre a thread do rclpy
        # (callback do GC) e a thread do loop de simulação.
        self._lock = threading.Lock()
        self._game = GameState()

    # ── estado de jogo (vindo do GameController real) ─────────────────────────

    @property
    def game(self) -> GameState:
        with self._lock:
            return self._game

    def on_game_state(self, gs: GameState) -> None:
        with self._lock:
            self._game = gs

    def is_active(self) -> bool:
        """True quando o robô pode jogar: PLAYING, não parado e não punido."""
        gs = self.game
        return gs.is_playing() and not gs.is_penalized(self.player_id)

    # ── contrato do agente ─────────────────────────────────────────────────────

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def step(self, state: SensorState) -> ActionCmd: ...
