"""
bridge/agent_interface.py — Abstract agent contract + StandaloneAgent (Phase 2)

AgentInterface is transport-agnostic:
  • StandaloneAgent  — scripted Python, no external deps (current)
  • ROS2Agent        — future, see ROADMAP_ROS2.md
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import threading
from game_controller_interface.msg import GameControlData
from typing import Optional

import numpy as np


# ── data contracts ─────────────────────────────────────────────────────────────

@dataclass
class SensorState:
    """Sensor snapshot published by SimBridge each control tick."""
    qpos:       np.ndarray            # shape (nq,) — full model qpos
    qvel:       np.ndarray            # shape (nv,) — full model qvel
    trunk_pos:  np.ndarray            # shape (3,)  — Trunk world XYZ
    trunk_quat: np.ndarray            # shape (4,)  — Trunk quaternion w x y z
    ball_pos:   np.ndarray            # shape (3,)  — ball world XYZ
    tick:       int = 0               # control tick counter
    sensordata: Optional[np.ndarray] = None  # MuJoCo sensordata (orientation quat + gyro)


@dataclass
class ActionCmd:
    """Command issued by agent → consumed by SimBridge each control tick."""
    vx:         float = 0.0   # forward  (m/s, body frame)
    vy:         float = 0.0   # lateral  (m/s, body frame)
    vyaw:       float = 0.0   # rotation (rad/s)
    head_pitch: float = 0.0   # rad
    head_yaw:   float = 0.0   # rad
    shoot:      bool  = False
    joint_pos:  Optional[np.ndarray] = None  # 21 body joint targets from deploy (/joint_ctrl)


# ── abstract interface ─────────────────────────────────────────────────────────

# TODO: Entender oque é o _lock

class AgentInterface(ABC):
    """
    One instance per robot.  SimBridge calls:
        agent.reset()                   — on episode start
        cmd = agent.step(state)         — every control tick (50 Hz)

    Implementors:
        StandaloneAgent  — pure-Python scripted logic (no ROS2)
        ROS2Agent        — bridges to/from hsl-player over ROS2 topics
    """

    def __init__(self, robot_name: str = "robot", team_id: int = 0) -> None:
        self.robot_name = robot_name
        self._team_id   = team_id
        self._lock = threading.Lock()

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def step(self, state: SensorState) -> ActionCmd: ...

    def _publish_game_controller(self) -> None:
        from game_controller_interface.msg import TeamInfo, RobotInfo
        msg = GameControlData()
        with self._lock:
            msg.state = self._game_state
        msg.secondary_state = 0
        # brain itera players[0..HL_MAX_NUM_PLAYERS-1] (11 players) — lista vazia causa segfault
        HL_MAX_NUM_PLAYERS = 11
        my_team = TeamInfo()
        my_team.team_number = self._team_id
        my_team.players = [RobotInfo() for _ in range(HL_MAX_NUM_PLAYERS)]
        oppo_team = TeamInfo()
        oppo_team.players = [RobotInfo() for _ in range(HL_MAX_NUM_PLAYERS)]
        msg.teams = [my_team, oppo_team]
        self._pub_gc.publish(msg)

    def set_game_state(self, state: int) -> None:
        with self._lock:
            self._game_state = state


# ── standalone scripted agent ──────────────────────────────────────────────────




