import math
from .agent_interface import AgentInterface,ActionCmd, SensorState
import numpy as np

from enum import Enum, auto

GC_INITIAL  = 0
GC_READY    = 1
GC_SET      = 2
GC_PLAYING  = 3


class _Phase(Enum):
    STOP    = auto()   # not used, but could be for pre-game or post-goal pause
    RESTART = auto()   # not used, but could be for auto-restart after goal
    SEEK     = auto()   # turn in place to locate ball
    ALIGN    = auto()   # rotate to face ball
    APPROACH = auto()   # walk toward ball
    KICK     = auto()   # close enough — kick


class DefaultAgent(AgentInterface):
    """
    Scripted agent that chases and kicks the ball (no ROS2 required).

    State machine:  SEEK → ALIGN → APPROACH → KICK → SEEK …

    Class-level constants (override per-instance after construction):
        WALK_SPEED    forward speed while approaching (m/s)
        TURN_SPEED    angular speed when aligning      (rad/s)
        KICK_DIST     max distance to trigger shoot    (m)
        ALIGN_TOL     yaw-error tolerance to stop turn (rad)
    """

    WALK_SPEED = 0.30   # m/s
    TURN_SPEED = 0.80   # rad/s
    KICK_DIST  = 0.35   # m
    ALIGN_TOL  = 0.15   # rad  ≈ 8.6°
    KICK_TICKS = 10     # ctrl ticks to hold shoot flag

    def __init__(self, robot_name: str = "robot") -> None:
        super().__init__(robot_name)
        self._game_state = GC_INITIAL
        self._phase      = _Phase.STOP
        self._kick_held  = 0

    # ── episode reset ──────────────────────────────────────────────────────────

    def reset(self) -> None:
        self._phase     = _Phase.STOP
        self._kick_held = 0
        # TODO: Volta para posição incial

    # ── geometry helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _yaw_from_quat(q: np.ndarray) -> float:
        """Extract yaw from MuJoCo quaternion (w, x, y, z)."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _signed_angle_to(robot_pos: np.ndarray, robot_yaw: float,
                         target: np.ndarray) -> float:
        """Signed angle (rad) robot must turn to face target, in [-π, π]."""
        dx = target[0] - robot_pos[0]
        dy = target[1] - robot_pos[1]
        desired = math.atan2(dy, dx)
        err = desired - robot_yaw
        return (err + math.pi) % (2.0 * math.pi) - math.pi

    # ── control step ───────────────────────────────────────────────────────────

    def step(self, state: SensorState) -> ActionCmd:
        pos   = state.trunk_pos
        yaw   = self._yaw_from_quat(state.trunk_quat)
        ball  = state.ball_pos
        dist  = float(np.linalg.norm(ball[:2] - pos[:2]))
        angle = self._signed_angle_to(pos, yaw, ball)
        cmd   = ActionCmd()

        # ── SEEK: spin until we have a bearing (immediate if ball pos is known) ─
        if self._phase == _Phase.SEEK:
            self._phase = _Phase.ALIGN

        # ── ALIGN: rotate to face ball ─────────────────────────────────────────
        if self._phase == _Phase.ALIGN:
            if abs(angle) < self.ALIGN_TOL:
                self._phase = _Phase.APPROACH
            else:
                cmd.vyaw = math.copysign(self.TURN_SPEED, angle)

        # ── APPROACH: walk toward ball, correct heading continuously ───────────
        elif self._phase == _Phase.APPROACH:
            if dist <= self.KICK_DIST:
                self._phase = _Phase.KICK
            else:
                cmd.vx = self.WALK_SPEED
                if abs(angle) > self.ALIGN_TOL:
                    cmd.vyaw = math.copysign(
                        min(self.TURN_SPEED, abs(angle) * 1.5), angle
                    )

        # ── KICK: hold shoot flag for KICK_TICKS then restart ─────────────────
        elif self._phase == _Phase.KICK:
            cmd.shoot = True
            self._kick_held += 1
            if self._kick_held >= self.KICK_TICKS:
                self._phase     = _Phase.SEEK
                self._kick_held = 0

        if self._phase == _Phase.STOP:
            pass  # do nothing

        

        return cmd