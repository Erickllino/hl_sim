"""
hl_sim/agents/scripted.py — agente scriptado (sem ROS2 na lógica de decisão).

Persegue e chuta a bola usando ground truth do MuJoCo.  Serve de adversário e de
preenchimento de time enquanto não há um brain por robô.

Obedece ao GameController real: só joga quando o estado é PLAYING e o jogador não
está punido.  Fora disso fica parado, como um robô de verdade obedecendo ao árbitro.
"""
import math
from enum import Enum, auto

import numpy as np

from hl_sim.agents.base import AgentInterface, ActionCmd, SensorState


class _Phase(Enum):
    STOP     = auto()   # fora de jogo (INITIAL/READY/SET/FINISHED ou punido)
    SEEK     = auto()   # procurando a bola
    ALIGN    = auto()   # girando para encarar a bola
    APPROACH = auto()   # andando até a bola
    KICK     = auto()   # perto o bastante — chuta


class ScriptedAgent(AgentInterface):
    """
    Máquina de estados: STOP → SEEK → ALIGN → APPROACH → KICK → SEEK …

    Constantes de classe (dá para sobrescrever por instância após construir):
        WALK_SPEED    velocidade à frente na aproximação (m/s)
        TURN_SPEED    velocidade angular no alinhamento  (rad/s)
        KICK_DIST     distância máxima para chutar       (m)
        ALIGN_TOL     tolerância de yaw para parar o giro (rad)
    """

    WALK_SPEED = 0.30   # m/s
    TURN_SPEED = 0.80   # rad/s
    KICK_DIST  = 0.35   # m
    ALIGN_TOL  = 0.15   # rad  ≈ 8.6°
    KICK_TICKS = 10     # control ticks segurando o flag de chute

    def __init__(
        self,
        robot_name: str = "robot",
        team_id:    int = 0,
        player_id:  int = 1,
    ) -> None:
        super().__init__(robot_name, team_id=team_id, player_id=player_id)
        self._phase     = _Phase.STOP
        self._kick_held = 0

    # ── reset de episódio ──────────────────────────────────────────────────────

    def reset(self) -> None:
        self._phase     = _Phase.STOP
        self._kick_held = 0

    # ── helpers de geometria ───────────────────────────────────────────────────

    @staticmethod
    def _yaw_from_quat(q: np.ndarray) -> float:
        """Extrai yaw do quaternion do MuJoCo (w, x, y, z)."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _signed_angle_to(robot_pos: np.ndarray, robot_yaw: float,
                         target: np.ndarray) -> float:
        """Ângulo com sinal (rad) que o robô precisa girar para encarar o alvo."""
        dx = target[0] - robot_pos[0]
        dy = target[1] - robot_pos[1]
        err = math.atan2(dy, dx) - robot_yaw
        return (err + math.pi) % (2.0 * math.pi) - math.pi

    # ── passo de controle ──────────────────────────────────────────────────────

    def step(self, state: SensorState) -> ActionCmd:
        cmd = ActionCmd()

        # ── árbitro manda: sem PLAYING (ou punido), o robô não joga ────────────
        if not self.is_active():
            self._phase     = _Phase.STOP
            self._kick_held = 0
            # TODO(fidelidade): em READY o robô deveria caminhar até sua posição
            # de kickoff, e em SET ficar imóvel esperando o apito.
            return cmd

        # entrou em jogo agora
        if self._phase == _Phase.STOP:
            self._phase = _Phase.SEEK

        pos   = state.trunk_pos
        yaw   = self._yaw_from_quat(state.trunk_quat)
        ball  = state.ball_pos
        dist  = float(np.linalg.norm(ball[:2] - pos[:2]))
        angle = self._signed_angle_to(pos, yaw, ball)

        # ── SEEK: com ground truth a direção já é conhecida ───────────────────
        if self._phase == _Phase.SEEK:
            self._phase = _Phase.ALIGN

        # ── ALIGN: gira para encarar a bola ───────────────────────────────────
        if self._phase == _Phase.ALIGN:
            if abs(angle) < self.ALIGN_TOL:
                self._phase = _Phase.APPROACH
            else:
                cmd.vyaw = math.copysign(self.TURN_SPEED, angle)

        # ── APPROACH: anda até a bola corrigindo o rumo ───────────────────────
        elif self._phase == _Phase.APPROACH:
            if dist <= self.KICK_DIST:
                self._phase = _Phase.KICK
            else:
                cmd.vx = self.WALK_SPEED
                if abs(angle) > self.ALIGN_TOL:
                    cmd.vyaw = math.copysign(
                        min(self.TURN_SPEED, abs(angle) * 1.5), angle
                    )

        # ── KICK: segura o flag por KICK_TICKS e recomeça ─────────────────────
        elif self._phase == _Phase.KICK:
            cmd.shoot = True
            self._kick_held += 1
            if self._kick_held >= self.KICK_TICKS:
                self._phase     = _Phase.SEEK
                self._kick_held = 0

        return cmd
