"""
hl_sim/ros2/game_controller_link.py — o sim como CONSUMIDOR do GameController real.

Caminho de competição (idêntico ao de campo):

    [GameController oficial]  ──UDP broadcast 3838──▶  [game_controller_node]
                                                              │ ROS2
                                    ┌─────────────────────────┴─────────────────┐
                                    ▼                                           ▼
                              [brain do hsl-player]                    [GameControllerLink]
                                    │                                           │
                                    └──UDP 3939 (RGrt v4)──▶ GameController      ▼
                                                                          agentes do sim

O sim NÃO publica /robocup/game_controller — quem publica é o game_controller_node
do hsl-player, a partir dos pacotes UDP do árbitro.  Publicar aqui criaria um
segundo publisher no mesmo tópico e o brain receberia estados intercalados.

Este módulo só traduz a mensagem para GameState e a entrega aos agentes, resolvendo
"meu time" vs "adversário" pelo team_id de cada agente — a mesma lógica de
brain.cpp:1395-1425.
"""
from __future__ import annotations

from typing import Iterable, Optional

from rclpy.node import Node
from game_controller_interface.msg import GameControlData

from hl_sim.agents.base import AgentInterface
from hl_sim.protocol.gamecontroller import (
    GameState,
    HL_MAX_NUM_PLAYERS,
    STATE_INITIAL,
    STRUCT_VERSION,
)

GC_TOPIC = "/robocup/game_controller"

# Reexportado de hl_sim.protocol.gamecontroller — fonte única do protocolo.
EXPECTED_STRUCT_VERSION = STRUCT_VERSION


class GameControllerLink:
    """
    Assina /robocup/game_controller e distribui GameState para os agentes.

    Uso:
        link = GameControllerLink(node, agents)
        # ... rclpy.spin(node) em background
    """

    def __init__(
        self,
        node:   Node,
        agents: Iterable[AgentInterface],
        topic:  str = GC_TOPIC,
    ) -> None:
        self._node   = node
        self._agents = list(agents)
        self._packets = 0
        self._warned_teams: set = set()
        self._warned_version = False

        node.create_subscription(GameControlData, topic, self._on_gc, 10)

        teams = sorted({a.team_id for a in self._agents})
        node.get_logger().info(
            f"GameControllerLink assinando {topic} "
            f"({len(self._agents)} agentes, team_ids={teams}). "
            f"Aguardando o game_controller_node publicar pacotes do árbitro."
        )

    # ── estatística ────────────────────────────────────────────────────────────

    @property
    def packets_received(self) -> int:
        return self._packets

    # ── callback ───────────────────────────────────────────────────────────────

    def _on_gc(self, msg: GameControlData) -> None:
        self._packets += 1

        version = int(msg.version)
        if version != EXPECTED_STRUCT_VERSION and not self._warned_version:
            self._warned_version = True
            self._node.get_logger().warning(
                f"GameControlData com version={version}, esperado "
                f"{EXPECTED_STRUCT_VERSION}. O game_controller_node do hsl-player "
                f"descarta pacotes com versão diferente — confira a versão do "
                f"struct emitida pelo GameController."
            )

        for agent in self._agents:
            gs = self._to_game_state(msg, agent.team_id)
            if gs is not None:
                agent.on_game_state(gs)

    # ── conversão ──────────────────────────────────────────────────────────────

    def _to_game_state(
        self,
        msg:     GameControlData,
        team_id: int,
    ) -> Optional[GameState]:
        """
        Resolve o pacote do ponto de vista de um time.  Retorna None se o pacote
        não contém esse time — mesmo critério de brain.cpp:1407.
        """
        teams = list(msg.teams)
        mine = oppo = None
        for i, team in enumerate(teams):
            if int(team.team_number) == team_id:
                mine = team
                oppo = teams[1 - i]
                break

        if mine is None:
            if team_id not in self._warned_teams:
                self._warned_teams.add(team_id)
                present = [int(t.team_number) for t in teams]
                self._node.get_logger().warning(
                    f"Pacote do GameController não contém o team_id={team_id} "
                    f"(times no pacote: {present}). O estado de jogo desses agentes "
                    f"vai ficar em INITIAL. Alinhe o team_id do sim, o game.team_id "
                    f"do config.yaml do brain e o número do time no GameController."
                )
            return None

        penalties = tuple(
            int(p.penalty) for p in list(mine.players)[:HL_MAX_NUM_PLAYERS]
        )

        return GameState(
            state          = int(msg.state),
            game_phase     = int(msg.game_phase),
            set_play       = int(msg.set_play),
            stopped        = bool(msg.stopped),
            secs_remaining = int(msg.secs_remaining),
            secondary_time = int(msg.secondary_time),
            kicking_team   = int(msg.kicking_team),
            my_score       = int(mine.score),
            oppo_score     = int(oppo.score),
            penalties      = penalties,
            packet_number  = int(msg.packet_number),
            received       = True,
        )
