"""
hl_sim/ros2/robot_link.py — uma ilha DDS por robô.

Por que um contexto rclpy separado por robô
-------------------------------------------
Os tópicos do brain são **absolutos** (brain.cpp:193-197, main.cpp:45-46):

    /low_state   /odometer_state   /booster_vision/detection
    /robocup/game_controller   /remote_controller_state

Nome absoluto ignora `__ns:=`, então namespace ROS2 não separa dois brains — os
seis se atropelariam nos mesmos tópicos.  A separação é feita por
**ROS_DOMAIN_ID**, um por robô.

Isso não é contorno: é o que já acontece no robô real.  O `configs/fastdds.xml`
do hsl-player restringe o DDS a 127.0.0.1 e às interfaces internas do robô, então
no campo cada robô é uma ilha DDS e o único canal entre eles é o UDP do árbitro
(3838 broadcast / 3939 unicast).  Domínios separados reproduzem exatamente isso.

Cada RobotLink abre seu próprio `rclpy.Context` no domínio do robô, com o nó, o
ROS2Agent e o GameControllerLink daquele container.  O SimBridge continua sendo
um só, numa thread só, falando com todos os agentes.
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from hl_sim.config import RobotConfig
from hl_sim.ros2.agent import ROS2Agent
from hl_sim.ros2.game_controller_link import GameControllerLink


class RobotLink:
    """Contexto rclpy + nó + agente + link do GameController de um robô."""

    def __init__(self, cfg: RobotConfig) -> None:
        self.cfg = cfg

        # Contexto próprio: é o que amarra este nó ao ROS_DOMAIN_ID do robô,
        # sem tocar na variável de ambiente do processo (que é global e serviria
        # para um robô só).
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=cfg.domain_id)

        self.node = Node(f"hl_sim_{cfg.name}", context=self.context)
        self.agent = ROS2Agent(
            cfg.name, self.node, player_id=cfg.player_id, team_id=cfg.team_id
        )
        # O sim consome o estado de jogo do mesmo tópico que o brain consome,
        # publicado pelo game_controller_node que roda no container deste robô.
        self.gc_link = GameControllerLink(self.node, [self.agent])

        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self.node)
        self._thread: Optional[threading.Thread] = None

        self.node.get_logger().info(
            f"{cfg.name}: domínio DDS {cfg.domain_id}, "
            f"team_id={cfg.team_id}, player_id={cfg.player_id}, role={cfg.role}"
        )

    @property
    def packets_received(self) -> int:
        return self.gc_link.packets_received

    def start(self) -> None:
        """Roda o executor numa thread, para o loop do MuJoCo não bloquear."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._spin, name=f"spin-{self.cfg.name}", daemon=True
        )
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            # Contexto derrubado no shutdown: saída normal, não erro.
            if self.context.ok():
                raise

    def shutdown(self) -> None:
        self._executor.shutdown()
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)


def build_links(robots: list[RobotConfig]) -> list[RobotLink]:
    """Cria um RobotLink por robô e começa a girar todos."""
    links = [RobotLink(cfg) for cfg in robots]
    for link in links:
        link.start()
    return links
