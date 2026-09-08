#!/usr/bin/env python3
"""
hl_sim/cli/run_ros2.py — SimBridge + N brains do hsl-player + GameController real.

Cadeia de competição reproduzida por inteiro, N ≤ 6 robôs com brain de verdade:

    [GameController oficial]
        │ UDP 3838 broadcast                  ▲ UDP 3939 unicast (RGrt v4)
        ▼                                     │  enviado por cada brain
    ┌─────────────────── container robotN (ROS_DOMAIN_ID = N) ───────────────┐
    │  [game_controller_node] ──/robocup/game_controller──▶ [brain_node]     │
    └────────────────────────────┬──────────────────────────────────────────┘
                                 │  mesmo domínio DDS
                    ┌────────────┴─────────────┐
                    ▼                          ▼
             [GameControllerLink]         LocoApiTopicReq
                    │                          │
                    └────────▶ [RobotLink] ◀───┘
                                   │
                          [SimBridge / MuJoCo]  ── um só, todas as ilhas
                                   │ /low_state /odometer_state /booster_vision/*

Os robôs NÃO listados em --brains recebem um ScriptedAgent (Python puro, sem
ROS2).  A cena sempre tem os 6 corpos, então os índices de qpos/ctrl não mudam
com N — só muda quem controla cada corpo.

Pré-requisitos para o estado de jogo funcionar:
  1. GameController oficial emitindo broadcast UDP na 3838
  2. o game_controller_node rodando dentro de cada container de robô
  3. team_id/player_id de config/match.yaml iguais aos do config_local.yaml de
     cada brain e ao cadastro dos times no GameController

Sem (1) e (2) os agentes ficam em INITIAL e não se movem — que é o comportamento
correto de um robô sem árbitro.

Uso:
    python -m hl_sim.cli.run_ros2 --brains 1,3 --viewer
    python -m hl_sim.cli.run_ros2 --brains all --duration 120
"""
from __future__ import annotations

import argparse
import sys

from hl_sim import config as match_config
from hl_sim.agents.scripted import ScriptedAgent
from hl_sim.sim.bridge import SimBridge


def _parse_brains(raw: str, cfg: match_config.MatchConfig) -> list[int]:
    """'1,3' → [1, 3];  'all' → todos;  'none' → []."""
    raw = raw.strip().lower()
    if raw in ("", "none"):
        return []
    if raw == "all":
        return [r.number for r in cfg.robots]

    numbers = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            n = int(part)
        except ValueError:
            raise SystemExit(f"--brains: '{part}' não é um número de robô")
        cfg.by_number(n)  # valida contra config/match.yaml
        numbers.append(n)
    return numbers


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SimBridge + brains do hsl-player + GameController real"
    )
    parser.add_argument("--brains", default="all",
                        help="robôs controlados por um brain de verdade: "
                             "'1,3', 'all' ou 'none'  [default: all]")
    parser.add_argument("--viewer", action="store_true",
                        help="Abre o viewer interativo do MuJoCo")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Segundos de simulação (0 = infinito)  [default: 0]")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Fator de tempo real (0 = máximo)  [default: 1.0]")
    parser.add_argument("--config", default=None,
                        help="caminho alternativo para match.yaml")
    args = parser.parse_args()

    cfg = match_config.load(args.config)
    brain_numbers = set(_parse_brains(args.brains, cfg))

    links = []
    if brain_numbers:
        # rclpy só é importado quando algum robô precisa de ROS2: assim
        # `--brains none` roda numa máquina sem ROS2 nenhum.
        from hl_sim.ros2.robot_link import build_links
        links = build_links([r for r in cfg.robots if r.number in brain_numbers])

    links_by_number = {link.cfg.number: link for link in links}

    # Ordem dos agentes = ordem dos slots = ordem dos <include> na cena.
    agents = []
    for robot in cfg.robots:   # já vem ordenado por slot
        link = links_by_number.get(robot.number)
        if link is not None:
            agents.append(link.agent)
        else:
            agents.append(ScriptedAgent(
                robot.name, team_id=robot.team_id, player_id=robot.player_id
            ))

    print(f"· brains ROS2: {sorted(brain_numbers) or '(nenhum)'}")
    print(f"· scriptados : {[r.number for r in cfg.robots if r.number not in brain_numbers] or '(nenhum)'}")

    bridge = SimBridge(agents=agents)
    try:
        bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)
    finally:
        mudos = [l.cfg.name for l in links if l.packets_received == 0]
        if mudos:
            print(
                f"\nAVISO: nenhum pacote do GameController chegou para: "
                f"{', '.join(mudos)}.\n"
                f"  Confira: (1) GameController oficial emitindo na 3838, "
                f"(2) game_controller_node rodando nesses containers, "
                f"(3) ros2 topic hz /robocup/game_controller com o "
                f"ROS_DOMAIN_ID certo.",
                file=sys.stderr,
            )
        for link in links:
            link.shutdown()


if __name__ == "__main__":
    main()
