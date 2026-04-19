#!/usr/bin/env python3
"""
scripts/run_ros2.py — Fase 3: SimBridge + ROS2Agent

Uso:
    uv run python scripts/run_ros2.py --viewer
    uv run python scripts/run_ros2.py --duration 120 --speed 1.0
    uv run sim-ros2 --viewer
"""
import argparse
import threading

import rclpy
from rclpy.node import Node

from bridge.sim_bridge import SimBridge
from bridge.ros2_agent import ROS2Agent, GC_PLAYING
from bridge.default_agent import DefaultAgent


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SimBridge + ROS2Agent — Fase 3 do hl_sim"
    )
    parser.add_argument("--viewer",   action="store_true",
                        help="Abre o viewer interativo MuJoCo")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Segundos de simulação (0 = infinito)  [default: 0]")
    parser.add_argument("--speed",    type=float, default=1.0,
                        help="Fator de velocidade real (0 = máximo)  [default: 1.0]")
    args = parser.parse_args()

    rclpy.init()
    node = Node("hl_sim_bridge")

    agent1 = ROS2Agent("T1_1", node, player_id=1)   # brain-controlled
    agent2 = DefaultAgent("T1_2")                 # scripted opponent

    bridge = SimBridge(agents=[agent1, agent2])

    # seta PLAYING antes de iniciar para o brain não travar em INITIAL
    agent1.set_game_state(GC_PLAYING)

    # ROS2 spin em background para que os callbacks funcionem enquanto o sim roda
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)
    finally:
        rclpy.shutdown()



if __name__ == "__main__":
    main()
