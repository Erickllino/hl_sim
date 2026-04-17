"""
bridge/ros2_agent.py — Fase 3: ponte ROS2 entre SimBridge e hsl-player

Publica sensores do simulador nos tópicos esperados pelo brain e converte
comandos ROS2 recebidos do brain em ActionCmd para o SimBridge.

Tópicos publicados (sim → brain):
    /low_state                    booster_interface/msg/LowState
    /odometer_state               booster_interface/msg/Odometer
    /booster_vision/detection     vision_interface/msg/Detections
    /booster_vision/line_segments vision_interface/msg/LineSegments
    /robocup/game_controller      game_controller_interface/msg/GameControlData
    /head_pose                    geometry_msgs/msg/Pose

Tópicos assinados (brain → sim):
    LocoApiTopicReq               booster_msgs/msg/RpcReqMsg
    /rl_move                      geometry_msgs/msg/Twist
"""
from __future__ import annotations

import json
import math
import threading
import time as _time

import numpy as np
import rclpy
from rclpy.node import Node

from booster_msgs.msg import RpcReqMsg
from booster_interface.msg import LowState, Odometer, MotorState
from vision_interface.msg import Detections, DetectedObject, LineSegments
from game_controller_interface.msg import GameControlData
from geometry_msgs.msg import Pose, Twist

from bridge.agent_interface import AgentInterface, ActionCmd, SensorState

# ── LocoApiTopicReq api_ids ────────────────────────────────────────────────────
KMOVE        = 2001   # {vx, vy, vyaw}
KROTATE_HEAD = 2004   # {yaw, pitch}
KSHOOT       = 2024   # {} — one-shot

# ── índices das juntas no qpos/qvel (de sim_bridge.py) ────────────────────────
# qpos[0:7]   = ball free-joint (x y z qw qx qy qz)
# qpos[7:14]  = trunk free-joint (x y z qw qx qy qz)
# qpos[14:37] = 23 posições de junta
# qvel[0:6]   = ball dof
# qvel[6:12]  = trunk dof
# qvel[12:35] = 23 velocidades de junta
JOINT_QPOS_START = 14
JOINT_QVEL_START = 12
NUM_JOINTS       = 23

# ── GameControlData state values ───────────────────────────────────────────────
GC_INITIAL  = 0
GC_READY    = 1
GC_SET      = 2
GC_PLAYING  = 3


class ROS2Agent(AgentInterface):
    """
    Ponte entre SimBridge e hsl-player via ROS2.
    Um nó por robô; use player_id para diferenciar namespaces futuros.

    Uso típico:
        rclpy.init()
        node  = Node('hl_sim_bridge')
        agent = ROS2Agent('T1_1', node, player_id=1)
        bridge = SimBridge(agents=[agent])
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        bridge.run(viewer=True)
    """

    def __init__(self, robot_name: str, node: Node, player_id: int = 1) -> None:
        super().__init__(robot_name)
        self._node       = node
        self._cmd        = ActionCmd()
        self._lock       = threading.Lock()
        self._game_state = GC_INITIAL

        # ── publishers (sim → brain) ───────────────────────────────────────────
        self._pub_low   = node.create_publisher(LowState,        '/low_state',                    10)
        self._pub_odom  = node.create_publisher(Odometer,        '/odometer_state',               10)
        self._pub_det   = node.create_publisher(Detections,      '/booster_vision/detection',     10)
        self._pub_lines = node.create_publisher(LineSegments,    '/booster_vision/line_segments', 10)
        self._pub_gc    = node.create_publisher(GameControlData, '/robocup/game_controller',      10)
        self._pub_head  = node.create_publisher(Pose,            '/head_pose',                    10)

        # ── subscribers (brain → sim) ──────────────────────────────────────────
        node.create_subscription(RpcReqMsg, 'LocoApiTopicReq', self._on_loco,    10)
        node.create_subscription(Twist,     '/rl_move',         self._on_rl_move, 10)

        # ── game controller: publica estado a 1 Hz ─────────────────────────────
        node.create_timer(1.0, self._publish_game_controller)

        node.get_logger().info(
            f"ROS2Agent '{robot_name}' (player_id={player_id}) pronto."
        )

    # ── AgentInterface ─────────────────────────────────────────────────────────

    def reset(self) -> None:
        with self._lock:
            self._cmd        = ActionCmd()
            self._game_state = GC_PLAYING

    def step(self, state: SensorState) -> ActionCmd:
        self._publish_sensors(state)
        with self._lock:
            cmd = ActionCmd(
                vx         = self._cmd.vx,
                vy         = self._cmd.vy,
                vyaw       = self._cmd.vyaw,
                head_yaw   = self._cmd.head_yaw,
                head_pitch = self._cmd.head_pitch,
                shoot      = self._cmd.shoot,
            )
            self._cmd.shoot = False  # shoot é one-shot
        return cmd

    # ── callbacks (brain → sim) ────────────────────────────────────────────────

    def _on_loco(self, msg: RpcReqMsg) -> None:
        try:
            # api_id fica no campo header (JSON string), body contém os parâmetros
            header = json.loads(msg.header) if msg.header else {}
            body   = json.loads(msg.body)   if msg.body   else {}
            api_id = int(header.get('api_id', -1))
        except (json.JSONDecodeError, ValueError):
            self._node.get_logger().warning(f"LocoApiTopicReq inválido: header={msg.header!r} body={msg.body!r}")
            return

        with self._lock:
            if api_id == KMOVE:
                self._cmd.vx   = float(body.get('vx',   0.0))
                self._cmd.vy   = float(body.get('vy',   0.0))
                self._cmd.vyaw = float(body.get('vyaw', 0.0))
            elif api_id == KROTATE_HEAD:
                self._cmd.head_yaw   = float(body.get('yaw',   0.0))
                self._cmd.head_pitch = float(body.get('pitch', 0.0))
            elif api_id == KSHOOT:
                self._cmd.shoot = True

    def _on_rl_move(self, msg: Twist) -> None:
        with self._lock:
            self._cmd.vx   = msg.linear.x
            self._cmd.vy   = msg.linear.y
            self._cmd.vyaw = msg.angular.z

    # ── publishers (sim → brain) ───────────────────────────────────────────────

    def _publish_sensors(self, state: SensorState) -> None:
        self._publish_low_state(state)
        self._publish_odometer(state)
        self._publish_detections(state)
        self._pub_lines.publish(LineSegments())
        self._publish_head_pose(state)

    def _publish_low_state(self, state: SensorState) -> None:
        msg = LowState()
        q  = state.qpos[JOINT_QPOS_START: JOINT_QPOS_START + NUM_JOINTS]
        dq = state.qvel[JOINT_QVEL_START: JOINT_QVEL_START + NUM_JOINTS]
        for i in range(NUM_JOINTS):
            motor = MotorState()
            motor.q  = float(q[i])
            motor.dq = float(dq[i])
            msg.motor_state_serial.append(motor)
        self._pub_low.publish(msg)

    def _publish_odometer(self, state: SensorState) -> None:
        msg = Odometer()
        msg.x = float(state.trunk_pos[0])
        msg.y = float(state.trunk_pos[1])
        q = state.trunk_quat  # (w, x, y, z)
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        msg.theta = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self._pub_odom.publish(msg)

    def _publish_detections(self, state: SensorState) -> None:
        msg = Detections()

        # usa wall clock diretamente — evita sec=0 quando use_sim_time está ativo no brain
        t = _time.time()
        msg.header.stamp.sec     = int(t)
        msg.header.stamp.nanosec = int((t % 1.0) * 1e9)

        ball_rel = _world_to_robot_frame(state.ball_pos, state.trunk_pos, state.trunk_quat)
        det = DetectedObject()
        det.label      = 'Ball'
        det.confidence = 100.0
        # brain reads position_projection[0] and [1] for x,y in robot frame
        # use plain Python lists — ROS2 serializes float32[] correctly from list
        det.position_projection = [float(ball_rel[0]), float(ball_rel[1]), float(ball_rel[2])]
        det.position            = [float(ball_rel[0]), float(ball_rel[1]), float(ball_rel[2])]
        msg.detected_objects.append(det)

        # brain's detectProcessVisionBox reads corner_pos as 5 corners × 2 coords (10 floats)
        # wide forward-facing trapezoid in robot frame (x=forward, y=left)
        msg.corner_pos = [
            8.0,  5.0,   # far-left
            8.0, -5.0,   # far-right
            1.0, -1.5,   # near-right
            1.0,  1.5,   # near-left
            0.0,  0.0,   # robot origin (5th point)
        ]

        self._pub_det.publish(msg)

    def _publish_head_pose(self, state: SensorState) -> None:
        msg = Pose()
        msg.position.x = float(state.trunk_pos[0])
        msg.position.y = float(state.trunk_pos[1])
        msg.position.z = float(state.trunk_pos[2]) + 0.3  # deslocamento aprox. cabeça
        trunk_yaw = _quat_to_yaw(state.trunk_quat)
        with self._lock:
            total_yaw = trunk_yaw + self._cmd.head_yaw
        h = total_yaw * 0.5
        msg.orientation.w = math.cos(h)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(h)
        self._pub_head.publish(msg)

    def _publish_game_controller(self) -> None:
        msg = GameControlData()
        with self._lock:
            msg.state = self._game_state  # campo correto: state (não game_state)
        msg.secondary_state = 0
        self._pub_gc.publish(msg)

    def set_game_state(self, state: int) -> None:
        with self._lock:
            self._game_state = state


# ── helpers de geometria ───────────────────────────────────────────────────────

def _quat_to_yaw(q: np.ndarray) -> float:
    """MuJoCo quaternion (w, x, y, z) → yaw (rad)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _world_to_robot_frame(
    point_world: np.ndarray,
    robot_pos:   np.ndarray,
    robot_quat:  np.ndarray,
) -> np.ndarray:
    """Transforma ponto do referencial mundo para o referencial do robô (yaw only)."""
    yaw = _quat_to_yaw(robot_quat)
    dx  = point_world[0] - robot_pos[0]
    dy  = point_world[1] - robot_pos[1]
    dz  = point_world[2] - robot_pos[2]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([c * dx - s * dy, s * dx + c * dy, dz], dtype=np.float64)
