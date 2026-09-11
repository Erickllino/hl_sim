"""
hl_sim/ros2/agent.py — Fase 3: ponte ROS2 entre SimBridge e hsl-player

Publica sensores do simulador nos tópicos esperados pelo brain e converte
comandos ROS2 recebidos do brain em ActionCmd para o SimBridge.

Tópicos publicados (sim → brain):
    /low_state                    booster_interface/msg/LowState
    /odometer_state               booster_interface/msg/Odometer
    /booster_soccer/detection     vision_interface/msg/Detections
    /booster_soccer/line_segments vision_interface/msg/LineSegments
    /head_pose                    geometry_msgs/msg/Pose

Tópicos assinados:
    LocoApiTopicReq               booster_msgs/msg/RpcReqMsg   (brain → sim: head, shoot)
    /joint_ctrl                   booster_interface/msg/LowCmd (deploy → sim: joint targets)

O sim NÃO publica /robocup/game_controller.  Na competição quem publica esse tópico
é o game_controller_node do hsl-player, a partir dos pacotes UDP do GameController
oficial (porta 3838).  O sim apenas consome esse tópico — ver
bridge/game_controller_link.py.
"""
from __future__ import annotations

import json
import math
import time as _time

import numpy as np
import rclpy
from rclpy.node import Node

from booster_msgs.msg import RpcReqMsg
from booster_interface.msg import LowState, Odometer, MotorState

# LowCmd só existe em alguns branches do hsl-player: competition-alan traz 16
# mensagens em booster_interface, sim_stable traz 4.  Ele serve ao caminho
# opcional /joint_ctrl (deploy → sim, alvos de junta), que não faz parte da
# cadeia do brain — então a ausência dele não pode derrubar o simulador.
try:
    from booster_interface.msg import LowCmd
except ImportError:
    LowCmd = None
from vision_interface.msg import Detections, DetectedObject

# Mesma história do LowCmd: vision_interface tem 10 mensagens em competition-alan
# e 2 em sim_stable.  O brain do sim_stable nem assina /booster_soccer/line_segments,
# então a ausência do tipo é informação, não erro.
try:
    from vision_interface.msg import LineSegments
except ImportError:
    LineSegments = None
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from hl_sim.agents.base import AgentInterface, ActionCmd, SensorState

# ── LocoApiTopicReq api_ids ────────────────────────────────────────────────────
KMOVE           = 2001   # {vx, vy, vyaw}
KROTATE_HEAD    = 2004   # {yaw, pitch}
KGET_UP         = 2008   # {} — get up from fallen
KSHOOT          = 2024   # {} — one-shot
KHIGH_KICK      = 100001 # {} — high kick
KROBOCUP_WALK   = 100008 # {} — enable robocup walk mode
KRL_KICK        = 100011 # {kick_speed, kick_dir, cancel}
KRL_FANCY_KICK  = 100012 # {kick_speed, kick_dir, cancel}

# ── índices das juntas no qpos/qvel (de sim_bridge.py) ────────────────────────
# qpos[0:7]   = ball free-joint (x y z qw qx qy qz)
# qpos[7:14]  = trunk free-joint (x y z qw qx qy qz)
# qpos[14:37] = 23 posições de junta (head_yaw, head_pitch, 21 body joints)
# qvel[0:6]   = ball dof
# qvel[6:12]  = trunk dof
# qvel[12:35] = 23 velocidades de junta
JOINT_QPOS_START      = 14
JOINT_QVEL_START      = 12
NUM_JOINTS            = 23
NUM_BODY_JOINTS       = 21   # body joints without head (2)
JOINT_QPOS_BODY_START = JOINT_QPOS_START + 2  # skip head_yaw, head_pitch
JOINT_QVEL_BODY_START = JOINT_QVEL_START + 2

# ── sensordata layout (soccer_scene.xml sensors section) ─────────────────────
# sensordata[0:4] = framequat orientation (w, x, y, z)
# sensordata[4:7] = gyro angular-velocity body frame (wx, wy, wz)
SENSOR_QUAT_IDX = 0
SENSOR_GYRO_IDX = 4


def _set_if(msg, field: str, value) -> bool:
    """
    Atribui `field` só se a mensagem daquele build o tiver.

    Os schemas de visão divergem entre branches do hsl-player: Detections perde
    radar_x/radar_y/corner_pos no sim_stable, e DetectedObject perde
    color/target_uv/position_cam/position_confidence.  Os de sensor (LowState,
    Odometer, MotorState) são idênticos.  Em vez de ramificar por versão, o sim
    preenche o que o schema oferece — o que vale nos dois branches.
    """
    if hasattr(msg, field):
        setattr(msg, field, value)
        return True
    return False


class ROS2Agent(AgentInterface):
    """
    Ponte entre SimBridge e hsl-player via ROS2.
    Um nó por robô; use player_id para diferenciar namespaces futuros.

    team_id/player_id precisam bater com game.team_id / game.player_id do
    config.yaml do brain e com o cadastro do time no GameController.

    Uso típico:
        rclpy.init()
        node  = Node('hl_sim_bridge')
        agent = ROS2Agent('T1_1', node, player_id=1)
        bridge = SimBridge(agents=[agent])
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        bridge.run(viewer=True)
    """

    def __init__(self, robot_name: str, node: Node, player_id: int = 1, team_id: int = 0) -> None:
        super().__init__(robot_name, team_id=team_id, player_id=player_id)
        self._node      = node
        self._cmd       = ActionCmd()
        self._kick_active = False
        self._kick_updated = 0.0
        self._kick_id = 0
        self._joint_pos = None  # 21 body joint targets from deploy (/joint_ctrl)

        # ── publishers (sim → brain/deploy) ───────────────────────────────────
        self._pub_low   = node.create_publisher(LowState,        '/low_state',                    10)
        self._pub_odom  = node.create_publisher(Odometer,        '/odometer_state',               10)
        self._pub_det   = node.create_publisher(Detections,      '/booster_soccer/detection',     10)
        self._pub_lines = (
            node.create_publisher(LineSegments, '/booster_soccer/line_segments', 10)
            if LineSegments is not None else None
        )
        self._pub_head  = node.create_publisher(Pose,            '/head_pose',                    10)

        # ── subscribers ────────────────────────────────────────────────────────
        node.create_subscription(RpcReqMsg, 'LocoApiTopicReq', self._on_loco,       10)
        node.create_subscription(Bool, 'kick_intent', self._on_kick_intent, 10)
        if LowCmd is not None:
            node.create_subscription(LowCmd, '/joint_ctrl', self._on_joint_ctrl, 10)
        else:
            node.get_logger().info(
                "booster_interface/LowCmd não existe neste build do hsl-player; "
                "/joint_ctrl desativado (só afeta o caminho do deploy, não o brain)."
            )

        node.get_logger().info(
            f"ROS2Agent '{robot_name}' (team_id={team_id}, player_id={player_id}) pronto."
        )

    # ── AgentInterface ─────────────────────────────────────────────────────────

    def reset(self) -> None:
        with self._lock:
            self._cmd       = ActionCmd()
            self._kick_active = False
            self._kick_updated = 0.0
            self._kick_id = 0
            self._joint_pos = None
            # O estado de jogo NÃO é resetado aqui: ele pertence ao GameController,
            # não ao episódio do simulador.

    def step(self, state: SensorState) -> ActionCmd:
        self._publish_sensors(state)
        with self._lock:
            cmd = ActionCmd(
                vx         = self._cmd.vx,
                vy         = self._cmd.vy,
                vyaw       = self._cmd.vyaw,
                head_yaw   = self._cmd.head_yaw,
                head_pitch = self._cmd.head_pitch,
                shoot      = self._cmd.shoot or (
                    self._kick_active and _time.monotonic() - self._kick_updated < 0.25
                ),
                kick_id    = self._kick_id,
                joint_pos  = self._joint_pos.copy() if self._joint_pos is not None else None,
            )
            self._cmd.shoot = False  # shoot é one-shot
        return cmd

    # ── callbacks (brain → sim) ────────────────────────────────────────────────

    def _on_kick_intent(self, msg: Bool) -> None:
        with self._lock:
            now = _time.monotonic()
            if msg.data and (not self._kick_active or now - self._kick_updated >= 0.25):
                self._kick_id += 1
            self._kick_active = msg.data
            self._kick_updated = now

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
            elif api_id in (KSHOOT, KHIGH_KICK, KRL_KICK, KRL_FANCY_KICK):
                self._cmd.shoot = True
            elif api_id == KGET_UP:
                pass  # kinematic sim never falls — no action needed
            elif api_id == KROBOCUP_WALK:
                pass  # walk is always active via /rl_move in sim

    def _on_joint_ctrl(self, msg) -> None:   # LowCmd, quando existe
        cmds = msg.motor_cmd
        n = min(len(cmds), NUM_BODY_JOINTS)
        pos = np.array([cmds[i].q for i in range(n)], dtype=np.float64)
        with self._lock:
            self._joint_pos = pos

    # ── publishers (sim → brain) ───────────────────────────────────────────────

    def _publish_sensors(self, state: SensorState) -> None:
        self._publish_low_state(state) # /low_state
        self._publish_odometer(state) # /odometer_state
        self._publish_detections(state) # /booster_soccer/detection
        # Tópico vazio: o brain de alguns branches espera vê-lo existir.
        if self._pub_lines is not None:
            self._pub_lines.publish(LineSegments())
        self._publish_head_pose(state) # /head_pose

    def _publish_low_state(self, state: SensorState) -> None:
        msg = LowState()

        # 21 body joints (skip head_yaw[0] and head_pitch[1])
        q  = state.qpos[JOINT_QPOS_BODY_START: JOINT_QPOS_BODY_START + NUM_BODY_JOINTS]
        dq = state.qvel[JOINT_QVEL_BODY_START: JOINT_QVEL_BODY_START + NUM_BODY_JOINTS]
        for i in range(NUM_BODY_JOINTS):
            motor = MotorState()
            motor.q  = float(q[i])
            motor.dq = float(dq[i])
            msg.motor_state_serial.append(motor)

        # IMU from MuJoCo sensors: framequat (w,x,y,z) + gyro body frame (wx,wy,wz)
        if state.sensordata is not None and len(state.sensordata) >= 7:
            qw = float(state.sensordata[SENSOR_QUAT_IDX])
            qx = float(state.sensordata[SENSOR_QUAT_IDX + 1])
            qy = float(state.sensordata[SENSOR_QUAT_IDX + 2])
            qz = float(state.sensordata[SENSOR_QUAT_IDX + 3])
            roll, pitch, yaw = _quat_to_rpy(qw, qx, qy, qz)
            msg.imu_state.rpy = [roll, pitch, yaw]
            msg.imu_state.gyro = [
                float(state.sensordata[SENSOR_GYRO_IDX]),
                float(state.sensordata[SENSOR_GYRO_IDX + 1]),
                float(state.sensordata[SENSOR_GYRO_IDX + 2]),
            ]

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
        pos = [float(ball_rel[0]), float(ball_rel[1]), float(ball_rel[2])]
        det.position_projection = pos
        det.position            = pos
        _set_if(det, "position_cam", pos)
        _set_if(det, "position_confidence", 100)
        _set_if(det, "color", "")

        # projeta posição 3D para pixel para o brain usar no CamTrackBall
        with self._lock:
            head_yaw   = self._cmd.head_yaw
            head_pitch = self._cmd.head_pitch
        u, v = _ball_to_pixel(ball_rel, head_yaw, head_pitch)
        half = 20
        det.xmin = int(u - half)
        det.xmax = int(u + half)
        det.ymin = int(v - half)
        det.ymax = int(v + half)

        _set_if(det, "target_uv", [float(u), float(v)])

        msg.detected_objects.append(det)

        # detectProcessVisionBox do brain lê corner_pos como 5 cantos × 2 coords
        # (10 floats): trapézio largo à frente, no frame do robô (x=frente, y=esquerda).
        _set_if(msg, "corner_pos", [
            8.0,  5.0,   # far-left
            8.0, -5.0,   # far-right
            1.0, -1.5,   # near-right
            1.0,  1.5,   # near-left
            0.0,  0.0,   # origem do robô (5º ponto)
        ])

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




# ── helpers de geometria ───────────────────────────────────────────────────────

def _quat_to_rpy(w: float, x: float, y: float, z: float):
    """Quaternion (w, x, y, z) → (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


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


def _ball_to_pixel(
    ball_robot: np.ndarray,
    head_yaw: float,
    head_pitch: float,
) -> tuple:
    """
    Projeta bola (frame robô) para pixel (u, v) considerando rotação da cabeça e câmera.

    Câmera intrínseca do brain config:
        fx=fy=260.66, cx=325.6, cy=182.0 (imagem 640×380)

    camToHead (do brain debug):
        [[ 0.015, -0.034,  0.999],
         [-1.000,  0.002,  0.015],
         [-0.003, -0.999, -0.034]]
    headToCam = camToHead^T
    """
    bx, by, bz = float(ball_robot[0]), float(ball_robot[1]), float(ball_robot[2])

    # 1. Robô → frame da cabeça: desfaz yaw (rotação em torno de z)
    cy, sy = math.cos(head_yaw), math.sin(head_yaw)
    bx_h =  bx * cy + by * sy
    by_h = -bx * sy + by * cy
    bz_h =  bz

    # 2. Desfaz pitch (rotação em torno do eixo y da cabeça; positivo = olhar para baixo)
    cp, sp = math.cos(head_pitch), math.sin(head_pitch)
    bx_h2 =  bx_h * cp - bz_h * sp
    by_h2 =  by_h
    bz_h2 =  bx_h * sp + bz_h * cp

    # 3. Frame da cabeça → frame da câmera: aplica headToCam = camToHead^T
    X_c =  0.015 * bx_h2 - 1.000 * by_h2 - 0.003 * bz_h2
    Y_c = -0.034 * bx_h2 + 0.002 * by_h2 - 0.999 * bz_h2
    Z_c =  0.999 * bx_h2 + 0.015 * by_h2 - 0.034 * bz_h2

    if Z_c <= 0.01:
        return 325.6, 182.0  # bola atrás da câmera — retorna centro

    # 4. Projeção perspectiva
    fx, fy, cx, cy_img = 260.66, 260.66, 325.6, 182.0
    u = fx * X_c / Z_c + cx
    v = fy * Y_c / Z_c + cy_img
    return u, v
