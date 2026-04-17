# Próximos Passos — Integração ROS2 (Fase 3)

O simulador já tem a Fase 2 funcionando: MuJoCo + `StandaloneAgent` (agente
scripted sem ROS2). O próximo passo é criar o `ROS2Agent` para que o brain
do `hsl-player` (rodando no Docker) controle o robô simulado.

---

## Estado atual

```
[StandaloneAgent]  →  SimBridge  →  MuJoCo
```

## Meta da Fase 3

```
[hsl-player brain]  ←→  ROS2  ←→  ROS2Agent  →  SimBridge  →  MuJoCo
     (Docker)                       (host)
```

O brain não sabe que está falando com o simulador — recebe os mesmos tópicos
que receberia do robô real.

---

## Passo 1 — Verificar pré-requisitos

```bash
# No host (Ubuntu 24.04), ROS2 Jazzy ou no Docker (Ubuntu 22.04, Humble)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Verificar se os pacotes de mensagens estão disponíveis
ros2 interface show booster_interface/msg/Odometer
ros2 interface show booster_msgs/msg/RpcReqMsg
ros2 interface show vision_interface/msg/Detections
```

Se os pacotes não estiverem disponíveis no host, compile o workspace do
`hsl-player` e source:

```bash
cd ~/Documents/hl_unification/hsl-player
colcon build --packages-skip vision --symlink-install
source install/setup.bash
```

---

## Passo 2 — Criar `bridge/ros2_agent.py`

Crie o arquivo `bridge/ros2_agent.py` implementando `AgentInterface`.

### Tópicos que o ROS2Agent deve **publicar** (sim → brain)

| Tópico | Tipo | O que publicar |
|--------|------|----------------|
| `/low_state` | `booster_interface/msg/LowState` | Estado das juntas (`qpos[14:37]`, `qvel[12:35]`) |
| `/odometer_state` | `booster_interface/msg/Odometer` | Posição e orientação do trunk (`trunk_pos`, `trunk_quat`) |
| `/booster_vision/detection` | `vision_interface/msg/Detections` | Posição da bola, postes, marcadores — ground truth do MuJoCo |
| `/booster_vision/line_segments` | `vision_interface/msg/LineSegments` | Linhas do campo (pode começar vazio) |
| `/robocup/game_controller` | `game_controller_interface/msg/GameControlData` | Estado do jogo (INITIAL → READY → SET → PLAYING) |
| `/head_pose` | `geometry_msgs/msg/Pose` | Pose da cabeça (pode ser identidade no início) |

### Tópicos que o ROS2Agent deve **assinar** (brain → sim)

| Tópico | Tipo | O que fazer |
|--------|------|-------------|
| `LocoApiTopicReq` | `booster_msgs/msg/RpcReqMsg` | `api_id=2001` (kMove): atualizar `ActionCmd.vx/vy/vyaw` |
| `LocoApiTopicReq` | `booster_msgs/msg/RpcReqMsg` | `api_id=2004` (kRotateHead): atualizar `head_yaw/pitch` |
| `LocoApiTopicReq` | `booster_msgs/msg/RpcReqMsg` | `api_id=2024` (kShoot): setar `ActionCmd.shoot=True` |
| `/rl_move` | `geometry_msgs/msg/Twist` | Alternativa ao kMove via deploy RL |

### Skeleton base

```python
# bridge/ros2_agent.py
from __future__ import annotations
import json
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node

from booster_msgs.msg import RpcReqMsg
from booster_interface.msg import LowState, Odometer
from vision_interface.msg import Detections, Detection, LineSegments
from game_controller_interface.msg import GameControlData
from geometry_msgs.msg import Pose, Twist

from bridge.agent_interface import AgentInterface, ActionCmd, SensorState

# api_ids do LocoApiTopicReq
KMOVE        = 2001
KROTATE_HEAD = 2004
KSHOOT       = 2024


class ROS2Agent(AgentInterface):
    """
    Ponte entre SimBridge e hsl-player via ROS2.
    Um nó por robô; use player_id para diferenciar namespaces.
    """

    CTRL_HZ = 50  # frequência de publicação dos sensores

    def __init__(self, robot_name: str, node: Node, player_id: int = 1) -> None:
        super().__init__(robot_name)
        self._node = node
        self._cmd  = ActionCmd()
        self._lock = threading.Lock()

        # publishers (sim → brain)
        self._pub_low   = node.create_publisher(LowState,       '/low_state',       10)
        self._pub_odom  = node.create_publisher(Odometer,       '/odometer_state',  10)
        self._pub_det   = node.create_publisher(Detections,     '/booster_vision/detection', 10)
        self._pub_lines = node.create_publisher(LineSegments,   '/booster_vision/line_segments', 10)
        self._pub_gc    = node.create_publisher(GameControlData,'/robocup/game_controller', 10)
        self._pub_head  = node.create_publisher(Pose,           '/head_pose',        10)

        # subscribers (brain → sim)
        node.create_subscription(RpcReqMsg, 'LocoApiTopicReq', self._on_loco, 10)
        node.create_subscription(Twist,     '/rl_move',         self._on_rl_move, 10)

    def reset(self) -> None:
        with self._lock:
            self._cmd = ActionCmd()

    def step(self, state: SensorState) -> ActionCmd:
        self._publish_sensors(state)
        with self._lock:
            cmd = ActionCmd(
                vx=self._cmd.vx, vy=self._cmd.vy, vyaw=self._cmd.vyaw,
                head_yaw=self._cmd.head_yaw, head_pitch=self._cmd.head_pitch,
                shoot=self._cmd.shoot,
            )
            self._cmd.shoot = False  # shoot é one-shot
        return cmd

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_loco(self, msg: RpcReqMsg) -> None:
        body = json.loads(msg.data) if msg.data else {}
        with self._lock:
            if msg.api_id == KMOVE:
                self._cmd.vx   = float(body.get('vx',   0.0))
                self._cmd.vy   = float(body.get('vy',   0.0))
                self._cmd.vyaw = float(body.get('vyaw', 0.0))
            elif msg.api_id == KROTATE_HEAD:
                self._cmd.head_yaw   = float(body.get('yaw',   0.0))
                self._cmd.head_pitch = float(body.get('pitch', 0.0))
            elif msg.api_id == KSHOOT:
                self._cmd.shoot = True

    def _on_rl_move(self, msg: Twist) -> None:
        with self._lock:
            self._cmd.vx   = msg.linear.x
            self._cmd.vy   = msg.linear.y
            self._cmd.vyaw = msg.angular.z

    # ── publishers ─────────────────────────────────────────────────────────────

    def _publish_sensors(self, state: SensorState) -> None:
        self._publish_odometer(state)
        self._publish_detections(state)
        self._pub_lines.publish(LineSegments())
        self._pub_head.publish(Pose())

    def _publish_odometer(self, state: SensorState) -> None:
        msg = Odometer()
        msg.pose.position.x = float(state.trunk_pos[0])
        msg.pose.position.y = float(state.trunk_pos[1])
        msg.pose.position.z = float(state.trunk_pos[2])
        q = state.trunk_quat  # w x y z (MuJoCo)
        msg.pose.orientation.w = float(q[0])
        msg.pose.orientation.x = float(q[1])
        msg.pose.orientation.y = float(q[2])
        msg.pose.orientation.z = float(q[3])
        self._pub_odom.publish(msg)

    def _publish_detections(self, state: SensorState) -> None:
        # TODO: converter posições 3D do MuJoCo em detecções 2D de câmera
        # Por enquanto publica a posição da bola em coordenadas do campo
        msg = Detections()
        # ... preencher com ball_pos, goalpost positions, etc.
        self._pub_det.publish(msg)
```

---

## Passo 3 — Criar `scripts/run_ros2.py`

```python
#!/usr/bin/env python3
# scripts/run_ros2.py — Fase 3: SimBridge + ROS2Agent
import argparse
import threading

import rclpy
from rclpy.node import Node

from bridge.sim_bridge import SimBridge
from bridge.ros2_agent import ROS2Agent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--viewer',   action='store_true')
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--speed',    type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('hl_sim_bridge')

    agent  = ROS2Agent('T1_1', node, player_id=1)
    bridge = SimBridge(agents=[agent])

    # ROS2 spin em background para callbacks funcionarem enquanto sim roda
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    try:
        bridge.run(duration=args.duration, viewer=args.viewer, speed=args.speed)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Adicione ao `pyproject.toml`:
```toml
[project.scripts]
sim-ros2 = "scripts.run_ros2:main"
```

---

## Passo 4 — Testar integração básica

**Terminal 1 — Simulador:**
```bash
cd ~/Documents/hl_sim
source ~/Documents/hl_unification/hsl-player/install/setup.bash
uv run python scripts/run_ros2.py --viewer
```

**Terminal 2 — Brain (Docker):**
```bash
docker compose -f ~/Documents/hl_unification/docker/docker-compose.yml run --rm dev
# dentro do container:
source /workspace/hsl-player/install/setup.bash
ros2 launch brain launch.py tree:=game.xml sim:=true disable_log:=true
```

**Terminal 3 — Verificar tópicos:**
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic hz /low_state        # deve mostrar ~50 Hz
ros2 topic hz /odometer_state   # deve mostrar ~50 Hz
```

---

## Passo 5 — Game Controller mock

O brain fica parado em `INITIAL` sem o game controller. Publique o estado
`PLAYING` para ele começar a agir:

```python
# Adicionar ao ROS2Agent.__init__:
from game_controller_interface.msg import GameControlData
import threading

self._pub_gc = node.create_publisher(GameControlData, '/robocup/game_controller', 1)
node.create_timer(1.0, self._publish_game_state)
self._game_started = False

def _publish_game_state(self):
    msg = GameControlData()
    msg.game_state = 3  # PLAYING
    msg.secondary_state = 0
    self._pub_gc.publish(msg)
```

---

## Passo 6 — Detecções de visão (ground truth)

A parte mais importante para o brain reagir ao ambiente. O brain assina
`/booster_vision/detection` esperando objetos detectados. No sim, calculamos
isso diretamente do MuJoCo:

```python
def _publish_detections(self, state: SensorState) -> None:
    msg = Detections()

    # bola — posição relativa ao robô no referencial da câmera
    ball_world = state.ball_pos
    robot_pos  = state.trunk_pos
    # TODO: transformar ball_world → frame câmera usando trunk_quat + extrínsecos
    # Referência: vision.yaml → camera.extrin (matriz 4x4 câmera→cabeça)

    self._pub_det.publish(msg)
```

Consulte `vision_interface/msg/Detections.msg` para o formato exato:
```bash
ros2 interface show vision_interface/msg/Detections
ros2 interface show vision_interface/msg/Detection
```

---

## Checklist Fase 3

- [ ] `bridge/ros2_agent.py` criado
- [ ] `scripts/run_ros2.py` criado
- [ ] Sim publica `/low_state` a 50 Hz — brain não crasha
- [ ] Sim publica `/odometer_state` — brain tem posição do robô
- [ ] Brain recebe `/robocup/game_controller` estado PLAYING
- [ ] Brain envia `LocoApiTopicReq` e robô se move no sim
- [ ] Sim publica `/booster_vision/detection` com posição da bola
- [ ] Brain persegue bola no sim

## Fase 4 — Multi-robô

Ver `ROADMAP_ROS2.md` §Multi-robot para adicionar múltiplos corpos no
`soccer_scene.xml` e rodar N brains em paralelo.
