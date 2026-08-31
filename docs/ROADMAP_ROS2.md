# ROADMAP — ROS2 Integration (Phase 3+)

Steps to run `hl_sim` with the real `hsl-player` brain on a machine that has ROS2 installed.

---

## Prerequisites

| Item | Command / location |
|------|--------------------|
| ROS2 Humble (or Iron) | `source /opt/ros/humble/setup.bash` |
| `hl_sim` cloned | `~/Documents/hl_sim` |
| `hl_unification` cloned | `~/Documents/hl_unification` |
| `booster_msgs` / `booster_interface` built | `colcon build` inside your workspace |
| Python 3.10 venv active | `uv sync` inside `hl_sim/` |

---

## Step 1 — Install `hl_sim` into the ROS2 environment

```bash
# inside hl_sim/
uv sync
# ROS2 Python overlays may require the package to be pip-installed:
pip install -e .
```

---

## Step 2 — Implement `ROS2Agent`

Create `bridge/ros2_agent.py` implementing `AgentInterface`.

### Role
- **Receives** velocity commands from `hsl-player` via the `LocoApiTopicReq` topic.
- **Publishes** sensor state (`/low_state`, `/odometer_state`) so `hsl-player` can run.
- **Publishes** vision ground truth (`/booster_vision/ball`, `/booster_vision/detection`).

### Skeleton

```python
# bridge/ros2_agent.py
from __future__ import annotations
import rclpy
from rclpy.node import Node
from bridge.agent_interface import AgentInterface, SensorState, ActionCmd

# Message types (adjust to your workspace)
from booster_msgs.msg import RpcReqMsg          # LocoApiTopicReq
from booster_interface.msg import LowState, OdometerState

LOCO_TOPIC  = "LocoApiTopicReq"
LOW_STATE   = "/low_state"
ODOM_STATE  = "/odometer_state"
BALL_TOPIC  = "/booster_vision/ball"

class ROS2Agent(AgentInterface):
    """
    Bridges SimBridge ↔ hsl-player over ROS2.
    One node per robot; use robot_name as ROS2 namespace.
    """

    def __init__(self, robot_name: str, node: Node) -> None:
        super().__init__(robot_name)
        ns = f"/{robot_name}"
        self._node = node
        self._cmd  = ActionCmd()

        # publisher: sensor → hsl-player
        self._pub_low   = node.create_publisher(LowState,       ns + LOW_STATE,  10)
        self._pub_odom  = node.create_publisher(OdometerState,  ns + ODOM_STATE, 10)
        # subscriber: hsl-player → sim
        node.create_subscription(RpcReqMsg, ns + LOCO_TOPIC,
                                 self._on_loco_cmd, 10)

    def reset(self) -> None:
        self._cmd = ActionCmd()

    def step(self, state: SensorState) -> ActionCmd:
        self._publish_low_state(state)
        self._publish_odom(state)
        return self._cmd          # last command from hsl-player

    def _on_loco_cmd(self, msg: RpcReqMsg) -> None:
        import json
        body = json.loads(msg.data)
        api_id = msg.api_id

        if api_id == 2001:   # kMove
            self._cmd.vx   = float(body.get("vx",   0.0))
            self._cmd.vy   = float(body.get("vy",   0.0))
            self._cmd.vyaw = float(body.get("vyaw", 0.0))

        elif api_id == 2004: # kRotateHead
            self._cmd.head_yaw   = float(body.get("yaw",   0.0))
            self._cmd.head_pitch = float(body.get("pitch", 0.0))

        elif api_id == 2024: # kShoot
            self._cmd.shoot = True

    def _publish_low_state(self, state: SensorState) -> None:
        msg = LowState()
        # map qpos[14:37] → msg.joint_state.q  (23 joints)
        # map qvel[12:35] → msg.joint_state.dq (23 dofs)
        # TODO: fill according to booster_interface/LowState definition
        self._pub_low.publish(msg)

    def _publish_odom(self, state: SensorState) -> None:
        msg = OdometerState()
        msg.position.x = float(state.trunk_pos[0])
        msg.position.y = float(state.trunk_pos[1])
        msg.position.z = float(state.trunk_pos[2])
        # TODO: fill quaternion from state.trunk_quat
        self._pub_odom.publish(msg)
```

---

## Step 3 — Wire `ROS2Agent` into `SimBridge`

```python
# run_ros2.py  (create at repo root)
import rclpy
from rclpy.node import Node
from bridge.sim_bridge   import SimBridge
from bridge.ros2_agent   import ROS2Agent

def main():
    rclpy.init()
    node = Node("hl_sim_bridge")

    agents = [ROS2Agent("T1_1", node)]
    bridge = SimBridge(agents=agents)

    import threading
    # spin ROS2 in background so callbacks fire while sim runs
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    bridge.run(duration=0, viewer=False, speed=1.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## Step 4 — Multi-robot (3v3): namespace layout

Each robot needs:
1. **A unique ROS2 namespace** (`/T1_1`, `/T1_2`, … `/T1_6`).
2. **A separate `hsl-player` process** launched with that namespace.
3. **A dedicated body in `soccer_scene.xml`** (duplicate the `<body name="Trunk" …>` block, rename to `Trunk_2`, etc., and update free-joint names).

Launch pattern:
```bash
# terminal A — hsl-player robot 1
ros2 run hsl_player hsl_player_node --ros-args -r __ns:=/T1_1

# terminal B — hsl-player robot 2
ros2 run hsl_player hsl_player_node --ros-args -r __ns:=/T1_2

# terminal C — SimBridge with 6 agents
uv run python run_ros2.py --robots T1_1 T1_2 T1_3 T1_4 T1_5 T1_6
```

No Docker or VMs needed — all processes share the same DDS domain on one machine.

---

## Step 5 — Game Controller mock

Publish `/robocup/game_controller` to drive the
`INITIAL → READY → SET → PLAYING` state machine that `hsl-player` expects:

```python
from booster_msgs.msg import GameControllerState   # adjust type

pub = node.create_publisher(GameControllerState, "/robocup/game_controller", 1)
# timer: publish PLAYING after 3 s
```

---

## Step 6 — Vision mock

Publish ground-truth positions from `MjData.xpos`:

| Topic | Type | Source |
|-------|------|--------|
| `/booster_vision/ball` | geometry_msgs/PointStamped | `d.xpos[BALL_BODY_ID]` |
| `/booster_vision/detection` | custom | all robot bodies |

---

## Step 7 — Locomotion policy (future)

When ready to use the RL policy from `booster_deploy`:

```
booster_deploy/tasks/locomotion/locomotion.py   ← LocomotionPolicy
booster_deploy/tasks/locomotion/models/t1_walk.pt
```

Replace the kinematic base control in `SimBridge._apply()` with:
1. Load `LocomotionPolicy` with `T1WalkControllerCfg`.
2. Each ctrl tick: call `policy.step(obs)` → 23 joint targets.
3. Write targets to `self.d.ctrl[:]` instead of `HOME_CTRL`.
4. Remove the free-joint kinematic override — let physics handle the base.

---

## Checklist

- [ ] ROS2 environment sourced, workspace built
- [ ] `hl_sim` pip-installed in ROS2 Python
- [ ] `ROS2Agent` — LowState / OdometerState messages filled correctly
- [ ] `hsl-player` launches and receives `/low_state` without errors
- [ ] Single robot walks toward ball in sim
- [ ] Multiply robots in scene XML
- [ ] 3v3 match runs end-to-end
- [ ] Game controller state machine wired
- [ ] Vision topics → `hsl-player` uses ball detection
