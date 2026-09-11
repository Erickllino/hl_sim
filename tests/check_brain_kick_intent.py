"""Smoke test do brain compilado: execute dentro da imagem robot."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from std_msgs.msg import Bool
from booster_msgs.msg import RpcReqMsg

rclpy.init()
node = rclpy.create_node('observe_brain_kick')
intents, moves = [], []
node.create_subscription(Bool, 'kick_intent', lambda msg: intents.append(msg.data), 100)
node.create_subscription(RpcReqMsg, 'LocoApiTopicReq', lambda msg: moves.append(json.loads(msg.header).get('api_id')), 100)
workspace = Path(os.environ['HSL_DIR'])
with tempfile.TemporaryDirectory() as tmp:
    tree = Path(tmp) / 'kick.xml'
    tree.write_text('''<root BTCPP_format="4"><BehaviorTree ID="MainTree">
    <Kick speed_limit="0.8" min_msec_kick="200"/>
    </BehaviorTree></root>''')
    log = Path(tmp) / 'brain.log'
    args = [str(workspace / 'install/brain/lib/brain/brain_node'), '--ros-args',
            '--params-file', str(workspace / 'install/brain/share/brain/config/config.yaml'),
            '-p', 'game.player_role:=striker', '-p', 'game.team_id:=56', '-p', 'game.player_id:=2',
            '-p', 'enable_com:=false', '-p', 'strategy.abort_kick_when_ball_moved:=false',
            '-p', 'obstacle_avoidance.avoid_during_kick:=false',
            '-p', f'tree_file_path:={tree}',
            '-p', f'vision_config_path:={workspace}/install/vision/share/vision/config/vision.yaml']
    with log.open('w') as output:
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, cwd=tmp)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.02)
                if True in intents and False in intents and 2001 in moves:
                    break
            assert process.poll() is None, log.read_text()[-6000:]
            assert True in intents and False in intents and 2001 in moves, (intents, moves, log.read_text()[-6000:])
            print('PASS: brain real publicou Kick ativo/inativo e movimento (2001), sem SDK interno.')
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
node.destroy_node()
rclpy.shutdown()
