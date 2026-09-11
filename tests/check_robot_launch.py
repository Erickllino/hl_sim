"""Smoke test isolado do launch padrão: UDP v20 -> GameController ROS."""
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

import rclpy
from game_controller_interface.msg import GameControlData

rclpy.init()
node = rclpy.create_node('observe_robot_launch')
received = []
node.create_subscription(GameControlData, '/robocup/game_controller', received.append, 10)
header = struct.pack('<4s10Bhh', b'RGme', 20, 1, 3, 0, 0, 0, 3, 0, 1, 56, 600, 0)
teams = b''.join(struct.pack('<6B2H60s', n, 0, 0, 1, 0, 0, 0, 0, bytes(60)) for n in (56, 55))
with tempfile.TemporaryDirectory() as tmp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
    log = Path(tmp) / 'launch.log'
    with log.open('w') as output:
        process = subprocess.Popen(['/usr/local/bin/entrypoint.sh', 'ros2', 'launch',
                                    '/workspace/docker/launch/robot.launch.py'],
                                   stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=tmp)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and process.poll() is None:
                udp.sendto(header + teams, ('127.0.0.1', 3838))
                rclpy.spin_once(node, timeout_sec=.05)
                if received:
                    break
            assert process.poll() is None and received, log.read_text()[-6000:]
            assert received[-1].version == 20 and received[-1].state == 3
            assert [t.team_number for t in received[-1].teams] == [56, 55]
            assert 'process has died' not in log.read_text(), log.read_text()[-6000:]
            print('PASS: launch padrão iniciou e recebeu UDP v20 de localhost sem whitelist física.')
        finally:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
node.destroy_node()
rclpy.shutdown()
