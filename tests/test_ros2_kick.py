"""Execute na imagem sim com os ambientes ROS e hsl-player carregados."""
import time
import unittest

try:
    import rclpy
    from std_msgs.msg import Bool
    from hl_sim.ros2.agent import ROS2Agent
    from hl_sim.sim.bridge import SimBridge
    import mujoco
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'requer ROS2 e interfaces main26')
class ROSKickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_main26_kick')
        cls.agent = ROS2Agent('robot1', cls.node)
        cls.bridge = SimBridge([cls.agent])
        cls.publisher = cls.node.create_publisher(Bool, 'kick_intent', 10)
        deadline = time.monotonic() + 5
        while cls.publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=.02)
        assert cls.publisher.get_subscription_count() >= 1

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.bridge.reset()
        b = self.bridge
        adr = b.layout.ball_qpos_adr
        b.d.qpos[adr:adr + 3] = [b._kin[0].x + .3, b._kin[0].y, .11]
        mujoco.mj_forward(b.m, b.d)

    def publish(self, active):
        previous = self.agent._kick_updated
        self.publisher.publish(Bool(data=active))
        deadline = time.monotonic() + 2
        while self.agent._kick_updated == previous and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertNotEqual(self.agent._kick_updated, previous)

    def test_ros_intent_reaches_mujoco(self):
        self.publish(True)
        self.bridge._ctrl_tick()
        self.assertGreater(self.bridge.d.xfrc_applied[self.bridge.layout.ball_body_id, 0], 0)

    def test_timeout_cancel_and_new_activation(self):
        self.publish(True)
        first_id = self.agent._kick_id
        self.publish(True)
        self.assertEqual(self.agent._kick_id, first_id)
        self.publish(False)
        self.assertFalse(self.agent.step(self.bridge._sensor(0)).shoot)
        self.publish(True)
        self.assertGreater(self.agent._kick_id, first_id)
        time.sleep(.3)
        self.assertFalse(self.agent.step(self.bridge._sensor(0)).shoot)

    def test_new_vision_topics(self):
        topics = dict(self.node.get_topic_names_and_types())
        self.assertIn('/booster_soccer/detection', topics)
        self.assertIn('/booster_soccer/line_segments', topics)
        self.assertNotIn('/booster_vision/detection', topics)


if __name__ == '__main__':
    unittest.main()
