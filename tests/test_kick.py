import math
import unittest

import mujoco
import numpy as np

from hl_sim.agents.base import ActionCmd, AgentInterface
from hl_sim.sim.bridge import SimBridge
from hl_sim.sim.kick import KickConfig


class FixedAgent(AgentInterface):
    def reset(self):
        self.cmd = ActionCmd()

    def step(self, state):
        return self.cmd


class KickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.agent = FixedAgent()
        cls.bridge = SimBridge([cls.agent])

    def setUp(self):
        self.bridge.reset()
        self.b = self.bridge
        self.ball = self.b.layout.ball_body_id

    def place_ball(self, dx, dy=0, height=0.11):
        adr = self.b.layout.ball_qpos_adr
        ks = self.b._kin[0]
        self.b.d.qpos[adr:adr + 3] = [ks.x + dx, ks.y + dy, height]
        mujoco.mj_forward(self.b.m, self.b.d)

    def apply(self, shoot=True, **kwargs):
        self.b.d.xfrc_applied[self.ball] = 0
        self.b._apply(0, ActionCmd(shoot=shoot, **kwargs))
        return self.b.d.xfrc_applied[self.ball, :2].copy()

    def test_requires_intent_range_front_and_low_ball(self):
        for dx, dy, height, shoot in [(0.3, 0, .11, False), (1, 0, .11, True),
                                      (-.3, 0, .11, True), (0, .3, .11, True),
                                      (.3, 0, .6, True)]:
            with self.subTest(dx=dx, dy=dy, height=height, shoot=shoot):
                self.b.reset()
                self.place_ball(dx, dy, height)
                np.testing.assert_array_equal(self.apply(shoot), [0, 0])

    def test_one_impulse_per_intent_and_can_rearm(self):
        self.place_ball(.3)
        forces = [self.apply() for _ in range(12)]
        np.testing.assert_allclose(forces[:5], [[10, 0]] * 5)
        np.testing.assert_array_equal(forces[5:], np.zeros((7, 2)))
        self.apply(False)
        np.testing.assert_allclose(self.apply(), [10, 0])

    def test_new_id_rearms_even_if_false_was_between_ticks(self):
        self.place_ball(.3)
        for _ in range(6):
            self.apply(kick_id=1)
        np.testing.assert_allclose(self.apply(kick_id=2), [10, 0])

    def test_custom_force_and_duration(self):
        old = self.b.kick
        try:
            self.b.kick = KickConfig(force_newtons=7, duration_seconds=.04)
            self.place_ball(.3)
            np.testing.assert_allclose(self.apply(), [7, 0])
            np.testing.assert_allclose(self.apply(), [7, 0])
            np.testing.assert_array_equal(self.apply(), [0, 0])
        finally:
            self.b.kick = old

    def test_intent_can_wait_until_in_range(self):
        self.place_ball(1)
        np.testing.assert_array_equal(self.apply(), [0, 0])
        self.place_ball(.3)
        np.testing.assert_allclose(self.apply(), [10, 0])

    def test_force_direction_stays_fixed_during_pulse(self):
        self.b._kin[0].yaw = math.pi / 2
        self.place_ball(0, .3)
        np.testing.assert_allclose(self.apply(), [0, 10], atol=1e-12)
        self.b._kin[0].yaw = 0
        np.testing.assert_allclose(self.apply(), [0, 10], atol=1e-12)

    def test_real_physics_moves_ball_without_robot_contact(self):
        self.place_ball(.3)
        initial = self.b.d.xpos[self.ball].copy()
        self.agent.cmd = ActionCmd(shoot=True)
        for _ in range(15):
            self.b._ctrl_tick()
        self.assertGreater(self.b.d.xpos[self.ball, 0], initial[0] + .1)
        self.assertGreater(self.b.d.xpos[self.ball, 2], .07)

    def test_contact_masks_exclude_all_robot_ball_pairs(self):
        m = self.b.m
        ball_geoms = np.where(m.geom_bodyid == self.ball)[0]
        roots = {r.trunk_body_id for r in self.b.layout.robots}
        floor_contact = False
        for g in range(m.ngeom):
            body = int(m.geom_bodyid[g])
            while body and body not in roots:
                body = int(m.body_parentid[body])
            for bg in ball_geoms:
                allowed = ((m.geom_contype[g] & m.geom_conaffinity[bg]) or
                           (m.geom_contype[bg] & m.geom_conaffinity[g]))
                if body in roots:
                    self.assertFalse(allowed)
                elif m.geom_bodyid[g] == 0 and allowed:
                    floor_contact = True
        self.assertTrue(floor_contact)

    def test_invalid_config(self):
        for value in [-1, 0, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                KickConfig(force_newtons=value)


if __name__ == '__main__':
    unittest.main()
