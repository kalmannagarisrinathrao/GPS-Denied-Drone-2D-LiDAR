import math
import unittest
from types import SimpleNamespace

from lidar_processor.goto_node import (
    carrot, choose_direction, corridor_blocked, enu_to_ned,
    make_cloud, near_returns, ned_yaw_to_enu_quaternion, scan_to_enu,
    wrap_pi)

HALF_WIDTH = 0.536
LENGTH = 2.0
STEP = math.radians(10)
FOV = math.radians(100)


def make_scan(blocked=None, default=math.inf):
    """270 deg scan, 1 deg steps. blocked: {deg: range}."""
    ranges = [default] * 271
    for deg, r in (blocked or {}).items():
        ranges[deg + 135] = r
    return SimpleNamespace(
        ranges=ranges, angle_min=-math.radians(135),
        angle_increment=math.radians(1.0), range_min=0.1, range_max=30.0)


def pick(scan, target_deg=0.0, side_lock=0):
    return choose_direction(
        scan, math.radians(target_deg), HALF_WIDTH, LENGTH, STEP, FOV,
        side_lock)


class TestGeometry(unittest.TestCase):

    def test_carrot_limits_lead(self):
        n, e, dist = carrot(0.0, 0.0, 10.0, 0.0, 1.0)
        self.assertAlmostEqual(n, 1.0)
        self.assertAlmostEqual(e, 0.0)
        self.assertAlmostEqual(dist, 10.0)

    def test_carrot_stops_at_goal(self):
        n, e, dist = carrot(0.0, 0.0, 0.5, 0.5, 1.0)
        self.assertEqual((n, e), (0.5, 0.5))

    def test_wrap_pi(self):
        self.assertAlmostEqual(wrap_pi(3 * math.pi), math.pi, places=6)

    def test_enu_to_ned(self):
        self.assertEqual(enu_to_ned(3.0, 5.0), (5.0, 3.0))

    def test_facing_east_is_zero_enu_yaw(self):
        z, w = ned_yaw_to_enu_quaternion(math.pi / 2.0)
        self.assertAlmostEqual(z, 0.0)
        self.assertAlmostEqual(w, 1.0)


class TestCorridor(unittest.TestCase):

    def test_return_inside_corridor_blocks(self):
        returns = near_returns(make_scan({0: 1.5}), LENGTH)
        self.assertTrue(corridor_blocked(returns, 0.0, HALF_WIDTH, LENGTH))

    def test_return_beyond_length_ignored(self):
        returns = near_returns(make_scan({0: 3.0}), LENGTH)
        self.assertFalse(corridor_blocked(returns, 0.0, HALF_WIDTH, LENGTH))

    def test_return_outside_width_ignored(self):
        returns = near_returns(make_scan({60: 1.0}), LENGTH)
        self.assertFalse(corridor_blocked(returns, 0.0, HALF_WIDTH, LENGTH))

    def test_invalid_returns_ignored(self):
        scan = make_scan({0: math.nan})
        self.assertEqual(near_returns(scan, LENGTH), [])


class TestChooseDirection(unittest.TestCase):

    def test_clear_goes_straight_to_target(self):
        self.assertAlmostEqual(pick(make_scan(), 15.0), math.radians(15.0))

    def test_obstacle_ahead_picks_free_direction(self):
        scan = make_scan({d: 1.2 for d in range(-20, 21)})
        choice = pick(scan)
        self.assertIsNotNone(choice)
        returns = near_returns(scan, LENGTH)
        self.assertFalse(corridor_blocked(returns, choice, HALF_WIDTH, LENGTH))
        self.assertNotAlmostEqual(choice, 0.0)

    def test_picks_nearest_free_side(self):
        # obstacle ahead and to the left: going right is the short way round
        scan = make_scan({d: 1.2 for d in range(0, 21)})
        self.assertLess(pick(scan), 0.0)

    def test_wide_wall_needs_large_turn(self):
        scan = make_scan({d: 1.5 for d in range(-60, 61)})
        self.assertGreaterEqual(abs(math.degrees(pick(scan))), 80.0)
        self.assertLessEqual(abs(math.degrees(pick(scan))), 100.0)

    def test_side_lock_forces_that_side(self):
        scan = make_scan({d: 1.2 for d in range(-20, 21)})
        self.assertGreater(pick(scan, side_lock=1), 0.0)
        self.assertLess(pick(scan, side_lock=-1), 0.0)

    def test_side_lock_never_crosses_over(self):
        # left side fully blocked: locked-left has nothing, locked-right does
        blocked = {d: 1.0 for d in range(0, 101)}
        scan = make_scan(blocked)
        self.assertIsNone(pick(scan, side_lock=1))
        self.assertLess(pick(scan, side_lock=-1), 0.0)

    def test_nothing_free_returns_none(self):
        self.assertIsNone(pick(make_scan(default=1.0)))


class TestVisuals(unittest.TestCase):

    def test_ahead_when_facing_east(self):
        pts = scan_to_enu(make_scan({0: 2.0}), 1.0, 5.0, math.pi / 2, 12.0,
                          stride=1)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(pts[0][0], 3.0)
        self.assertAlmostEqual(pts[0][1], 5.0)

    def test_ahead_when_facing_north(self):
        pts = scan_to_enu(make_scan({0: 2.0}), 0.0, 0.0, 0.0, 12.0, stride=1)
        self.assertAlmostEqual(pts[0][0], 0.0)
        self.assertAlmostEqual(pts[0][1], 2.0)

    def test_left_of_east_is_north(self):
        pts = scan_to_enu(make_scan({90: 3.0}), 0.0, 0.0, math.pi / 2, 12.0,
                          stride=1)
        self.assertAlmostEqual(pts[0][0], 0.0, places=6)
        self.assertAlmostEqual(pts[0][1], 3.0)

    def test_far_and_invalid_returns_dropped(self):
        scan = make_scan({0: 20.0, 10: math.nan})
        self.assertEqual(scan_to_enu(scan, 0.0, 0.0, 0.0, 12.0, 1), [])



if __name__ == '__main__':
    unittest.main()
