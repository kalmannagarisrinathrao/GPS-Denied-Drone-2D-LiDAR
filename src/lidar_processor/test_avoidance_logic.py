import unittest

from lidar_processor.avoidance_controller import AvoidanceController


class TestAvoidanceLogic(unittest.TestCase):

    def setUp(self):
        self.controller = AvoidanceController.__new__(
            AvoidanceController
        )

        self.controller.danger_distance = 2.5
        self.controller.safe_distance = 3.0

    def test_clear_path(self):
        result = self.controller.make_decision(5.0, 4.0, 4.0)
        self.assertEqual(result, "MOVE FORWARD - FRONT PATH CLEAR")

    def test_left_blocked(self):
        result = self.controller.make_decision(2.0, 1.0, 4.0)
        self.assertEqual(result, "AVOID RIGHT")

    def test_right_blocked(self):
        result = self.controller.make_decision(2.0, 4.0, 1.0)
        self.assertEqual(result, "AVOID LEFT")

    def test_both_sides_blocked(self):
        result = self.controller.make_decision(2.0, 1.0, 2.0)
        self.assertEqual(
            result,
            "STOP - NO SAFE TURNING DIRECTION"
        )

    def test_front_caution(self):
        result = self.controller.make_decision(2.7, 4.0, 4.0)
        self.assertEqual(result, "CAUTION - FRONT OBSTACLE NEAR")

    def test_unknown_front(self):
        result = self.controller.make_decision(None, 4.0, 4.0)
        self.assertEqual(result, "STOP - FRONT SENSOR DATA UNKNOWN")


if __name__ == "__main__":
    unittest.main(verbosity=2)
