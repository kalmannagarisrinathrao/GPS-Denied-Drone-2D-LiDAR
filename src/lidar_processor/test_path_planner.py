import unittest

from lidar_processor.path_planner import PathPlanner


class TestPathPlanner(unittest.TestCase):

    def setUp(self):
        self.planner = PathPlanner()

    def make_sectors(self, distance=1.0):
        return {
            angle: distance
            for angle in range(-90, 91, 5)
        }

    def test_clear_forward_corridor(self):

        sectors = self.make_sectors(5.0)

        result = self.planner.plan(sectors)

        self.assertEqual(result['direction'], 0.0)
        self.assertEqual(
            result['decision'],
            'FORWARD CORRIDOR CLEAR'
        )

    def test_left_gap(self):

        sectors = self.make_sectors()

        for angle in range(30, 61, 5):
            sectors[angle] = 5.0

        result = self.planner.plan(sectors)

        self.assertGreater(result['direction'], 0)

    def test_right_gap(self):

        sectors = self.make_sectors()

        for angle in range(-60, -29, 5):
            sectors[angle] = 5.0

        result = self.planner.plan(sectors)

        self.assertLess(result['direction'], 0)

    def test_no_safe_path(self):

        sectors = self.make_sectors()

        result = self.planner.plan(sectors)

        self.assertIsNone(result['direction'])
        self.assertEqual(result['speed'], 0.0)

    def test_unknown_front(self):

        sectors = self.make_sectors(5.0)
        sectors[0] = None

        result = self.planner.plan(sectors)

        self.assertIsNone(result['direction'])
        self.assertEqual(result['speed'], 0.0)

    def test_narrow_gap_rejected(self):

        sectors = self.make_sectors()

        # Only a 10-degree opening
        for angle in range(-5, 6, 5):
            sectors[angle] = 1.0

        result = self.planner.plan(sectors)

        self.assertEqual(
            result['decision'],
            'STOP - NO SAFE PATH'
        )

    def test_unknown_sector_breaks_gap(self):

        sectors = self.make_sectors()

        for angle in range(30, 61, 5):
            sectors[angle] = 5.0

        sectors[45] = None

        result = self.planner.plan(sectors)

        self.assertEqual(
            result['decision'],
            'STOP - NO SAFE PATH'
        )

    # --------------------------------------------------
    # NEW TEST 8: Reject 20-degree gap
    # --------------------------------------------------

    def test_20_degree_gap_rejected(self):

        sectors = self.make_sectors(2.0)

        for angle in range(-10, 11, 5):
            sectors[angle] = 5.0

        gaps = self.planner.find_safe_gaps(sectors)

        self.assertEqual(len(gaps), 0)

        result = self.planner.plan(sectors)

        self.assertEqual(
            result['decision'],
            'STOP - NO SAFE PATH'
        )

        self.assertIsNone(result['direction'])
        self.assertEqual(result['speed'], 0.0)

    # --------------------------------------------------
    # NEW TEST 9: Accept 25-degree gap
    # --------------------------------------------------

    def test_25_degree_gap_accepted(self):

        sectors = self.make_sectors(2.0)

        for angle in range(-10, 16, 5):
            sectors[angle] = 5.0

        gaps = self.planner.find_safe_gaps(sectors)

        self.assertEqual(len(gaps), 1)

        result = self.planner.plan(sectors)

        self.assertEqual(
            result['decision'],
            'ALTERNATIVE SAFE GAP SELECTED'
        )

        self.assertEqual(result['direction'], 2.5)
        self.assertGreater(result['speed'], 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
