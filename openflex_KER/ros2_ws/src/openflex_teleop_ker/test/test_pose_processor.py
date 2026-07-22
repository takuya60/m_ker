import math
import unittest

from openflex_teleop_ker.pose_processor import KerPoseProcessor, map_range


class PoseProcessorTest(unittest.TestCase):
    def test_map_range_supports_descending_input(self):
        self.assertAlmostEqual(map_range(0.0, 0.0, -60.0, 0.0, 0.02), 0.0)
        self.assertAlmostEqual(map_range(-60.0, 0.0, -60.0, 0.0, 0.02), 0.02)

    def test_processor_orders_right_then_left_and_maps_grippers(self):
        angles = list(range(7)) + [-30.0] + list(range(10, 17)) + [30.0]
        pose = KerPoseProcessor().process(angles)
        expected_right = [math.radians(index) for index in range(7)]
        expected_left = [math.radians(index) for index in range(10, 17)]
        for actual, expected in zip(pose.right_target[:7], expected_right):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(pose.left_target[:7], expected_left):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(pose.right_target[7], 0.022)
        self.assertAlmostEqual(pose.left_target[7], 0.022)

    def test_gripper_zero_is_open_and_travel_endpoint_is_closed(self):
        open_pose = KerPoseProcessor().process([0.0] * 16)
        self.assertAlmostEqual(open_pose.right_target[7], 0.044)
        self.assertAlmostEqual(open_pose.left_target[7], 0.044)

        closed_angles = [0.0] * 16
        closed_angles[7] = -60.0
        closed_angles[15] = 60.0
        closed_pose = KerPoseProcessor().process(closed_angles)
        self.assertAlmostEqual(closed_pose.right_target[7], 0.0)
        self.assertAlmostEqual(closed_pose.left_target[7], 0.0)

    def test_processor_rejects_wrong_firmware_shape(self):
        with self.assertRaisesRegex(ValueError, '16 angles'):
            KerPoseProcessor().process([0.0] * 20)


if __name__ == '__main__':
    unittest.main()
