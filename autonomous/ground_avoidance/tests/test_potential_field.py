"""合力法避障核心的无硬件测试。"""

import unittest

import numpy as np

from esp32_output import build_drive_command
from potential_field import PotentialFieldAvoider


def depth_frame(left: int = 3000, center: int = 3000, right: int = 3000) -> np.ndarray:
    """构造一张含三个扇区深度值的测试帧。"""

    frame = np.zeros((100, 120), dtype=np.uint16)
    frame[25:88, 4:40] = left
    frame[25:88, 40:80] = center
    frame[25:88, 80:116] = right
    return frame


class PotentialFieldTest(unittest.TestCase):
    """验证目标力、正面斥力和侧向绕行行为。"""

    def test_clear_path_moves_forward(self) -> None:
        result = PotentialFieldAvoider().compute(depth_frame())
        self.assertGreater(result.throttle, 0)
        self.assertAlmostEqual(result.steering, 0.0, places=3)
        self.assertFalse(result.obstacle_detected)

    def test_right_obstacle_turns_left(self) -> None:
        result = PotentialFieldAvoider().compute(depth_frame(right=500))
        self.assertGreater(result.throttle, 0)
        self.assertLess(result.steering, 0)
        self.assertTrue(result.obstacle_detected)
        left, right = [int(value) for value in build_drive_command(result.throttle, result.steering).split(",")[1:]]
        self.assertGreaterEqual(left, 0)
        self.assertGreaterEqual(right, 0)

    def test_center_obstacle_chooses_wider_left_side(self) -> None:
        result = PotentialFieldAvoider().compute(depth_frame(left=3000, center=500, right=1000))
        self.assertLess(result.steering, 0)
        self.assertEqual(result.throttle, 0.0)

    def test_algorithm_never_reverses_for_front_obstacle(self) -> None:
        result = PotentialFieldAvoider().compute(depth_frame(center=100))
        self.assertEqual(result.throttle, 0.0)
        self.assertGreaterEqual(result.throttle, 0.0)

    def test_invalid_depth_stops(self) -> None:
        result = PotentialFieldAvoider().compute(np.zeros((100, 120), dtype=np.uint16))
        self.assertEqual(result.throttle, 0.0)
        self.assertEqual(result.steering, 0.0)
        self.assertIn("没有有效", result.reason)

    def test_drive_command_is_limited(self) -> None:
        self.assertEqual(build_drive_command(0, 0), "S")
        self.assertEqual(build_drive_command(2, 0), "D,1000,1000")
        self.assertEqual(build_drive_command(0, 2), "D,1000,-1000")


if __name__ == "__main__":
    unittest.main()
