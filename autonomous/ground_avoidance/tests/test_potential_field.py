"""合力法避障核心的无硬件测试。"""

import unittest

import numpy as np

from esp32_output import build_drive_command
from potential_field import AvoidanceConfig, PotentialFieldAvoider


# 标定得到的地面模型参数（与现场实测一致），用于构造带地面平面的合成帧。
GROUND_HORIZON = 141.7
GROUND_K = 44.5

# 关闭地面剔除的配置：用于验证“理想扇区”下的力场和减速逻辑。
# 这些合成帧不含地面平面，因此显式关闭地面模型，专注于力场行为本身。
NO_GROUND = AvoidanceConfig(ground_horizon_row=None, ground_scale_m_row=None)


def depth_frame(left: int = 3000, center: int = 3000, right: int = 3000) -> np.ndarray:
    """构造一张含三个扇区深度值的测试帧（低分辨率，历史兼容）。"""

    frame = np.zeros((100, 120), dtype=np.uint16)
    frame[25:88, 4:40] = left
    frame[25:88, 40:80] = center
    frame[25:88, 80:116] = right
    return frame


def sector_frame(
    left: float = 6.0,
    center: float = 6.0,
    right: float = 6.0,
    width: int = 480,
    height: int = 270,
) -> np.ndarray:
    """构造只含扇区障碍、不含地面的深度帧，单位为米。

    传入 None 表示该扇区没有有效回波（全零），用于验证未知深度的行为。
    配合 NO_GROUND 使用；带地面的场景请使用 ground_frame。
    """

    frame = np.zeros((height, width), dtype=np.uint16)
    top, bottom = int(height * 0.25), int(height * 0.88)
    roi_left, roi_right = int(width * 0.06), int(width * 0.94)
    sector = (roi_right - roi_left) / 3.0
    for index, distance in enumerate((left, center, right)):
        if distance is None:
            continue
        start = roi_left + int(index * sector)
        end = roi_left + int((index + 1) * sector)
        frame[top:bottom, start:end] = int(distance * 1000)
    return frame


def ground_frame(
    left: float = 8.0,
    center: float = 8.0,
    right: float = 8.0,
    width: int = 480,
    height: int = 270,
    far: float = 8.0,
) -> np.ndarray:
    """构造“地面 + 远处背景 + 可选扇区障碍”的深度帧，单位为米。

    地平线以下按标定模型铺地面，其余为远处背景；扇区障碍取“地面与给定
    距离中更近者”，模拟立在地面上的障碍物。用于验证地面剔除逻辑。
    """

    rows = np.arange(height, dtype=np.float32).reshape(-1, 1)
    background = np.full((height, width), far, dtype=np.float32)
    with np.errstate(divide="ignore", invalid="ignore"):
        ground = np.where(
            rows > GROUND_HORIZON,
            GROUND_K / np.maximum(rows - GROUND_HORIZON, 1e-6),
            np.inf,
        )
    ground = np.repeat(ground, width, axis=1)
    # 地面比远处背景更近，因此地面区域覆盖背景。
    depth = np.where(np.isfinite(ground), np.minimum(background, ground), background)

    top, bottom = int(height * 0.25), int(height * 0.88)
    roi_left, roi_right = int(width * 0.06), int(width * 0.94)
    sector = (roi_right - roi_left) / 3.0
    for index, distance in enumerate((left, center, right)):
        start = roi_left + int(index * sector)
        end = roi_left + int((index + 1) * sector)
        depth[top:bottom, start:end] = np.minimum(depth[top:bottom, start:end], distance)
    return (depth * 1000).astype(np.uint16)


class PotentialFieldTest(unittest.TestCase):
    """验证目标力、正面斥力和侧向绕行行为。"""

    def test_clear_path_moves_forward(self) -> None:
        result = PotentialFieldAvoider(NO_GROUND).compute(depth_frame())
        self.assertGreater(result.throttle, 0)
        self.assertAlmostEqual(result.steering, 0.0, places=3)
        self.assertFalse(result.obstacle_detected)

    def test_right_obstacle_turns_left(self) -> None:
        result = PotentialFieldAvoider(NO_GROUND).compute(depth_frame(right=500))
        self.assertGreater(result.throttle, 0)
        self.assertLess(result.steering, 0)
        self.assertTrue(result.obstacle_detected)
        left, right = [int(value) for value in build_drive_command(result.throttle, result.steering).split(",")[1:]]
        self.assertGreaterEqual(left, 0)
        self.assertGreaterEqual(right, 0)

    def test_center_obstacle_chooses_wider_left_side(self) -> None:
        result = PotentialFieldAvoider(NO_GROUND).compute(depth_frame(left=3000, center=500, right=1000))
        self.assertLess(result.steering, 0)
        self.assertEqual(result.throttle, 0.0)

    def test_algorithm_never_reverses_for_front_obstacle(self) -> None:
        result = PotentialFieldAvoider(NO_GROUND).compute(depth_frame(center=100))
        self.assertEqual(result.throttle, 0.0)
        self.assertGreaterEqual(result.throttle, 0.0)

    def test_invalid_depth_stops(self) -> None:
        result = PotentialFieldAvoider(NO_GROUND).compute(np.zeros((100, 120), dtype=np.uint16))
        self.assertEqual(result.throttle, 0.0)
        self.assertEqual(result.steering, 0.0)
        self.assertIn("没有有效", result.reason)

    def test_drive_command_is_limited(self) -> None:
        self.assertEqual(build_drive_command(0, 0), "S")
        self.assertEqual(build_drive_command(2, 0), "D,1000,1000")
        self.assertEqual(build_drive_command(0, 2), "D,1000,-1000")

    # ---- 针对已发现缺陷的回归测试 ----

    def test_unknown_center_stops_instead_of_driving(self) -> None:
        """正前方深度无效时必须停车，不能满速盲开。"""

        result = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(6.0, None, 6.0))
        self.assertEqual(result.throttle, 0.0)
        self.assertEqual(result.steering, 0.0)
        self.assertTrue(result.obstacle_detected)
        self.assertIn("无效", result.reason)

    def test_far_readings_are_known_not_unknown(self) -> None:
        """普遍超量程（很远）应视为已知的远距离，而不是误判为未知。"""

        result = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(12.0, 12.0, 12.0))
        self.assertGreater(result.throttle, 0.0)
        self.assertIsNotNone(result.center_clearance_m)

    def test_sparse_depth_is_unknown(self) -> None:
        """扇区内只有极少有效像素时判定为未知，不能被少量散点洗成通畅。"""

        frame = sector_frame(6.0, None, 6.0)
        frame[67:237, 169:180] = 6000
        result = PotentialFieldAvoider(NO_GROUND).compute(frame)
        self.assertEqual(result.throttle, 0.0)
        self.assertIn("无效", result.reason)

    def test_unknown_sides_produce_no_lateral_force(self) -> None:
        """侧方未知时不产生转向力，也不应因此停车。"""

        result = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(None, 3.0, None))
        self.assertEqual(result.steering, 0.0)
        self.assertGreater(result.throttle, 0.0)

    def test_progressive_deceleration_is_monotonic(self) -> None:
        """前进速度应随正前方距离减小连续单调下降，而不是悬崖式急停。"""

        def throttle_at(distance: float) -> float:
            return PotentialFieldAvoider(NO_GROUND).compute(sector_frame(6.0, distance, 6.0)).throttle

        distances = (2.4, 1.8, 1.5, 1.2, 0.9, 0.7, 0.6, 0.5)
        speeds = [throttle_at(distance) for distance in distances]
        for closer, farther in zip(speeds[1:], speeds[:-1]):
            self.assertLessEqual(closer, farther + 1e-9)
        self.assertGreater(speeds[3], 0.0)
        self.assertLess(speeds[3], speeds[0])

    def test_close_obstacle_allows_sharper_steering(self) -> None:
        """正前畅通时，侧面障碍越近应允许越急的差速避让。"""

        far = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(6.0, 6.0, 1.0))
        near = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(6.0, 6.0, 0.4))
        self.assertLess(near.steering, 0.0)
        self.assertLess(near.steering, far.steering)

    def test_steering_smoothing_damps_direction_flips(self) -> None:
        """开启转向平滑后，左右交替障碍引起的摆动应明显减小。"""

        def swing(config: AvoidanceConfig) -> float:
            avoider = PotentialFieldAvoider(config)
            frames = [sector_frame(0.5, 6.0, 6.0), sector_frame(6.0, 6.0, 0.5)]
            sequence = [avoider.compute(frame).steering for frame in frames * 3]
            return max(abs(value) for value in sequence)

        smoothed = swing(NO_GROUND)
        raw = swing(AvoidanceConfig(ground_horizon_row=None, ground_scale_m_row=None, steering_smoothing=1.0))
        self.assertLess(smoothed, raw)

    def test_steering_never_reverses_inner_wheel_while_driving(self) -> None:
        """行驶中内侧车轮不得反转，否则会“边前进边原地甩头”而转过头错过开口。

        差速混合为 left=throttle+steering、right=throttle-steering，因此只要
        |steering| <= throttle，两个车轮就都 >= 0，车辆只弯行不甩头。
        """

        cases = [
            (6.0, 6.0, 0.25),   # 侧面极近，转向需求最大
            (6.0, 1.5, 0.25),
            (0.4, 1.2, 6.0),
            (6.0, 2.0, 0.8),
        ]
        for left, center, right in cases:
            result = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(left, center, right))
            if result.throttle > 0:
                self.assertLessEqual(
                    abs(result.steering), result.throttle + 1e-9,
                    msg=f"内轮反转：left={left} center={center} right={right} "
                        f"throttle={result.throttle} steering={result.steering}",
                )
                left_wheel, right_wheel = [
                    float(v) for v in build_drive_command(result.throttle, result.steering).split(",")[1:]
                ]
                self.assertGreaterEqual(left_wheel, 0)
                self.assertGreaterEqual(right_wheel, 0)

    def test_steering_stays_bounded_across_frames(self) -> None:
        """连续帧中油门骤降时，平滑后的转向仍须满足内轮不反转。

        这是真实数据里发现的问题：限制在平滑之前应用，油门下降后上限变小，
        平滑残留的旧转向会超过新上限，使内侧车轮出现轻微反转。
        """

        avoider = PotentialFieldAvoider(NO_GROUND)
        # 先建立较大的转向状态（正前畅通、侧面极近）。
        avoider.compute(sector_frame(6.0, 6.0, 0.4))
        # 随后正前方迅速变近，油门骤降，检查转向是否被重新限制。
        for center in (0.75, 0.80, 0.70, 1.00):
            result = avoider.compute(sector_frame(6.0, center, 0.4))
            if result.throttle > 0:
                self.assertLessEqual(
                    abs(result.steering), result.throttle + 1e-9,
                    msg=f"center={center} throttle={result.throttle} steering={result.steering}",
                )

    def test_front_blocked_allows_pivot_turn(self) -> None:
        """正前方被堵、已经停车时，才允许原地转向更宽的一侧（两轮反向）。"""

        result = PotentialFieldAvoider(NO_GROUND).compute(sector_frame(3.0, 0.5, 1.0))
        self.assertEqual(result.throttle, 0.0)
        self.assertLess(result.steering, 0.0)
        # 原地转向幅度受 pivot_steering_limit 限制，避免转太快冲过侧面开口。
        self.assertLessEqual(abs(result.steering), 0.60 + 1e-9)
        command = build_drive_command(result.throttle, result.steering)
        self.assertTrue(command.startswith("D,"), msg=command)
        left_wheel, right_wheel = [float(v) for v in command.split(",")[1:]]
        # 原地转向：两轮符号相反。
        self.assertLess(left_wheel * right_wheel, 0)

    def test_turn_side_switch_has_margin(self) -> None:
        """左右宽度接近时不应反向，明显更宽时才切换，避免开口处左右反复。"""

        avoider = PotentialFieldAvoider(NO_GROUND)
        first = avoider.compute(sector_frame(2.5, 0.5, 2.3))
        self.assertLess(first.steering, 0.0)
        # 另一侧只宽出 0.1 米，未超过切换余量，应保持原方向。
        near_tie = avoider.compute(sector_frame(2.3, 0.5, 2.4))
        self.assertLess(near_tie.steering, 0.0)
        # 另一侧明显更宽（1.0 米），此时允许反向。
        clearly_wider = avoider.compute(sector_frame(2.0, 0.5, 3.0))
        self.assertGreater(clearly_wider.steering, 0.0)

    # ---- 地面剔除回归测试 ----

    def test_ground_plane_is_not_treated_as_obstacle(self) -> None:
        """只看到地面时不能报障碍，否则车会误以为前方被堵而不敢走。

        这是实地测试中实际出现的问题：ROI 下沿包含近处地面（0.3~0.5 米），
        使三个扇区都被判为约 0.6 米，车辆持续原地打转。
        """

        result = PotentialFieldAvoider().compute(ground_frame())
        self.assertGreater(result.throttle, 0.0)
        self.assertFalse(result.obstacle_detected)
        self.assertGreater(result.center_clearance_m, 2.0)

    def test_ground_removal_still_detects_real_obstacle(self) -> None:
        """剔除地面后，立在地面上的真实障碍物仍然要被识别。"""

        result = PotentialFieldAvoider().compute(ground_frame(center=0.5))
        self.assertEqual(result.throttle, 0.0)
        self.assertTrue(result.obstacle_detected)
        self.assertLess(result.center_clearance_m, 1.0)

    def test_ground_removal_avoids_false_near_obstacle(self) -> None:
        """同一帧在开启/关闭地面剔除时应得到明显不同的距离估计。"""

        frame = ground_frame()
        with_ground = PotentialFieldAvoider().compute(frame)
        without_ground = PotentialFieldAvoider(NO_GROUND).compute(frame)
        # 关闭剔除时地面像素被当成近障碍，距离被严重低估。
        self.assertLess(without_ground.center_clearance_m, 2.0)
        self.assertGreater(with_ground.center_clearance_m, without_ground.center_clearance_m)
        self.assertFalse(with_ground.obstacle_detected)

    def test_ground_mask_disabled_returns_none(self) -> None:
        """未配置地面参数时应返回 None，表示关闭地面剔除。"""

        avoider = PotentialFieldAvoider(NO_GROUND)
        self.assertIsNone(avoider._ground_mask(ground_frame()))
        enabled = PotentialFieldAvoider()._ground_mask(ground_frame())
        self.assertIsNotNone(enabled)
        # 地面像素应被判为地面：图像底部（近处地面）几乎全是地面。
        self.assertGreater(float(np.mean(enabled[-20:])), 0.8)
        # 地平线以上不应被误判为地面。
        self.assertLess(float(np.mean(enabled[:100])), 0.2)


if __name__ == "__main__":
    unittest.main()
