"""地面车二维合力法避障核心。

本文件只接收深度矩阵并返回归一化差速控制量，不依赖串口、RealSense SDK、
Flask 或其他控制模块，因此可以在没有任何硬件的环境中单独测试和调参。
坐标约定：前方为 +X，右方为 +Y，最终 steering 的正方向表示向右转。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, radians, sin
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class AvoidanceConfig:
    """合力法参数；所有距离单位均为米。"""

    # 只使用图像中部偏下的区域，减少天花板、远处背景和无关区域干扰。
    roi_top_ratio: float = 0.25
    roi_bottom_ratio: float = 0.88
    roi_left_ratio: float = 0.06
    roi_right_ratio: float = 0.94

    # 以左、中、右三个扇区估计前方可通行距离。
    sector_angle_deg: float = 28.0
    clearance_percentile: float = 20.0
    invalid_depth_m: float = 8.0
    obstacle_distance_m: float = 0.60
    influence_distance_m: float = 2.2

    # 合力参数：目标力推动车辆向前，斥力把车辆推离障碍物。
    # 没有外部目标点时，把相机正前方定义为默认目标方向。
    goal_gain: float = 1.0
    repulsion_gain: float = 0.95
    center_turn_gain: float = 1.2
    steering_gain: float = 1.0
    max_throttle: float = 0.75
    default_forward_throttle: float = 0.65
    max_steering: float = 1.0
    # 前方畅通时限制转向相对推力的比例，避免差速混合后某个车轮反转。
    forward_steering_ratio: float = 0.35
    steering_smoothing: float = 1.0


@dataclass(frozen=True)
class AvoidanceOutput:
    """一次深度帧计算出的控制结果和诊断信息。"""

    throttle: float
    steering: float
    left_clearance_m: Optional[float]
    center_clearance_m: Optional[float]
    right_clearance_m: Optional[float]
    obstacle_detected: bool
    reason: str


class PotentialFieldAvoider:
    """使用前进目标力和障碍物斥力计算地面车差速输入。"""

    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.config = config or AvoidanceConfig()
        self._steering_state = 0.0

    def _sector_clearance(self, roi: np.ndarray, left: int, right: int) -> Optional[float]:
        """返回一个扇区的稳健近距离估计；没有有效深度时返回 None。"""

        values = roi[:, left:right].astype(np.float32) / 1000.0
        valid = values[np.isfinite(values) & (values > 0.08) & (values < self.config.invalid_depth_m)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, self.config.clearance_percentile))

    def _clearances(self, depth_mm: np.ndarray) -> tuple[Optional[float], Optional[float], Optional[float]]:
        """裁剪感兴趣区域并估计左、中、右三块区域的距离。"""

        if depth_mm.ndim != 2 or depth_mm.size == 0:
            return None, None, None
        height, width = depth_mm.shape
        top = max(0, min(height - 1, int(height * self.config.roi_top_ratio)))
        bottom = max(top + 1, min(height, int(height * self.config.roi_bottom_ratio)))
        roi = depth_mm[top:bottom]
        roi_left = max(0, min(width - 1, int(width * self.config.roi_left_ratio)))
        roi_right = max(roi_left + 1, min(width, int(width * self.config.roi_right_ratio)))
        roi = roi[:, roi_left:roi_right]

        sector_width = roi.shape[1] / 3.0
        distances = []
        for index in range(3):
            start = int(index * sector_width)
            end = int((index + 1) * sector_width)
            distances.append(self._sector_clearance(roi, start, max(start + 1, end)))
        return distances[0], distances[1], distances[2]

    def _repulsive_force(self, distance: Optional[float], angle_deg: float) -> tuple[float, float]:
        """计算单个扇区的斥力，返回 (前后方向, 左右方向)。"""

        if distance is None or distance >= self.config.influence_distance_m:
            return 0.0, 0.0
        distance = max(distance, 0.12)
        magnitude = self.config.repulsion_gain * (
            1.0 / distance - 1.0 / self.config.influence_distance_m
        ) / (distance * distance)
        angle = radians(angle_deg)
        # 障碍物位于右侧时，斥力的横向分量为负，车辆向左转。
        return -magnitude * cos(angle), -magnitude * sin(angle)

    def compute(self, depth_mm: np.ndarray) -> AvoidanceOutput:
        """根据一帧毫米深度图计算归一化 throttle/steering。"""

        left, center, right = self._clearances(depth_mm)
        if left is None and center is None and right is None:
            self._steering_state = 0.0
            return AvoidanceOutput(0.0, 0.0, left, center, right, True, "没有有效深度数据，停车")

        total_forward = self.config.goal_gain
        total_lateral = 0.0
        sector_angle = self.config.sector_angle_deg
        for index, (distance, angle) in enumerate(((left, -sector_angle), (center, 0.0), (right, sector_angle))):
            force_forward, force_lateral = self._repulsive_force(distance, angle)
            # 侧面障碍物只负责让车辆横向绕行，不能把车辆推成倒车。
            # 只有正面扇区的斥力参与前向减速。
            if index == 1:
                total_forward += force_forward
            total_lateral += force_lateral

        obstacle_detected = any(
            distance is not None and distance < self.config.obstacle_distance_m
            for distance in (left, center, right)
        )

        # 正面障碍物的角度为 0，单靠斥力只能减小前向合力；
        # 这里根据左右剩余空间补一个横向选择力，让车辆绕向更宽的一侧。
        if center is not None and center < self.config.influence_distance_m:
            left_space = left if left is not None else self.config.invalid_depth_m
            right_space = right if right is not None else self.config.invalid_depth_m
            wider_side = -1.0 if left_space >= right_space else 1.0
            urgency = max(0.0, min(1.0, (self.config.influence_distance_m - center) / self.config.influence_distance_m))
            total_lateral += wider_side * self.config.center_turn_gain * urgency

        # 当前版本只做前向避障，不自动倒车；遇到近距离正面障碍时会停车并转向。
        throttle = max(0.0, min(self.config.max_throttle, total_forward))
        if center is not None and center >= self.config.obstacle_distance_m:
            # 没有外部目标点时，保持一个明确的默认前进速度。
            throttle = max(throttle, self.config.default_forward_throttle)
        steering = max(-self.config.max_steering, min(self.config.max_steering, total_lateral * self.config.steering_gain))
        if center is not None and center < self.config.obstacle_distance_m:
            throttle = 0.0
        elif throttle > 0.0:
            # 差速混合为 left=throttle+steering、right=throttle-steering。
            # 前方没有近障碍时，限制 |steering| 小于 throttle，保证两轮同向前进，
            # 侧方障碍物只让车辆弯行，不会把车辆变成原地旋转。
            steering_limit = min(self.config.max_steering, throttle * self.config.forward_steering_ratio)
            steering = max(-steering_limit, min(steering_limit, steering))
        # 对转向目标做一阶平滑，避免左右空间估计轻微抖动就立刻反向。
        smoothing = max(0.0, min(1.0, self.config.steering_smoothing))
        steering = self._steering_state + smoothing * (steering - self._steering_state)
        self._steering_state = steering
        reason = "前方通畅" if not obstacle_detected else "检测到障碍物，按合力绕行"
        return AvoidanceOutput(throttle, steering, left, center, right, obstacle_detected, reason)
