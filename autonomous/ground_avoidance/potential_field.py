"""地面车二维合力法避障核心。

本文件只接收深度矩阵并返回归一化差速控制量，不依赖串口、RealSense SDK、
Flask 或其他控制模块，因此可以在没有任何硬件的环境中单独测试和调参。
坐标约定：前方为 +X，右方为 +Y，最终 steering 的正方向表示向右转。

安全设计要点（与早期版本的差异也记录在 .ai/DECISIONS.md）：
1. 深度有效性分三态（近 / 远 / 未知）。正前方未知时按危险处理，直接停车，
   避免“看不见却满速前进”。
2. 前进速度由正前方距离的连续曲线决定，距离越近速度越低，不再出现
   恒速逼近后突然急停的悬崖式行为。
3. 侧方障碍物通过斥力产生转向；接近障碍物时放宽转向限制，允许差速车
   用更大的差速急转避让，而不是把转向比例固定在巡航值上。
4. 通过地面平面模型剔除近处地面像素。地面被误判为障碍会让可通行距离
   被严重低估，剔除后扇区距离才真正代表“到最近障碍物的距离”。

地面平面模型：相机前下方固定俯角的平地上，同一行像素对应的地面深度近似为
    s_ground(y) = K / (y - horizon)
其中 horizon 是地平线所在行、K 是尺度常数（单位：行·米）。两者只与相机
安装姿态有关，可用 `calibrate_ground.py` 一次性标定。详见 .ai/DECISIONS.md。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import radians, sin
from typing import Optional

import numpy as np


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """将浮点数限制在指定范围内。"""

    return max(minimum, min(maximum, value))


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
    # 低于 obstacle_distance_m 立即停车；从 slow_distance_m 开始按距离连续减速。
    obstacle_distance_m: float = 0.60
    slow_distance_m: float = 1.80
    influence_distance_m: float = 2.2
    # 扇区内有效测量像素占比低于该值时，该方向判定为未知而不是“通畅”。
    min_valid_ratio: float = 0.15

    # 地面平面模型 s_ground(y) = K / (y - horizon)，用于剔除近处地面像素。
    # 参数由 calibrate_ground.py 在现场标定，标定分辨率为 ground_reference_height；
    # 换用其他分辨率时按高度比例线性缩放。任一项为 None 时关闭地面剔除，
    # 退回到“固定 ROI”行为。相机安装角度或高度改变后必须重新标定。
    ground_horizon_row: Optional[float] = 141.7
    ground_scale_m_row: Optional[float] = 44.5
    ground_reference_height: int = 270
    # 实测深度达到“地面深度 - 该余量”即判定为地面。需大于深度噪声，又小于
    # 地面上障碍物的高度差，默认 0.20 米。
    ground_margin_m: float = 0.20

    # 合力参数：侧向斥力把车辆推离障碍物，正面距离决定前进速度。
    # 没有外部目标点时，把相机正前方定义为默认航行方向。
    repulsion_gain: float = 0.95
    center_turn_gain: float = 1.2
    steering_gain: float = 1.0
    max_throttle: float = 0.75
    default_forward_throttle: float = 0.65
    max_steering: float = 1.0
    # 巡航时的转向/推力比（0.35），避免差速混合后某个车轮反转、车辆画龙。
    forward_steering_ratio: float = 0.35
    # 障碍最近处转向比例放宽到的上限。注意它乘的是 throttle，所以即使取 1.0，
    # 内侧车轮也只是降到 0 而不会反转，不会出现“边前进边原地甩头”错过出口。
    close_steering_ratio: float = 1.0
    # 绕行方向切换余量：另一侧要宽出该距离才允许反向，避免在开口处左右反复。
    turn_switch_margin_m: float = 0.15
    # 正前方被堵、停车原地转向时的转向上限。取温和值，避免在狭小空间里
    # 旋转过快而转过头错过侧面开口；侧面一旦打开，距离变远会自动恢复前进。
    pivot_steering_limit: float = 0.60
    # 每次只吸收一部分新的转向目标，减少深度噪声引起的左右急摆。
    steering_smoothing: float = 0.25


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
    """使用前进目标和障碍物斥力计算地面车差速输入。"""

    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.config = config or AvoidanceConfig()
        self._steering_state = 0.0
        # 两侧信息相同或都未知时保持上一次转向方向，默认取左，避免左右抖动。
        self._turn_bias = -1.0

    def _ground_mask(self, depth_mm: np.ndarray) -> Optional[np.ndarray]:
        """返回与 depth_mm 同形状的布尔掩码，True 表示该像素判定为地面。

        地面模型 s_ground(y) = K / (y - horizon) 只描述地平线以下的行；地平线
        以上的行没有地面，返回 False。测量深度达到“地面深度 - 余量”即认为是
        地面：因为地面是这条视线方向上的最远平面，比地面更近的才是障碍物。
        未配置地面参数时返回 None，表示关闭地面剔除。
        """

        horizon = self.config.ground_horizon_row
        scale = self.config.ground_scale_m_row
        if horizon is None or scale is None or depth_mm.ndim != 2 or depth_mm.size == 0:
            return None

        height = depth_mm.shape[0]
        # 标定值是针对 ground_reference_height 的；换分辨率时按高度比例缩放。
        factor = height / float(self.config.ground_reference_height or height)
        horizon = horizon * factor
        scale = scale * factor

        rows = np.arange(height, dtype=np.float32).reshape(-1, 1)
        depth_m = depth_mm.astype(np.float32) / 1000.0
        with np.errstate(divide="ignore", invalid="ignore"):
            ground_depth = np.where(
                rows > horizon,
                scale / np.maximum(rows - horizon, 1e-6),
                np.inf,
            )
        return depth_m >= (ground_depth - self.config.ground_margin_m)

    def _sector_clearance(
        self,
        roi: np.ndarray,
        ground_roi: Optional[np.ndarray],
        left: int,
        right: int,
    ) -> Optional[float]:
        """返回一个扇区到最近障碍物的距离。

        返回 None 表示该方向“未知”（基本没有有效回波），调用方会按危险处理；
        返回 invalid_depth_m 表示有有效测量但全是地面/远处，即该方向畅通。
        """

        values = roi[:, left:right].astype(np.float32) / 1000.0
        total = values.size
        if total == 0:
            return None

        # 0 或负数代表没有回波，属于无效测量。有效测量占比过低时判定为未知，
        # 避免少量“很远的散点”把一个实际危险的方向洗成通畅。
        measured = np.isfinite(values) & (values > 0.0)
        if int(measured.sum()) < total * self.config.min_valid_ratio:
            return None

        # 剔除地面像素：它们不是障碍物。若剔除后没有剩余像素，说明该方向
        # 只有地面（没有站在地面上的障碍物），属于畅通。
        keep = measured
        if ground_roi is not None:
            keep = measured & ~ground_roi[:, left:right]
        readings = np.minimum(values[keep], self.config.invalid_depth_m)
        readings = readings[readings > 0.08]
        if readings.size == 0:
            return float(self.config.invalid_depth_m)
        return float(np.percentile(readings, self.config.clearance_percentile))

    def _clearances(self, depth_mm: np.ndarray) -> tuple[Optional[float], Optional[float], Optional[float]]:
        """裁剪感兴趣区域并估计左、中、右三块区域的距离。"""

        if depth_mm.ndim != 2 or depth_mm.size == 0:
            return None, None, None
        height, width = depth_mm.shape
        top = max(0, min(height - 1, int(height * self.config.roi_top_ratio)))
        bottom = max(top + 1, min(height, int(height * self.config.roi_bottom_ratio)))
        roi_left = max(0, min(width - 1, int(width * self.config.roi_left_ratio)))
        roi_right = max(roi_left + 1, min(width, int(width * self.config.roi_right_ratio)))

        ground = self._ground_mask(depth_mm)
        ground_roi = None if ground is None else ground[top:bottom, roi_left:roi_right]
        roi = depth_mm[top:bottom, roi_left:roi_right]

        sector_width = roi.shape[1] / 3.0
        distances = []
        for index in range(3):
            start = int(index * sector_width)
            end = int((index + 1) * sector_width)
            distances.append(
                self._sector_clearance(roi, ground_roi, start, max(start + 1, end))
            )
        return distances[0], distances[1], distances[2]

    def _repulsive_lateral(self, distance: Optional[float], angle_deg: float) -> float:
        """计算单个扇区的横向斥力；正方向表示把车辆推向右转。"""

        if distance is None or distance >= self.config.influence_distance_m:
            return 0.0
        distance = max(distance, 0.12)
        magnitude = self.config.repulsion_gain * (
            1.0 / distance - 1.0 / self.config.influence_distance_m
        ) / (distance * distance)
        # 障碍物位于左侧时角度为负，-sin(-28°) 为正，车辆向右转。
        return -magnitude * sin(radians(angle_deg))

    def _center_turn(self, center: float, left: Optional[float], right: Optional[float]) -> float:
        """正面近障碍时，产生指向更宽一侧的横向选择力。"""

        if left is not None and right is not None:
            # 带切换余量：只有另一侧明显更宽才反向，避免在开口处左右反复而错过出口。
            if left > right + self.config.turn_switch_margin_m:
                side = -1.0
            elif right > left + self.config.turn_switch_margin_m:
                side = 1.0
            else:
                side = self._turn_bias
        elif left is not None:
            side = -1.0
        elif right is not None:
            side = 1.0
        else:
            # 两侧都未知时沿用上一次方向，避免无信息时反复反向。
            side = self._turn_bias
        self._turn_bias = side

        # 只在 slow_distance 以内才开始侧向选择，避免提前很远就起转、错过近处开口。
        span = max(self.config.slow_distance_m - self.config.obstacle_distance_m, 1e-6)
        urgency = _clamp((self.config.slow_distance_m - center) / span, 0.0, 1.0)
        return side * self.config.center_turn_gain * urgency

    def _front_blocked(self, center: float) -> bool:
        """正前方是否近到需要停车并在原地转向。

        加 5 毫米容差：深度是 float32，0.6 米常表示为 0.60000002，严格比较会漏判，
        导致“既不前进也不转向”的死点。
        """

        return center <= self.config.obstacle_distance_m + 0.005

    def _forward_speed(self, center: float) -> float:
        """由正前方距离给出允许的前进速度，避免悬崖式急停。"""

        stop = self.config.obstacle_distance_m
        slow = self.config.slow_distance_m
        if self._front_blocked(center):
            return 0.0
        if center >= slow or slow <= stop:
            return _clamp(self.config.default_forward_throttle, 0.0, self.config.max_throttle)
        ratio = (center - stop) / (slow - stop)
        return _clamp(self.config.default_forward_throttle * ratio, 0.0, self.config.max_throttle)

    def _steering_limit(self, threat: float, throttle: float) -> float:
        """给出“行驶中”允许的转向上限，保证内侧车轮不反转。

        上限 = throttle * 比例，比例在巡航值 forward_steering_ratio 与最近值
        close_steering_ratio 之间按障碍距离插值。因为始终乘以 throttle，内侧车轮
        最多降到 0 而不会反转，避障时表现为“弯行”而不是“原地甩头”，才不会转过头
        错过开口。确实需要原地急转时，由 compute 在停车状态下另行放开。
        """

        far = self.config.forward_steering_ratio
        near = min(1.0, max(far, self.config.close_steering_ratio))
        stop = self.config.obstacle_distance_m
        slow = self.config.slow_distance_m
        if threat >= slow or slow <= stop:
            ratio = far
        elif threat <= stop:
            ratio = near
        else:
            blend = (slow - threat) / (slow - stop)
            ratio = far + (near - far) * blend
        return min(self.config.max_steering, throttle * ratio)

    def compute(self, depth_mm: np.ndarray) -> AvoidanceOutput:
        """根据一帧毫米深度图计算归一化 throttle/steering。"""

        left, center, right = self._clearances(depth_mm)
        if left is None and center is None and right is None:
            self._steering_state = 0.0
            return AvoidanceOutput(0.0, 0.0, left, center, right, True, "没有有效深度数据，停车")

        # 正前方深度未知时无法确认前方是否安全，按危险处理并停车，
        # 这是对早期版本“中心缺测仍满速前进”缺陷的直接修复。
        if center is None:
            self._steering_state = 0.0
            return AvoidanceOutput(0.0, 0.0, left, center, right, True, "正前方深度无效，停车待确认")

        sector_angle = self.config.sector_angle_deg
        total_lateral = 0.0
        for distance, angle in ((left, -sector_angle), (center, 0.0), (right, sector_angle)):
            # 侧方未知不产生转向力，避免在无信息时盲目打方向。
            total_lateral += self._repulsive_lateral(distance, angle)

        if center < self.config.slow_distance_m:
            total_lateral += self._center_turn(center, left, right)

        # 前进速度完全由正前方距离的连续曲线决定，斥力只负责横向绕行，
        # 不会再像早期版本那样把减速后的速度重新抬回默认值。
        throttle = self._forward_speed(center)
        steering = _clamp(
            total_lateral * self.config.steering_gain,
            -self.config.max_steering,
            self.config.max_steering,
        )

        # 差速混合为 left=throttle+steering、right=throttle-steering。
        # 行驶中把转向上限绑到 throttle，保证内侧车轮不反转，避障是“弯行”而非
        # “原地甩头”，否则会猛转过头错过开口；正前方被堵、已经停车时改用温和的
        # 原地转向上限，既能转向更宽的一侧，又不会转太快冲过侧面开口。
        front_blocked = self._front_blocked(center)
        threat = min(distance for distance in (left, center, right) if distance is not None)
        if front_blocked:
            limit = min(self.config.max_steering, self.config.pivot_steering_limit)
        else:
            limit = self._steering_limit(threat, throttle)
        steering = _clamp(steering, -limit, limit)

        # 对转向目标做一阶平滑，减少深度噪声引起的左右急摆；停车原地转向
        # 时不平滑，保证第一时间给出最大转向。平滑之后必须再限制一次：
        # 油门下降时上限会变小，仅靠平滑前的限制可能留下超过上限的旧值，
        # 从而让内侧车轮反转。
        if not front_blocked:
            smoothing = _clamp(self.config.steering_smoothing, 0.0, 1.0)
            steering = self._steering_state + smoothing * (steering - self._steering_state)
            steering = _clamp(steering, -limit, limit)
        self._steering_state = steering

        obstacle_detected = any(
            distance is not None and distance < self.config.slow_distance_m
            for distance in (left, center, right)
        )
        reason = "前方通畅" if not obstacle_detected else "检测到障碍物，按合力绕行"
        return AvoidanceOutput(throttle, steering, left, center, right, obstacle_detected, reason)
