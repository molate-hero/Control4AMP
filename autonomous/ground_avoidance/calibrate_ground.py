"""地面平面标定工具。

在平地上、相机按实际安装姿态固定好后运行本脚本，它会用深度数据拟合
地面模型 s_ground(y) = K / (y - horizon)，并打印可直接填回
AvoidanceConfig 的参数值。只读相机，不驱动任何硬件。

用法：
    PYTHONPATH=/home/orangepi/.local/lib/python3.10/site-packages python3 calibrate_ground.py

标定要点：
1. 车辆必须处于水平平地，前方 2~3 米内不要有障碍物；
2. 相机按实际安装姿态固定（含俯仰角），标定结果只对该姿态有效；
3. 相机俯仰角或安装高度改变后需要重新标定。
"""

from __future__ import annotations

import argparse

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - 无 SDK 的纯算法测试环境
    rs = None


def fit_ground(
    depth_m: np.ndarray,
    min_ratio: float = 0.4,
    fit_from_ratio: float = 0.5,
) -> tuple[float | None, float | None, int, float | None]:
    """在图像下半部拟合地面双曲线，返回 (horizon_row, K, 内点行数, 残差)。

    地面满足 1/s = (y - horizon)/K，即 1/s 对行号 y 线性：斜率 1/K、截距
    -horizon/K。采用“拟合-剔除离群-重拟合”的稳健流程，避免障碍物拉偏结果。
    """

    height, width = depth_m.shape
    rows: list[int] = []
    inverse: list[float] = []
    for y in range(int(height * fit_from_ratio), height):
        row = depth_m[y]
        valid = row[(row > 0.08) & (row < 8.0)]
        if valid.size < width * min_ratio:
            continue
        rows.append(y)
        inverse.append(1.0 / float(np.median(valid)))
    if len(rows) < 6:
        return None, None, 0, None

    ys = np.asarray(rows, dtype=np.float64)
    inv = np.asarray(inverse)
    slope, intercept = np.polyfit(ys, inv, 1)
    for _ in range(3):
        residual = inv - (slope * ys + intercept)
        mad = float(np.median(np.abs(residual - np.median(residual)))) + 1e-9
        keep = np.abs(residual) < 3.0 * mad
        if int(keep.sum()) < 6:
            break
        slope, intercept = np.polyfit(ys[keep], inv[keep], 1)
        ys, inv = ys[keep], inv[keep]
    if slope <= 0:
        return None, None, len(rows), None

    horizon = -intercept / slope
    scale = 1.0 / slope
    residual = float(np.median(np.abs(inv - (slope * ys + intercept))))
    return float(horizon), float(scale), len(ys), residual


def capture_depth(frames: int = 60) -> np.ndarray:
    """读取若干帧深度并取中值，降低噪声对拟合的影响。"""

    if rs is None:
        raise RuntimeError("缺少 pyrealsense2，请在 Orange Pi 的系统 Python 环境运行")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 30)
    profile = pipeline.start(config)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        collected = []
        for _ in range(frames):
            depth_frame = pipeline.wait_for_frames(timeout_ms=3000).get_depth_frame()
            if depth_frame:
                collected.append(np.asanyarray(depth_frame.get_data()).copy())
        if not collected:
            raise RuntimeError("没有读到任何深度帧")
        return np.median(np.stack(collected), axis=0).astype(np.float32) * scale
    finally:
        pipeline.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="标定地面平面模型 s=K/(y-horizon)")
    parser.add_argument("--frames", type=int, default=60, help="用于取中值的深度帧数")
    args = parser.parse_args()

    depth_m = capture_depth(args.frames)
    height = depth_m.shape[0]

    print("=== 逐行深度剖面 ===")
    for index in range(18):
        start = int(height * index / 18)
        end = int(height * (index + 1) / 18)
        band = depth_m[start:end]
        valid = band[(band > 0) & (band < 8.0)]
        median = float(np.median(valid)) if valid.size else float("nan")
        ratio = float(np.mean((band > 0) & (band < 8.0)) * 100)
        print(f"  行{start:3d}-{end:3d} 中位={median:6.2f}m 有效={ratio:5.1f}%")

    horizon, scale, inliers, residual = fit_ground(depth_m)
    print()
    print("=== 地面模型拟合结果 ===")
    if horizon is None:
        print("拟合失败：请确认车辆在平地、前方无遮挡、相机能看到 2~3 米内的地面。")
        return
    print(f"horizon_row ≈ {horizon:.1f}    K ≈ {scale:.1f}    内点行数={inliers}    残差中位={residual:.4f}")

    print()
    print("=== 请把下面两行填回 AvoidanceConfig ===")
    print(f"    ground_horizon_row: Optional[float] = {horizon:.1f}")
    print(f"    ground_scale_m_row: Optional[float] = {scale:.1f}")

    print()
    print("=== 逐行核对：实测中位 vs 地面预测 ===")
    for index in range(6, 18):
        start = int(height * index / 18)
        end = int(height * (index + 1) / 18)
        center = (start + end) / 2
        if center <= horizon:
            continue
        band = depth_m[start:end]
        valid = band[(band > 0) & (band < 8.0)]
        median = float(np.median(valid)) if valid.size else float("nan")
        predicted = scale / (center - horizon)
        tag = "地面" if median >= predicted - 0.20 else "障碍/未知"
        print(f"  行{start:3d}-{end:3d} 实测={median:5.2f}m 预测={predicted:5.2f}m -> {tag}")


if __name__ == "__main__":
    main()
