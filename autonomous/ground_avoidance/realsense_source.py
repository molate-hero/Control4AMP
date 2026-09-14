"""RealSense 深度帧读取适配器。

该适配器只负责相机生命周期和深度帧，不负责避障决策，也不连接 ESP32。
"""

from __future__ import annotations

from typing import Any

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - 无 SDK 的纯算法测试环境
    rs = None


class RealSenseDepthSource:
    """以低分辨率深度流为自动避障算法提供毫米矩阵。"""

    def __init__(self, width: int = 480, height: int = 270, fps: int = 30) -> None:
        if rs is None:
            raise RuntimeError("缺少 pyrealsense2，请在 OPi 的系统 Python 环境运行")
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline: Any = rs.pipeline()
        self._started = False

    def start(self) -> None:
        """启动深度流；若已有程序占用相机，RealSense SDK 会返回明确错误。"""

        if self._started:
            return
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self._pipeline.start(config)
        self._started = True

    def read_depth(self) -> np.ndarray:
        """等待一帧并复制成独立的毫米深度矩阵。"""

        if not self._started:
            raise RuntimeError("RealSense 深度源尚未启动")
        frames = self._pipeline.wait_for_frames(timeout_ms=3000)
        depth = frames.get_depth_frame()
        if not depth:
            raise RuntimeError("本次帧中没有深度数据")
        return np.asanyarray(depth.get_data()).copy()

    def close(self) -> None:
        """停止深度流并释放 USB 相机。"""

        if self._started:
            self._pipeline.stop()
            self._started = False

    def __enter__(self) -> "RealSenseDepthSource":
        self.start()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.close()
