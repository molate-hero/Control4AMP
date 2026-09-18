"""RealSense 彩色（RGB）图像采集与 JPEG 编码。

本模块只负责相机生命周期和图像编码，不做任何决策，也不连接串口。

与 `ground_avoidance/realsense_source.py` 的关系：两者都能占用同一台 RealSense，
但本模块只打开彩色流、那个只打开深度流，因此**不能同时运行**。切换时需要先
释放相机，`network_control` 已提供暂停/恢复深度预览的机制。
"""

from __future__ import annotations

import io
from typing import Any, Optional

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - 无 SDK 的纯逻辑测试环境
    rs = None

try:
    from PIL import Image
except ImportError:  # pragma: no cover - 无 Pillow 时给出清晰错误
    Image = None


def encode_jpeg_bgr(frame_bgr: Any, quality: int = 80) -> bytes:
    """把 BGR 顺序的彩色帧编码成 JPEG 字节。

    单独抽成纯函数，方便在没有相机、没有 pyrealsense2 的机器上直接测试。
    `ArrayLike` 类型直接用 `Any` 规避 numpy 版本之间的类型差异。
    """

    if Image is None:
        raise RuntimeError("缺少 Pillow，请安装 requirements.txt 中的依赖")
    array = np.asarray(frame_bgr)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("彩色帧必须是 HxWx3 的数组")
    # RealSense 彩色流是 BGR，Pillow 需要 RGB。
    rgb = np.ascontiguousarray(array[:, :, ::-1])
    image = Image.fromarray(rgb, mode="RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=int(quality))
    return buffer.getvalue()


class RealSenseColorSource:
    """提供 BGR 彩色帧，并可直接给出用于上传的 JPEG 字节。"""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        jpeg_quality: int = 80,
        frame_timeout_ms: int = 3000,
    ) -> None:
        if rs is None:
            raise RuntimeError("缺少 pyrealsense2，请在 OPi 的系统 Python 环境运行")
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.frame_timeout_ms = frame_timeout_ms
        self._pipeline: Any = rs.pipeline()
        self._started = False

    def start(self) -> None:
        """启动彩色流；若已有程序占用相机，SDK 会返回明确错误。"""

        if self._started:
            return
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self._pipeline.start(config)
        self._started = True

    def read_frame(self) -> np.ndarray:
        """等待一帧并复制成独立的 BGR 彩色矩阵。"""

        if not self._started:
            raise RuntimeError("RealSense 彩色源尚未启动")
        frames = self._pipeline.wait_for_frames(timeout_ms=self.frame_timeout_ms)
        color: Optional[Any] = frames.get_color_frame()
        if not color:
            raise RuntimeError("本次帧中没有彩色数据")
        return np.asanyarray(color.get_data()).copy()

    def capture_jpeg(self) -> bytes:
        """采集一帧并编码成 JPEG 字节，供 VLM 上传使用。"""

        return encode_jpeg_bgr(self.read_frame(), self.jpeg_quality)

    def close(self) -> None:
        """停止彩色流并释放 USB 相机。"""

        if self._started:
            self._pipeline.stop()
            self._started = False

    def __enter__(self) -> "RealSenseColorSource":
        self.start()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.close()
