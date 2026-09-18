"""彩色图源测试：本机没有 RealSense，只测可在本机运行的编码与依赖检查。

JPEG 编码是纯函数，开发机的 Pillow 就能跑，因此这部分可以离线回归。
"""

from __future__ import annotations

import io
import sys
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import numpy as np

import realsense_color_source as source

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None


@unittest.skipUnless(Image is not None, "本机没有 Pillow")
class EncodeJpegTest(unittest.TestCase):
    """验证 BGR → JPEG 的编码与颜色通道顺序。"""

    def test_solid_color_keeps_channel_order(self) -> None:
        # BGR 全图 = (蓝 10, 绿 20, 红 30)。
        frame = np.zeros((8, 6, 3), dtype=np.uint8)
        frame[:, :] = (10, 20, 30)

        jpeg = source.encode_jpeg_bgr(frame, quality=95)
        decoded = Image.open(io.BytesIO(jpeg)).convert("RGB")

        self.assertEqual(decoded.size, (6, 8))
        red, green, blue = decoded.getpixel((3, 4))
        # 若误把 BGR 当 RGB，红色与蓝色会互换；用容差吸收 JPEG 压缩误差。
        self.assertLess(abs(red - 30), 12)
        self.assertLess(abs(green - 20), 12)
        self.assertLess(abs(blue - 10), 12)

    def test_invalid_shape_raises(self) -> None:
        with self.assertRaises(ValueError):
            source.encode_jpeg_bgr(np.zeros((8, 6), dtype=np.uint8))

    def test_output_is_jpeg_soi(self) -> None:
        frame = np.zeros((4, 4, 3), dtype=np.uint8)

        jpeg = source.encode_jpeg_bgr(frame)

        # JPEG 文件以 SOI 标记 FF D8 开头。
        self.assertEqual(jpeg[:2], b"\xff\xd8")


class DependencyGuardTest(unittest.TestCase):
    """没有 pyrealsense2 时应给出清晰错误，而不是 NoneType 崩溃。"""

    @unittest.skipIf(source.rs is not None, "本机已装 pyrealsense2")
    def test_missing_pyrealsense2_raises_runtime_error(self) -> None:
        with self.assertRaises(RuntimeError):
            source.RealSenseColorSource()


if __name__ == "__main__":
    unittest.main()
