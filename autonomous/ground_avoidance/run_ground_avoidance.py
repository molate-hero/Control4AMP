"""RealSense 地面自动避障入口。

默认是 dry-run，只打印算法输出；只有显式添加 --hardware 才向 ESP32 发送命令。
"""

from __future__ import annotations

import argparse
import time

from esp32_output import Esp32Output
from potential_field import PotentialFieldAvoider
from realsense_source import RealSenseDepthSource


def main() -> None:
    parser = argparse.ArgumentParser(description="RealSense 合力法地面自动避障")
    parser.add_argument("--hardware", action="store_true", help="显式向真实 ESP32 发送差速命令")
    parser.add_argument("--esp32-port", default="/dev/ttyUSB0", help="ESP32 串口设备")
    parser.add_argument("--fps", type=int, default=30, help="深度帧率")
    args = parser.parse_args()

    avoider = PotentialFieldAvoider()
    output = Esp32Output(args.esp32_port, dry_run=not args.hardware)
    source = RealSenseDepthSource(fps=args.fps)
    output.open()
    source.start()
    print(f"[AUTONOMY] 模式={'HARDWARE' if args.hardware else 'DRY-RUN'}")
    try:
        while True:
            result = avoider.compute(source.read_depth())
            command = output.send(result.throttle, result.steering)
            print(
                f"[AUTONOMY] command={command} reason={result.reason} "
                f"clearance=({result.left_clearance_m},{result.center_clearance_m},{result.right_clearance_m})",
                flush=True,
            )
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("[AUTONOMY] 收到停止信号")
    finally:
        output.close()
        source.close()


if __name__ == "__main__":
    main()
