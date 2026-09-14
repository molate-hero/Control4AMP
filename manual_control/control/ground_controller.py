"""ESP32 地面车串口控制器。

ESP32 接收两类文本命令：
    S              停车
    D,left,right   左右轮速度，范围 -1000~1000

本模块不决定何时应该行驶，只负责可靠地发送已经经过安全策略的命令。
"""

from __future__ import annotations

import glob
import os
import threading
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - 在没有 pyserial 的开发机上给出清晰错误
    serial = None


class GroundController:
    """管理 ESP32 串口连接，并提供停车和差速命令发送接口。"""

    def __init__(self, port: str, baud: int, dry_run: bool = False) -> None:
        self.port = port
        self.baud = baud
        self.dry_run = dry_run or port.lower() in {"none", "simulation"}
        self._serial: Optional[object] = None
        self._lock = threading.RLock()
        self.last_command = "S"

    @staticmethod
    def discover_port() -> Optional[str]:
        """自动查找常见的 ESP32 USB 串口。"""

        candidates = sorted(glob.glob("/dev/ttyUSB*"))
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
        return candidates[0] if candidates else None

    def connect(self) -> None:
        """打开串口；仿真模式下只记录命令，不访问硬件。"""

        if self.dry_run:
            print("[GROUND] 仿真模式：不会向真实 ESP32 写入数据")
            return

        if self.port == "auto":
            self.port = self.discover_port() or ""
        if not self.port:
            raise RuntimeError("未找到 ESP32 串口，请使用 --ground-port 指定设备")
        if serial is None:
            raise RuntimeError("缺少 pyserial，请安装 requirements.txt 中的依赖")

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.1,
            write_timeout=0.2,
        )
        # 打开 USB 串口时，部分 ESP32 会自动复位，等待其完成启动。
        import time

        time.sleep(1.0)
        self._serial.reset_input_buffer()
        print(f"[GROUND] 已连接 ESP32：{self.port} @ {self.baud}")

    def send(self, command: str) -> None:
        """发送一条经过校验的地面车命令。"""

        if not command or "\n" in command or "\r" in command:
            raise ValueError("地面车命令格式非法")

        with self._lock:
            if self._serial is not None:
                self._serial.write(f"{command}\n".encode("utf-8"))
                self._serial.flush()
            else:
                print(f"[GROUND][SIM] {command}")
            self.last_command = command

    def stop(self) -> None:
        """向 ESP32 发送停车指令。"""

        self.send("S")

    def close(self) -> None:
        """关闭串口并尽力确保地面车停车。"""

        with self._lock:
            try:
                self.stop()
            finally:
                if self._serial is not None:
                    self._serial.close()
                    self._serial = None
