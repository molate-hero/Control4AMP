"""自动避障到 ESP32 差速协议的独立输出适配器。"""

from __future__ import annotations

from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - 纯算法测试环境
    serial = None


def build_drive_command(throttle: float, steering: float) -> str:
    """把归一化输入转换为 ESP32 的 D,left,right 文本命令。"""

    throttle = max(-1.0, min(1.0, float(throttle)))
    steering = max(-1.0, min(1.0, float(steering)))
    left = round((throttle + steering) * 1000)
    right = round((throttle - steering) * 1000)
    largest = max(abs(left), abs(right), 1000)
    if largest > 1000:
        left = round(left * 1000 / largest)
        right = round(right * 1000 / largest)
    if left == 0 and right == 0:
        return "S"
    return f"D,{left},{right}"


class Esp32Output:
    """独立的 ESP32 输出端；默认 dry_run，不会访问串口。"""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, dry_run: bool = True) -> None:
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self._serial: Optional[object] = None

    def open(self) -> None:
        if self.dry_run:
            return
        if serial is None:
            raise RuntimeError("缺少 pyserial")
        self._serial = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)

    def send(self, throttle: float, steering: float) -> str:
        command = build_drive_command(throttle, steering)
        if self._serial is not None:
            self._serial.write(f"{command}\n".encode("utf-8"))
            self._serial.flush()
        return command

    def stop(self) -> str:
        if self._serial is not None:
            self._serial.write(b"S\n")
            self._serial.flush()
        return "S"

    def close(self) -> None:
        try:
            self.stop()
        finally:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
