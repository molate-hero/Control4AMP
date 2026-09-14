"""CONTROL 项目的默认运行参数。

所有默认值都集中放在这里，便于在命令行参数之外统一查看和修改。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeConfig:
    """手动控制程序的默认硬件和安全参数。"""

    # PX4 飞控通过 Orange Pi 的 UART5 连接。
    px4_device: str = "/dev/ttyS5"
    px4_baud: int = 115200

    # ESP32 地面车默认通过 USB 串口连接。
    ground_device: str = "/dev/ttyUSB0"
    ground_baud: int = 115200

    # RC 通道和飞控心跳的失联保护时间。
    rc_timeout_seconds: float = 0.5
    heartbeat_timeout_seconds: float = 2.0

    # 主循环周期为 20 Hz，与原 manual_bridge 的控制周期一致。
    control_period_seconds: float = 0.05

    # 控制台状态输出周期，避免刷屏过快。
    print_period_seconds: float = 0.2
