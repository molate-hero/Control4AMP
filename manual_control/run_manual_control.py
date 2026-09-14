#!/usr/bin/env python3
"""CONTROL 项目统一入口。

默认连接真实 PX4 和 ESP32；使用 --dry-run 时只模拟地面车输出，
但仍需要 PX4 提供 RC 通道和心跳。
"""

from __future__ import annotations

import argparse

from config.defaults import RuntimeConfig
from control.channels import run_logic_self_test
from control.manual_bridge import BridgeOptions, ManualBridge


def build_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""

    defaults = RuntimeConfig()
    parser = argparse.ArgumentParser(description="Orange Pi 手动遥控：PX4 + ESP32")
    parser.add_argument("--px4-device", default=defaults.px4_device, help="PX4 串口设备")
    parser.add_argument("--px4-baud", type=int, default=defaults.px4_baud, help="PX4 波特率")
    parser.add_argument(
        "--ground-port",
        default=defaults.ground_device,
        help="ESP32 串口设备，支持 auto 或 none",
    )
    parser.add_argument("--ground-baud", type=int, default=defaults.ground_baud, help="ESP32 波特率")
    parser.add_argument(
        "--rc-timeout",
        type=float,
        default=defaults.rc_timeout_seconds,
        help="RC 数据超时秒数",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=defaults.heartbeat_timeout_seconds,
        help="PX4 心跳超时秒数",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="不向真实 ESP32 写入命令，仅打印地面车输出",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="只运行通道解析和差速逻辑测试，不连接硬件",
    )
    return parser


def main() -> None:
    """程序入口。"""

    args = build_parser().parse_args()
    if args.self_test:
        run_logic_self_test()
        print("[TEST] 纯逻辑自检通过")
        return

    options = BridgeOptions(
        px4_device=args.px4_device,
        px4_baud=args.px4_baud,
        ground_device=args.ground_port,
        ground_baud=args.ground_baud,
        rc_timeout=args.rc_timeout,
        heartbeat_timeout=args.heartbeat_timeout,
        control_period=RuntimeConfig.control_period_seconds,
        print_period=RuntimeConfig.print_period_seconds,
        dry_run=args.dry_run,
    )
    ManualBridge(options).start()


if __name__ == "__main__":
    main()
