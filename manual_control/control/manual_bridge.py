"""手动遥控总控制器。

控制关系：
    遥控器 -> PX4 RC_CHANNELS -> 本程序
                            ├-> AIR：地面车停车，PX4 继续由遥控器飞行
                            └-> GROUND：将 CH1/CH2 转为 ESP32 差速指令

程序只做控制权分配和安全停车，不会自动解锁、起飞或改变飞行模式。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from config.defaults import RuntimeConfig
from control.channels import ControlDecision, resolve_decision
from control.ground_controller import GroundController
from control.px4_controller import Px4Controller


@dataclass(frozen=True)
class BridgeOptions:
    """运行时参数。"""

    px4_device: str
    px4_baud: int
    ground_device: str
    ground_baud: int
    rc_timeout: float
    heartbeat_timeout: float
    control_period: float
    print_period: float
    dry_run: bool = False


class ManualBridge:
    """将 PX4 RC 输入分配给飞行系统和 ESP32 地面车。"""

    def __init__(self, options: BridgeOptions) -> None:
        self.options = options
        self.px4 = Px4Controller(options.px4_device, options.px4_baud)
        self.ground = GroundController(
            options.ground_device,
            options.ground_baud,
            dry_run=options.dry_run,
        )
        self._last_print = 0.0
        self._last_mode_name: str | None = None

    def start(self) -> None:
        """连接硬件并进入手动控制循环。"""

        # 先连接 PX4，确保地面模式判定始终有飞控解锁状态依据。
        self.px4.connect()
        self.ground.connect()
        print("[CONTROL] 手动遥控已启动；程序不会自动解锁或起飞")

        try:
            self._loop()
        finally:
            # 无论退出原因是什么，先让地面车停车，再关闭串口。
            self.ground.close()
            self.px4.close()
            print("[CONTROL] 手动遥控已停止")

    def _loop(self) -> None:
        """以固定周期读取 PX4 状态并更新地面车输出。"""

        while True:
            self.px4.poll(self.options.control_period)
            snapshot = self.px4.snapshot
            now = time.monotonic()

            # 心跳或 RC 数据超时，地面车必须停车。
            if (
                self.px4.heartbeat_age() > self.options.heartbeat_timeout
                or self.px4.rc_age() > self.options.rc_timeout
            ):
                self.ground.stop()
                self._print_status(
                    "FAILSAFE",
                    "S",
                    "PX4 心跳或 RC 数据超时，地面车停车",
                    force=True,
                )
                continue

            channels = snapshot.rc_channels
            required = (1, 2, 7, 9)
            if not all(channel in channels for channel in required):
                self.ground.stop()
                self._print_status(
                    "FAILSAFE",
                    "S",
                    "尚未收到完整 RC 通道",
                    force=True,
                )
                continue

            decision = resolve_decision(
                channels[1],
                channels[2],
                channels[7],
                channels[9],
                snapshot.armed,
            )
            self.ground.send(decision.command)
            self._print_status(
                decision.mode.value,
                decision.command,
                decision.reason,
            )

    def _print_status(
        self,
        mode: str,
        command: str,
        reason: str,
        force: bool = False,
    ) -> None:
        """按模式变化或固定周期输出状态，方便现场排查。"""

        now = time.monotonic()
        changed = self._last_mode_name != mode
        if not force and not changed and now - self._last_print < self.options.print_period:
            return

        snapshot = self.px4.snapshot
        print(
            f"[CONTROL] mode={mode} command={command} reason={reason} "
            f"px4_mode={snapshot.mode_name} armed={snapshot.armed} "
            f"alt={snapshot.relative_altitude}m",
            flush=True,
        )
        self._last_print = now
        self._last_mode_name = mode


def default_options(dry_run: bool = False) -> BridgeOptions:
    """根据默认配置创建运行参数。"""

    defaults = RuntimeConfig()
    return BridgeOptions(
        px4_device=defaults.px4_device,
        px4_baud=defaults.px4_baud,
        ground_device=defaults.ground_device,
        ground_baud=defaults.ground_baud,
        rc_timeout=defaults.rc_timeout_seconds,
        heartbeat_timeout=defaults.heartbeat_timeout_seconds,
        control_period=defaults.control_period_seconds,
        print_period=defaults.print_period_seconds,
        dry_run=dry_run,
    )
