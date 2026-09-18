"""基于时长的地面运动执行。

VLM 给出的动作是"以某个速度运动一段时间"，而 ESP32 指令**不上锁存**：一条
`D,left,right` 只在收到后生效，`network_control` 还有 0.5 秒地面看门狗，超时
会强制停车。因此一次动作必须在时长内**按 tick 反复重发**同一条指令，时长到点
后一定发送停车指令 `S`。

所有限幅（时长、速度）都在本模块完成，避免模型给出危险参数。差速混合复用
`manual_control` 的 `build_axis_command`，不重复实现第三份混合逻辑。

速度符号约定：
- `run_forward`：speed > 0 前进，speed < 0 后退。
- `rotate`：speed > 0 逆时针（左转），speed < 0 顺时针（右转）。
  这与既有 `D,500,-500` 表示原地右转的约定一致（见 manual_control/README.md）。
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from ground_link import build_axis_command

DEFAULT_TICK_SECONDS = 0.05
DEFAULT_MAX_ACTION_SECONDS = 2.0
DEFAULT_MAX_SPEED = 1.0


class MotionExecutor:
    """把一个"时长 + 速度"的动作翻译成持续重发的差速指令。

    `clock` 与 `sleeper` 可注入，测试时用假时钟即可不真实等待地验证时长逻辑。
    `on_command` 在每次下发指令后回调，供上层刷新看门狗和状态显示。
    """

    def __init__(
        self,
        driver: object,
        tick_seconds: float = DEFAULT_TICK_SECONDS,
        max_action_seconds: float = DEFAULT_MAX_ACTION_SECONDS,
        max_speed: float = DEFAULT_MAX_SPEED,
        stop_event: Optional[object] = None,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        on_command: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.driver = driver
        self.tick_seconds = max(0.01, float(tick_seconds))
        self.max_action_seconds = max(0.0, float(max_action_seconds))
        self.max_speed = max(0.0, float(max_speed))
        self._stop_event = stop_event
        self._clock = clock
        self._sleeper = sleeper
        self._on_command = on_command
        self.last_command = "S"

    def _stop_requested(self) -> bool:
        """是否收到了停止请求。"""

        return self._stop_event is not None and self._stop_event.is_set()

    def _clamp_speed(self, speed: float) -> float:
        """把速度限制在 ±max_speed 之内。"""

        try:
            value = float(speed)
        except (TypeError, ValueError):
            value = 0.0
        return max(-self.max_speed, min(self.max_speed, value))

    def _clamp_seconds(self, seconds: float) -> float:
        """把时长限制在 0 ~ max_action_seconds 之内。"""

        try:
            value = float(seconds)
        except (TypeError, ValueError):
            value = 0.0
        return max(0.0, min(self.max_action_seconds, value))

    def _emit(self, command: str) -> None:
        """发送一条指令并更新最近指令记录。"""

        self.driver.send(command)
        self.last_command = command
        if self._on_command is not None:
            self._on_command(command)

    def _timed_drive(self, command: str, seconds: float) -> str:
        """在指定时长内持续重发指令，结束（或被中断）时一定停车。"""

        seconds = self._clamp_seconds(seconds)
        if seconds <= 0.0 or self._stop_requested():
            # 时长为零或已请求停止时不产生运动，只保证停车。
            self.stop()
            return "S"

        deadline = self._clock() + seconds
        self._emit(command)
        while not self._stop_requested() and self._clock() < deadline:
            self._sleeper(self.tick_seconds)
            if self._stop_requested() or self._clock() >= deadline:
                break
            self._emit(command)
        # 无论正常结束还是被中断，都必须回到停车状态。
        self.stop()
        return command

    def run_forward(self, seconds: float, speed: float) -> str:
        """前进（speed < 0 为后退）指定秒数，返回实际下发的差速指令。"""

        speed = self._clamp_speed(speed)
        return self._timed_drive(build_axis_command(speed, 0.0), seconds)

    def rotate(self, seconds: float, speed: float) -> str:
        """原地旋转（speed < 0 为顺时针）指定秒数。"""

        speed = self._clamp_speed(speed)
        # steering 取负：speed 为负时 steering 为正，即向右（顺时针）转。
        return self._timed_drive(build_axis_command(0.0, -speed), seconds)

    def stop(self) -> str:
        """立即停车。"""

        self.driver.stop()
        self.last_command = "S"
        if self._on_command is not None:
            self._on_command("S")
        return "S"
