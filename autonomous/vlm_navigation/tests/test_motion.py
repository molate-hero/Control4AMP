"""运动执行器纯逻辑测试：不需要相机、串口和网络。

用假时钟把 sleep 变成"推进虚拟时间"，因此测试里不会真实等待。
"""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from motion import MotionExecutor


class FakeClock:
    """假时钟：sleep 时推进虚拟时间。"""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeDriver:
    """记录所有下发指令的假驱动。"""

    def __init__(self) -> None:
        self.commands: list[str] = []

    def send(self, command: str) -> None:
        self.commands.append(command)

    def stop(self) -> None:
        self.commands.append("S")


def make_executor(**kwargs) -> tuple[FakeDriver, MotionExecutor]:
    """创建使用假时钟、假驱动的执行器。"""

    clock = FakeClock()
    driver = FakeDriver()
    executor = MotionExecutor(driver, clock=clock, sleeper=clock.sleep, **kwargs)
    return driver, executor


class MotionExecutorTest(unittest.TestCase):
    """验证时长控制、符号约定、限幅和停车行为。"""

    def test_forward_sends_command_and_always_stops(self) -> None:
        driver, executor = make_executor(tick_seconds=0.05, max_action_seconds=2.0, max_speed=1.0)

        executor.run_forward(0.2, 0.5)

        # 第一条是前进指令，最后一条一定是停车。
        self.assertEqual(driver.commands[0], "D,500,500")
        self.assertEqual(driver.commands[-1], "S")
        # 时长内必须多次重发，满足看门狗对指令刷新的要求。
        self.assertEqual([c for c in driver.commands if c != "S"], ["D,500,500"] * 4)

    def test_negative_speed_means_backward(self) -> None:
        driver, executor = make_executor()

        executor.run_forward(0.1, -0.5)

        self.assertEqual(driver.commands[0], "D,-500,-500")

    def test_rotate_positive_is_counter_clockwise(self) -> None:
        driver, executor = make_executor()

        executor.rotate(0.1, 0.5)

        # 左轮后退、右轮前进 = 逆时针（左转）。
        self.assertEqual(driver.commands[0], "D,-500,500")

    def test_rotate_negative_is_clockwise_matching_existing_protocol(self) -> None:
        driver, executor = make_executor()

        executor.rotate(0.1, -0.5)

        # 与 manual_control 约定的 `D,500,-500` 表示原地右转一致。
        self.assertEqual(driver.commands[0], "D,500,-500")

    def test_speed_is_clamped_to_max(self) -> None:
        driver, executor = make_executor(max_speed=1.0)

        executor.run_forward(0.1, 5.0)

        self.assertEqual(driver.commands[0], "D,1000,1000")

    def test_duration_is_clamped_to_max(self) -> None:
        driver, executor = make_executor(tick_seconds=0.05, max_action_seconds=0.2)

        executor.run_forward(99.0, 0.5)

        # 99 秒被夹到 0.2 秒，按 0.05 的 tick 重发 4 次。
        self.assertEqual(len([c for c in driver.commands if c != "S"]), 4)
        self.assertEqual(driver.commands[-1], "S")

    def test_zero_seconds_does_not_move(self) -> None:
        driver, executor = make_executor()

        executor.run_forward(0.0, 0.5)

        self.assertEqual(driver.commands, ["S"])

    def test_stop_event_prevents_motion(self) -> None:
        stop_event = threading.Event()
        stop_event.set()
        driver, executor = make_executor(stop_event=stop_event)

        executor.run_forward(1.0, 0.5)

        # 已请求停止时不应产生任何运动指令，只保证停车。
        self.assertEqual(driver.commands, ["S"])

    def test_on_command_callback_reports_each_command(self) -> None:
        seen: list[str] = []
        clock = FakeClock()
        driver = FakeDriver()
        executor = MotionExecutor(
            driver,
            tick_seconds=0.05,
            max_action_seconds=1.0,
            clock=clock,
            sleeper=clock.sleep,
            on_command=seen.append,
        )

        executor.run_forward(0.1, 0.5)

        self.assertEqual(seen[0], "D,500,500")
        self.assertEqual(seen[-1], "S")

    def test_last_command_tracks_latest_command(self) -> None:
        _, executor = make_executor()

        command = executor.run_forward(0.1, 0.5)

        self.assertEqual(command, "D,500,500")
        # 动作结束后回到停车状态。
        self.assertEqual(executor.last_command, "S")


if __name__ == "__main__":
    unittest.main()
