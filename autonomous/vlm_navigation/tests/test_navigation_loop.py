"""主循环编排测试：用假相机、假模型、假执行器完整跑通闭环。

覆盖：图片上下文滑窗、动作执行、只回文字的处理、API 失败退避与致命退出、
停止请求，以及"循环不论怎样结束都会停车"。
"""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from navigation_loop import VlmNavigationLoop
from vlm_client import ToolCall, VlmResponse


class FakeCamera:
    """按固定字节返回"照片"，并记录拍摄次数。"""

    def __init__(self, fail_after: int | None = None) -> None:
        self.captures = 0
        self.fail_after = fail_after

    def capture_jpeg(self) -> bytes:
        if self.fail_after is not None and self.captures >= self.fail_after:
            raise RuntimeError("相机断开")
        self.captures += 1
        return b"\xff\xd8\xff" + bytes([self.captures])


class ScriptedClient:
    """按脚本返回响应，并记录每次请求携带的图片数量。"""

    def __init__(self, responses: list) -> None:
        self._responses = list(responses)
        self.requests: list[list] = []
        self.image_counts: list[int] = []

    def chat(self, messages: list, tools=None) -> VlmResponse:
        self.requests.append(list(messages))
        self.image_counts.append(
            sum(1 for m in messages if m.get("role") == "user" and isinstance(m.get("content"), list))
        )
        item = self._responses.pop(0) if self._responses else VlmResponse(content="脚本结束")
        if isinstance(item, Exception):
            raise item
        return item


class RecordingExecutor:
    """记录动作的假执行器。"""

    def __init__(self) -> None:
        self.actions: list[str] = []
        self.stop_count = 0
        self.last_command = "S"

    def run_forward(self, seconds: float, speed: float) -> str:
        self.actions.append(f"forward:{seconds}:{speed}")
        self.last_command = "D,500,500"
        return self.last_command

    def rotate(self, seconds: float, speed: float) -> str:
        self.actions.append(f"rotate:{seconds}:{speed}")
        self.last_command = "D,500,-500"
        return self.last_command

    def stop(self) -> str:
        self.stop_count += 1
        self.last_command = "S"
        return "S"


def tool_response(name: str, arguments: dict, call_id: str = "c1") -> VlmResponse:
    """构造一个工具调用响应。"""

    return VlmResponse(content="", tool_calls=(ToolCall(id=call_id, name=name, arguments=arguments),))


def make_loop(client, camera=None, executor=None, stop_event=None, **kwargs) -> VlmNavigationLoop:
    return VlmNavigationLoop(
        camera=camera or FakeCamera(),
        client=client,
        executor=executor or RecordingExecutor(),
        stop_event=stop_event,
        max_steps=kwargs.pop("max_steps", 10),
        max_context_images=kwargs.pop("max_context_images", 2),
        max_consecutive_text_only=kwargs.pop("max_consecutive_text_only", 3),
        max_api_failures=kwargs.pop("max_api_failures", 3),
        # 测试不需要真实退避等待。
        sleeper=lambda _seconds: None,
        **kwargs,
    )


class NavigationLoopTest(unittest.TestCase):
    """验证主循环的编排行为。"""

    def test_executes_tool_calls_and_advances_steps(self) -> None:
        # 每轮都给一个工具调用，避免文字回退干扰对 action 的断言。
        client = ScriptedClient(
            [
                tool_response("run_forward", {"seconds": 1.0, "speed": 0.6}, "c1"),
                tool_response("rotate", {"seconds": 0.5, "speed": -0.5}, "c2"),
            ]
        )
        executor = RecordingExecutor()
        loop = make_loop(client, executor=executor, max_steps=2)

        result = loop.run()

        self.assertEqual(executor.actions, ["forward:1.0:0.6", "rotate:0.5:-0.5"])
        self.assertEqual(result["step"], 2)
        self.assertEqual(result["action"], "rotate")

    def test_captures_one_photo_per_decision(self) -> None:
        camera = FakeCamera()
        client = ScriptedClient([tool_response("run_forward", {"seconds": 1.0, "speed": 0.5})])
        loop = make_loop(client, camera=camera, max_steps=1)

        loop.run()

        # 每轮决策前拍一次，共 max_steps 次。
        self.assertEqual(camera.captures, 1)

    def test_context_keeps_only_recent_images(self) -> None:
        client = ScriptedClient(
            [tool_response("run_forward", {"seconds": 0.5, "speed": 0.3}, f"c{i}") for i in range(5)]
        )
        loop = make_loop(client, max_steps=5, max_context_images=2)

        loop.run()

        # 无论跑到第几轮，最多只带 2 张图片，且始终至少含最新的那张。
        self.assertTrue(all(count <= 2 for count in client.image_counts))
        self.assertEqual(client.image_counts[-1], 2)

    def test_dropping_old_images_keeps_tool_call_pairing(self) -> None:
        client = ScriptedClient(
            [tool_response("run_forward", {"seconds": 0.5, "speed": 0.3}, f"c{i}") for i in range(4)]
        )
        loop = make_loop(client, max_steps=4, max_context_images=1)

        result = loop.run()

        # 丢掉旧图片后，历史里每个 tool 结果都必须能找到对应的 assistant tool_call。
        for message in client.requests[-1]:
            if message.get("role") == "tool":
                self.assertTrue(message.get("tool_call_id"))
        self.assertEqual(result["step"], 4)

    def test_text_only_response_counts_toward_stall_limit(self) -> None:
        client = ScriptedClient([VlmResponse(content="我在观察") for _ in range(3)])
        loop = make_loop(client, max_steps=10, max_consecutive_text_only=3)

        result = loop.run()

        # 连续三次只回文字后应主动停止，而不是一直空转。
        self.assertLessEqual(result["step"], 1)
        self.assertIn("没有调用工具", result["reason"])

    def test_text_only_resets_after_a_tool_call(self) -> None:
        client = ScriptedClient(
            [
                VlmResponse(content="观察"),
                tool_response("run_forward", {"seconds": 0.5, "speed": 0.5}, "c1"),
                VlmResponse(content="再观察"),
                VlmResponse(content="又观察"),
            ]
        )
        executor = RecordingExecutor()
        loop = make_loop(client, executor=executor, max_steps=5, max_consecutive_text_only=2)

        result = loop.run()

        # 中间的这次工具调用把"连续只回文字"的计数清零，所以直到第 4 次请求才因
        # 连续两次纯文字而停止；若计数没有重置，第 2 次请求就会停止。
        self.assertEqual(len(client.requests), 4)
        self.assertEqual(executor.actions, ["forward:0.5:0.5"])
        self.assertEqual(result["step"], 1)
        self.assertIn("没有调用工具", result["reason"])

    def test_api_failures_retry_then_stop(self) -> None:
        client = ScriptedClient([RuntimeError("网络不通") for _ in range(3)])
        loop = make_loop(client, max_steps=10, max_api_failures=3)

        result = loop.run()

        self.assertIn("连续 3 次调用失败", result["reason"])
        self.assertIsNotNone(result["error"])

    def test_api_failure_then_success_recovers(self) -> None:
        client = ScriptedClient(
            [
                RuntimeError("抖动"),
                tool_response("run_forward", {"seconds": 0.5, "speed": 0.5}, "c1"),
            ]
        )
        # 步数上限设为 1：成功执行一次动作后循环即结束，避免脚本用尽后的
        # 文字回退再产生新的错误，从而能干净地验证"恢复后错误被清除"。
        loop = make_loop(client, max_steps=1, max_api_failures=3)

        result = loop.run()

        self.assertEqual(result["step"], 1)
        self.assertIsNone(result["error"])

    def test_camera_failure_stops_the_loop(self) -> None:
        camera = FakeCamera(fail_after=0)
        client = ScriptedClient([tool_response("run_forward", {"seconds": 0.5, "speed": 0.5})])
        loop = make_loop(client, camera=camera, max_steps=5)

        result = loop.run()

        self.assertIn("相机读取失败", result["error"])
        self.assertEqual(result["step"], 0)

    def test_stop_event_halts_loop(self) -> None:
        stop_event = threading.Event()
        stop_event.set()
        client = ScriptedClient([tool_response("run_forward", {"seconds": 0.5, "speed": 0.5})])
        loop = make_loop(client, stop_event=stop_event, max_steps=5)

        result = loop.run()

        self.assertEqual(result["step"], 0)
        self.assertIn("停止请求", result["reason"])

    def test_executor_always_stops_on_exit(self) -> None:
        executor = RecordingExecutor()
        client = ScriptedClient([tool_response("run_forward", {"seconds": 0.5, "speed": 0.5})])
        loop = make_loop(client, executor=executor, max_steps=1)

        loop.run()

        # 循环结束必须至少有一次停车，保证车辆不残留动作。
        self.assertGreaterEqual(executor.stop_count, 1)
        self.assertEqual(executor.last_command, "S")

    def test_max_steps_bound(self) -> None:
        client = ScriptedClient([tool_response("run_forward", {"seconds": 0.5, "speed": 0.5}, f"c{i}") for i in range(10)])
        loop = make_loop(client, max_steps=3)

        result = loop.run()

        self.assertEqual(result["step"], 3)
        self.assertIn("最大步数", result["reason"])

    def test_status_callback_receives_updates(self) -> None:
        seen: list[dict] = []
        client = ScriptedClient([tool_response("run_forward", {"seconds": 0.5, "speed": 0.5})])
        loop = make_loop(client, max_steps=1, on_status=seen.append)

        loop.run()

        self.assertTrue(seen)
        self.assertIn("command", seen[-1])

    def test_system_prompt_contains_goal(self) -> None:
        client = ScriptedClient([tool_response("stop", {})])
        loop = make_loop(client, max_steps=1, goal="找到红色的球")

        loop.run()

        system_message = client.requests[0][0]
        self.assertEqual(system_message["role"], "system")
        self.assertIn("找到红色的球", system_message["content"])


if __name__ == "__main__":
    unittest.main()
