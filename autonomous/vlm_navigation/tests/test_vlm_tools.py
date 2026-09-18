"""工具注册与派发测试：验证 schema 完整性和参数落到执行器的方式。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import vlm_tools


class RecordingExecutor:
    """记录每次工具调用的假执行器。"""

    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.last_command = "S"

    def run_forward(self, seconds: float, speed: float) -> str:
        self.calls.append(("run_forward", seconds, speed))
        self.last_command = "D,500,500"
        return self.last_command

    def rotate(self, seconds: float, speed: float) -> str:
        self.calls.append(("rotate", seconds, speed))
        self.last_command = "D,500,-500"
        return self.last_command

    def stop(self) -> str:
        self.calls.append(("stop",))
        self.last_command = "S"
        return "S"


class ToolDefinitionsTest(unittest.TestCase):
    """检查工具定义满足 OpenAI function-calling 的最小结构要求。"""

    def test_expected_tools_are_registered(self) -> None:
        self.assertEqual(sorted(vlm_tools.tool_names()), ["rotate", "run_forward", "stop"])

    def test_definitions_have_required_structure(self) -> None:
        for definition in vlm_tools.TOOL_DEFINITIONS:
            self.assertEqual(definition["type"], "function")
            function = definition["function"]
            self.assertTrue(function["name"])
            # 描述必须非空：模型完全依赖它理解参数语义。
            self.assertTrue(function["description"].strip())
            self.assertEqual(function["parameters"]["type"], "object")

    def test_motion_tools_require_seconds_and_speed(self) -> None:
        by_name = {d["function"]["name"]: d["function"] for d in vlm_tools.TOOL_DEFINITIONS}
        for name in ("run_forward", "rotate"):
            required = by_name[name]["parameters"]["required"]
            self.assertIn("seconds", required)
            self.assertIn("speed", required)


class DispatchTest(unittest.TestCase):
    """验证派发把参数原样传给执行器，并返回中文结果文本。"""

    def setUp(self) -> None:
        self.executor = RecordingExecutor()

    def test_dispatch_run_forward(self) -> None:
        result = vlm_tools.dispatch_tool("run_forward", {"seconds": 1.5, "speed": 0.6}, self.executor)

        self.assertEqual(self.executor.calls, [("run_forward", 1.5, 0.6)])
        self.assertIn("run_forward", result)

    def test_dispatch_rotate(self) -> None:
        vlm_tools.dispatch_tool("rotate", {"seconds": 0.5, "speed": -0.5}, self.executor)

        self.assertEqual(self.executor.calls, [("rotate", 0.5, -0.5)])

    def test_dispatch_stop(self) -> None:
        result = vlm_tools.dispatch_tool("stop", {}, self.executor)

        self.assertEqual(self.executor.calls, [("stop",)])
        self.assertIn("停车", result)

    def test_unknown_tool_does_not_raise(self) -> None:
        result = vlm_tools.dispatch_tool("fly_away", {}, self.executor)

        self.assertEqual(self.executor.calls, [])
        self.assertIn("未知工具", result)

    def test_bad_arguments_fall_back_to_zero(self) -> None:
        # 模型偶尔会给出非数值参数，应退回 0 而不是抛异常。
        vlm_tools.dispatch_tool("run_forward", {"seconds": "很久", "speed": None}, self.executor)

        self.assertEqual(self.executor.calls, [("run_forward", 0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
