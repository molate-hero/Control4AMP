"""VLM 可调用的地面运动工具（OpenAI function-calling 格式）。

目前只有三个工具，刻意保持最小集合：

- `run_forward(seconds, speed)`：前进 / 后退；
- `rotate(seconds, speed)`：原地旋转；
- `stop()`：停车，作为模型判断"不能再动"时的安全出口。

每个工具的 `description` 都用中文写清符号语义和取值边界，因为模型完全依赖
这些文字来决定怎么调用；描述里的措辞改动会直接影响模型行为，请谨慎修改。

本文件命名为 `vlm_tools.py` 而不是 `tools.py`：项目 README 已把顶层 `tools/`
预留给"诊断、标定和运维工具"，避免将来同名遮蔽。
"""

from __future__ import annotations

from typing import Any

# 供 VLM 使用的工具定义，直接作为 `tools` 参数发给 OpenAI 兼容接口。
TOOL_DEFINITIONS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "run_forward",
            "description": (
                "让地面车沿车头方向直线行驶指定的秒数。speed 为正表示前进，"
                "为负表示后退；speed 取值 -1.0 到 1.0，绝对值越大越快。"
                "seconds 取值 0.2 到 2.0，程序会把超出范围的值限制在上限内。"
                "执行结束后车辆会自动停车，无需再单独调用停车。"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "seconds": {"type": "number", "description": "运动时长，单位秒，建议 0.2~2.0。"},
                    "speed": {"type": "number", "description": "归一化速度，正为前进、负为后退，范围 -1.0~1.0。"},
                },
                "required": ["seconds", "speed"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "rotate",
            "description": (
                "让地面车原地旋转指定的秒数，车体位置基本不变、只改变朝向。"
                "speed 为正表示逆时针（左转），为负表示顺时针（右转）；"
                "speed 取值 -1.0 到 1.0，绝对值越大转得越快。"
                "seconds 取值 0.2 到 2.0，程序会把超出范围的值限制在上限内。"
                "执行结束后车辆会自动停车。"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "seconds": {"type": "number", "description": "旋转时长，单位秒，建议 0.2~2.0。"},
                    "speed": {"type": "number", "description": "归一化角速度，正为逆时针、负为顺时针，范围 -1.0~1.0。"},
                },
                "required": ["seconds", "speed"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop",
            "description": (
                "立即停车并保持静止。当画面显示前方有障碍、"
                "继续前进或旋转不安全，或者当前不需要移动时调用。"
            ),
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
]


def tool_names() -> list[str]:
    """返回已注册的工具名列表。"""

    return [definition["function"]["name"] for definition in TOOL_DEFINITIONS]


def _as_float(value: Any, default: float = 0.0) -> float:
    """把模型给的值稳妥地转成浮点数，非法值退回默认值。"""

    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def dispatch_tool(name: str, arguments: dict[str, Any], executor: Any) -> str:
    """执行一次工具调用，返回一段中文结果文本供写回 tool 消息。

    参数非法或工具名未知时**不抛异常**，而是把问题写回结果文本，让模型有机会
    自行纠正；这比直接中断整个循环更符合 function calling 的用法。
    """

    arguments = arguments or {}
    if name == "run_forward":
        seconds = _as_float(arguments.get("seconds"), 0.0)
        speed = _as_float(arguments.get("speed"), 0.0)
        command = executor.run_forward(seconds, speed)
        return f"已执行 run_forward：时长 {seconds:.2f} 秒、速度 {speed:.2f}，下发指令 {command}。"
    if name == "rotate":
        seconds = _as_float(arguments.get("seconds"), 0.0)
        speed = _as_float(arguments.get("speed"), 0.0)
        command = executor.rotate(seconds, speed)
        return f"已执行 rotate：时长 {seconds:.2f} 秒、速度 {speed:.2f}，下发指令 {command}。"
    if name == "stop":
        executor.stop()
        return "已停车。"
    return f"未知工具 {name}，本次未执行任何动作。"
