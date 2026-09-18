"""VLM 视觉导航主循环：拍照 → 决策 → 执行 → 再拍照。

每一轮把当前画面和任务目标发给 VLM，模型通过 function calling 给出前进/后退
或旋转动作；动作执行完再拍一张新照片进入下一轮，如此循环。

本文件只做编排，实际的图像采集、模型调用和运动执行分别由注入的
`camera` / `client` / `executor` 完成，所以可以用假对象在无硬件、无网络的情况下
完整测试循环逻辑（见 tests/test_navigation_loop.py）。

上下文管理：历史里最多保留 `max_context_images` 张图片，超出时从最早的图片消息
开始丢弃。只丢 `user` 角色的图片消息，assistant/tool 消息的配对关系不受影响，
满足接口对 tool_calls 与 tool 结果必须成对出现的要求。
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

import vlm_tools
from vlm_client import VlmResponse, assistant_message, tool_result_message, user_image_message

DEFAULT_GOAL = "向前行进并探索周围环境"

# 系统提示词：说明身份、工具语义和每次动作的粒度。
SYSTEM_PROMPT = """你是一台陆空两栖机器车的视觉导航大脑，现在只负责地面行驶。

你会周期性收到车辆正前方摄像头拍到的彩色照片，需要判断当前环境并决定下一步动作。

可用动作（通过工具调用下发，每次只调用一个）：
- run_forward(seconds, speed)：沿车头方向直行。speed 为正前进、为负后退，取值 -1.0~1.0。
- rotate(seconds, speed)：原地旋转。speed 为正逆时针（左转）、为负顺时针（右转），取值 -1.0~1.0。
- stop()：立即停车。

工作方式与要求：
1. 默认目标是向前行进并探索周围环境；在没有障碍时优先前进。
2. 每次动作的 seconds 保持较短（0.2~2.0 秒），因为动作结束后你会收到一张新照片，可以据此继续调整。
3. 前方有障碍或通道狭窄时，用较短的 rotate 调整朝向，或先 run_forward 后退一点拉开距离，再重新观察。
4. 画面里如果无法判断是否安全，宁可调用 stop 或小幅前进，也不要一次给很长的动作。
5. 每次回复只下发一个动作，动作会自动在时长结束后停车；不要为了"停车"而额外重复调用 stop。
6. 请用中文简单说明你看到的情况和这样做的理由。
"""


class VlmNavigationLoop:
    """把相机、VLM 客户端和运动执行器串成一个闭环。"""

    def __init__(
        self,
        camera: Any,
        client: Any,
        executor: Any,
        stop_event: Optional[object] = None,
        goal: str = DEFAULT_GOAL,
        on_status: Optional[Callable[[dict[str, Any]], None]] = None,
        max_context_images: int = 2,
        max_steps: int = 40,
        max_consecutive_text_only: int = 3,
        max_api_failures: int = 3,
        request_interval_seconds: float = 0.2,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self.camera = camera
        self.client = client
        self.executor = executor
        self.goal = goal or DEFAULT_GOAL
        self._stop_event = stop_event
        self._on_status = on_status
        self.max_context_images = max(1, int(max_context_images))
        self.max_steps = max(1, int(max_steps))
        self.max_consecutive_text_only = max(1, int(max_consecutive_text_only))
        self.max_api_failures = max(1, int(max_api_failures))
        self.request_interval_seconds = max(0.0, float(request_interval_seconds))
        self._sleeper = sleeper

        self._history: list[dict[str, Any]] = []
        self._consecutive_text_only = 0
        self._api_failures = 0
        self._fatal: Optional[str] = None
        self._status: dict[str, Any] = {
            "step": 0,
            "action": "",
            "command": "S",
            "reason": "尚未开始",
            "text": "",
            "error": None,
        }

    # ---------- 对外接口 ----------

    def status(self) -> dict[str, Any]:
        """返回当前状态快照。"""

        return dict(self._status)

    def run(self) -> dict[str, Any]:
        """执行主循环，直到停止请求、步数上限或不可恢复错误；返回最终状态。"""

        self._reset()
        try:
            while self._can_continue():
                if not self._capture_observation():
                    break
                response = self._request()
                if response is None:
                    # 单次失败已在 _request 内退避处理，继续下一轮。
                    continue
                self._history.append(assistant_message(response))
                if not response.tool_calls:
                    self._handle_text_only(response)
                    continue
                self._consecutive_text_only = 0
                self._execute_tool_calls(response)
                self._status["step"] += 1
                if self._can_continue():
                    self._sleeper(self.request_interval_seconds)
        finally:
            # 无论循环因为什么结束，都必须让车停下来。
            self.executor.stop()
        self._finalize_reason()
        self._emit_status()
        return dict(self._status)

    # ---------- 循环内部步骤 ----------

    def _reset(self) -> None:
        self._history = [{"role": "system", "content": self._system_prompt()}]
        self._consecutive_text_only = 0
        self._api_failures = 0
        self._fatal = None
        self._status.update({"step": 0, "action": "", "command": "S", "reason": "正在启动", "text": "", "error": None})
        self._emit_status()

    def _system_prompt(self) -> str:
        return f"{SYSTEM_PROMPT}\n本次任务目标：{self.goal}。"

    def _stopped(self) -> bool:
        return self._stop_event is not None and self._stop_event.is_set()

    def _can_continue(self) -> bool:
        return not self._stopped() and self._fatal is None and self._status["step"] < self.max_steps

    def _capture_observation(self) -> bool:
        """拍一张照片并作为新的观察追加到历史。"""

        try:
            jpeg = self.camera.capture_jpeg()
        except Exception as exc:
            self._fatal = f"相机读取失败，已停车：{exc}"
            self._status["error"] = self._fatal
            return False
        text = (
            f"这是地面车摄像头当前画面（即将执行第 {self._status['step'] + 1} 个动作）。"
            f"任务目标：{self.goal}。请判断环境并决定下一步动作。"
        )
        self._history.append(user_image_message(jpeg, text))
        # 在发起请求之前就把图片数量收拢到上限，否则提示词里会多带一张图片。
        self._trim_history()
        self._emit_status()
        return True

    def _request(self) -> Optional[VlmResponse]:
        """调用 VLM；失败时记录并退避，超过上限则置为致命错误。"""

        try:
            response = self.client.chat(self._history, tools=vlm_tools.TOOL_DEFINITIONS)
            self._api_failures = 0
            # 成功一次就清掉上一次的临时错误，避免网页一直显示已经恢复的旧故障。
            self._status["error"] = None
            return response
        except Exception as exc:
            self._api_failures += 1
            message = f"VLM 调用失败（第 {self._api_failures} 次）：{exc}"
            self._status["error"] = message
            self._status["reason"] = message
            self._emit_status()
            # 调用失败时先确保停车，避免车辆保持在上一次动作中。
            self.executor.stop()
            if self._api_failures >= self.max_api_failures:
                self._fatal = f"VLM 连续 {self._api_failures} 次调用失败，已停车退出"
            else:
                self._sleeper(self.request_interval_seconds * 2)
            return None

    def _handle_text_only(self, response: VlmResponse) -> None:
        """模型只给文字、没有动作时的处理。"""

        self._consecutive_text_only += 1
        self._status["text"] = response.content
        self._status["action"] = "观察"
        self._status["command"] = "S"
        self._status["reason"] = "模型只给出文字，没有调用工具"
        self._emit_status()
        if self._consecutive_text_only >= self.max_consecutive_text_only:
            self._fatal = f"模型连续 {self._consecutive_text_only} 次没有调用工具，停止以免空转"
        else:
            self._sleeper(self.request_interval_seconds)

    def _execute_tool_calls(self, response: VlmResponse) -> None:
        """依次执行模型请求的工具调用，并把结果写回历史。"""

        for call in response.tool_calls:
            if self._stopped():
                break
            result = vlm_tools.dispatch_tool(call.name, call.arguments, self.executor)
            self._history.append(tool_result_message(call.id, result))
            self._status["action"] = call.name
            self._status["command"] = getattr(self.executor, "last_command", "S")
            self._status["reason"] = result
            self._emit_status()

    def _trim_history(self) -> None:
        """只保留最近 max_context_images 张图片，控制上下文长度。"""

        image_indices = [
            index
            for index, message in enumerate(self._history)
            if message.get("role") == "user" and isinstance(message.get("content"), list)
        ]
        excess = len(image_indices) - self.max_context_images
        if excess <= 0:
            return
        drop = set(image_indices[:excess])
        self._history = [message for index, message in enumerate(self._history) if index not in drop]

    def _finalize_reason(self) -> None:
        """给出循环结束原因的最终说明。"""

        if self._fatal:
            self._status["error"] = self._status["error"] or self._fatal
            self._status["reason"] = self._fatal
        elif self._stopped():
            self._status["reason"] = "已收到停止请求，停车退出"
        elif self._status["step"] >= self.max_steps:
            self._status["reason"] = f"达到最大步数 {self.max_steps}，停车退出"

    def _emit_status(self) -> None:
        if self._on_status is not None:
            self._on_status(dict(self._status))
