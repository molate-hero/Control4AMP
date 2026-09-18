"""VLM 客户端测试：不发真实网络请求，用假 opener 验证请求与解析。

假 opener 的签名与 `vlm_client.default_opener` 一致，测试里同时捕获请求内容，
因此既能验证"发出去什么"，也能验证"怎么解析回来的"。
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import vlm_client
from vlm_client import ToolCall, VlmClient, VlmResponse


class CapturingOpener:
    """记录请求并返回预设响应的假 opener。"""

    def __init__(self, status: int = 200, payload: object = None, raw: bytes | None = None) -> None:
        self.status = status
        self.payload = payload
        self.raw = raw
        self.requests: list[dict] = []

    def __call__(self, url: str, headers: dict, body: bytes, timeout: float):
        self.requests.append({"url": url, "headers": headers, "body": json.loads(body.decode("utf-8")), "timeout": timeout})
        if self.raw is not None:
            return self.status, self.raw
        return self.status, json.dumps(self.payload or {}).encode("utf-8")


def tool_call_payload() -> dict:
    """构造一个带工具调用的正常响应体。"""

    return {
        "choices": [
            {
                "message": {
                    "content": "前方开阔，向前走一小段。",
                    "tool_calls": [
                        {
                            "id": "call_1",
                            "type": "function",
                            "function": {"name": "run_forward", "arguments": '{"seconds": 1.0, "speed": 0.6}'},
                        }
                    ],
                }
            }
        ]
    }


class VlmClientTest(unittest.TestCase):
    """验证请求构造、鉴权头和响应解析。"""

    def test_endpoint_is_joined_correctly(self) -> None:
        client = VlmClient("https://api.example.com/v1/", "k", "m")

        self.assertEqual(client.endpoint, "https://api.example.com/v1/chat/completions")

    def test_missing_api_key_raises(self) -> None:
        client = VlmClient("https://api.example.com/v1", None, "m")

        with self.assertRaises(RuntimeError):
            client.chat([{"role": "user", "content": "hi"}])

    def test_chat_sends_auth_tools_and_parses_tool_call(self) -> None:
        opener = CapturingOpener(payload=tool_call_payload())
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        response = client.chat([{"role": "user", "content": "看画面"}], tools=[{"type": "function"}])

        request = opener.requests[0]
        self.assertEqual(request["headers"]["Authorization"], "Bearer secret")
        self.assertEqual(request["body"]["model"], "vlm-1")
        self.assertEqual(request["body"]["tool_choice"], "auto")
        self.assertEqual(request["body"]["tools"], [{"type": "function"}])

        self.assertEqual(len(response.tool_calls), 1)
        call = response.tool_calls[0]
        self.assertEqual(call.id, "call_1")
        self.assertEqual(call.name, "run_forward")
        self.assertEqual(call.arguments, {"seconds": 1.0, "speed": 0.6})
        self.assertIn("前方开阔", response.content)

    def test_no_tools_means_no_tool_choice(self) -> None:
        opener = CapturingOpener(payload={"choices": [{"message": {"content": "ok"}}]})
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        client.chat([{"role": "user", "content": "hi"}])

        self.assertNotIn("tools", opener.requests[0]["body"])
        self.assertNotIn("tool_choice", opener.requests[0]["body"])

    def test_http_error_raises_with_status(self) -> None:
        opener = CapturingOpener(status=500, raw=b'{"error": "boom"}')
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        with self.assertRaises(RuntimeError) as context:
            client.chat([{"role": "user", "content": "hi"}])

        self.assertIn("500", str(context.exception))

    def test_invalid_json_raises(self) -> None:
        opener = CapturingOpener(status=200, raw=b"not json")
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        with self.assertRaises(RuntimeError):
            client.chat([{"role": "user", "content": "hi"}])

    def test_arguments_sent_as_object_are_accepted(self) -> None:
        # 有些服务直接返回对象而不是 JSON 字符串。
        payload = {"choices": [{"message": {"tool_calls": [{"id": "c", "function": {"name": "rotate", "arguments": {"seconds": 0.5, "speed": -0.5}}}]}}]}
        opener = CapturingOpener(payload=payload)
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        response = client.chat([{"role": "user", "content": "hi"}])

        self.assertEqual(response.tool_calls[0].arguments, {"seconds": 0.5, "speed": -0.5})

    def test_malformed_arguments_become_empty_dict(self) -> None:
        payload = {"choices": [{"message": {"tool_calls": [{"id": "c", "function": {"name": "stop", "arguments": "{不是 JSON"}}]}}]}
        opener = CapturingOpener(payload=payload)
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        response = client.chat([{"role": "user", "content": "hi"}])

        self.assertEqual(response.tool_calls[0].arguments, {})

    def test_missing_choices_raises(self) -> None:
        opener = CapturingOpener(payload={"error": "no choice"})
        client = VlmClient("https://api.example.com/v1", "secret", "vlm-1", opener=opener)

        with self.assertRaises(RuntimeError):
            client.chat([{"role": "user", "content": "hi"}])


class MessageBuilderTest(unittest.TestCase):
    """验证消息构造函数的输出结构。"""

    def test_image_message_uses_jpeg_data_url(self) -> None:
        message = vlm_client.user_image_message(b"\xff\xd8\xff", "看这张图")

        self.assertEqual(message["role"], "user")
        text_part, image_part = message["content"]
        self.assertEqual(text_part, {"type": "text", "text": "看这张图"})
        self.assertTrue(image_part["image_url"]["url"].startswith("data:image/jpeg;base64,"))

    def test_assistant_message_serializes_tool_calls(self) -> None:
        response = VlmResponse(
            content="前进",
            tool_calls=(ToolCall(id="call_9", name="run_forward", arguments={"seconds": 1.0, "speed": 0.6}),),
        )

        message = vlm_client.assistant_message(response)

        self.assertEqual(message["role"], "assistant")
        call = message["tool_calls"][0]
        self.assertEqual(call["id"], "call_9")
        self.assertEqual(call["function"]["name"], "run_forward")
        # 参数必须被序列化成 JSON 字符串，符合接口要求。
        self.assertEqual(json.loads(call["function"]["arguments"]), {"seconds": 1.0, "speed": 0.6})

    def test_assistant_message_without_tools_has_no_tool_calls_key(self) -> None:
        message = vlm_client.assistant_message(VlmResponse(content="只是在观察"))

        self.assertNotIn("tool_calls", message)

    def test_tool_result_message_shape(self) -> None:
        message = vlm_client.tool_result_message("call_1", "已执行")

        self.assertEqual(message, {"role": "tool", "tool_call_id": "call_1", "content": "已执行"})


if __name__ == "__main__":
    unittest.main()
