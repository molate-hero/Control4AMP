"""OpenAI 兼容接口的 VLM 客户端。

只依赖 Python 标准库 `urllib` 和 `json`，不依赖任何厂商 SDK，也不依赖
`requests`。这样做的原因：机器人上的虚拟环境只保证有 Flask / pyserial /
pymavlink，`requests` 不一定装了，而 urllib 一定可用；换服务商时也只是改
`base_url` 和 `model`。

网络出口通过 `opener` 注入，测试时可传入假的 opener，不发真实请求。
"""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

# 一次 HTTP POST 的函数签名：(url, headers, body_bytes, timeout) -> (status, body_bytes)。
Opener = Callable[[str, dict[str, str], bytes, float], "tuple[int, bytes]"]


@dataclass(frozen=True)
class ToolCall:
    """模型请求的一次工具调用。"""

    id: str
    name: str
    arguments: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class VlmResponse:
    """一次对话回复：文字内容 + 可选的工具调用。"""

    content: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    raw: dict[str, Any] = field(default_factory=dict)


def default_opener(url: str, headers: dict[str, str], body: bytes, timeout: float) -> "tuple[int, bytes]":
    """用标准库发起一次 HTTP POST，并把 HTTP 错误也转成返回值。"""

    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return int(response.status), response.read()
    except urllib.error.HTTPError as exc:
        # 4xx/5xx 也带上响应体，便于把服务端错误信息原样报出来。
        return int(exc.code), exc.read()
    except urllib.error.URLError as exc:
        raise RuntimeError(f"VLM 网络请求失败：{exc.reason}") from exc


def encode_image_data_url(jpeg_bytes: bytes) -> str:
    """把 JPEG 字节编码成 data URL，供 image_url 字段使用。"""

    return "data:image/jpeg;base64," + base64.b64encode(jpeg_bytes).decode("ascii")


def user_image_message(jpeg_bytes: bytes, text: str) -> dict[str, Any]:
    """构造带图片的 user 消息。"""

    return {
        "role": "user",
        "content": [
            {"type": "text", "text": text},
            {"type": "image_url", "image_url": {"url": encode_image_data_url(jpeg_bytes)}},
        ],
    }


def assistant_message(response: VlmResponse) -> dict[str, Any]:
    """把回复序列化成可写回历史的 assistant 消息。"""

    message: dict[str, Any] = {"role": "assistant", "content": response.content or ""}
    if response.tool_calls:
        message["tool_calls"] = [
            {
                "id": call.id,
                "type": "function",
                "function": {
                    "name": call.name,
                    "arguments": json.dumps(call.arguments, ensure_ascii=False),
                },
            }
            for call in response.tool_calls
        ]
    return message


def tool_result_message(tool_call_id: str, content: str) -> dict[str, Any]:
    """构造工具执行结果消息。"""

    return {"role": "tool", "tool_call_id": tool_call_id, "content": content}


def _short(text: str, limit: int = 300) -> str:
    """截断过长的错误文本，避免刷屏。"""

    text = (text or "").strip()
    return text if len(text) <= limit else text[:limit] + "..."


def _content_to_text(content: Any) -> str:
    """把 message.content 统一转成纯文本（模型偶尔返回分段列表）。"""

    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(str(item.get("text", "")))
        return "\n".join(part for part in parts if part)
    return "" if content is None else str(content)


def _parse_arguments(arguments: Any) -> dict[str, Any]:
    """解析工具参数；模型有时给 JSON 字符串，有时直接给对象。"""

    if isinstance(arguments, dict):
        return arguments
    if not arguments:
        return {}
    try:
        parsed = json.loads(arguments)
    except (TypeError, ValueError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


def parse_response(payload: dict[str, Any]) -> VlmResponse:
    """从 OpenAI 兼容的响应体里取出文字和工具调用。"""

    choices = payload.get("choices") or []
    if not choices:
        raise RuntimeError("VLM 响应缺少 choices 字段")
    message = choices[0].get("message") or {}
    tool_calls = []
    for index, item in enumerate(message.get("tool_calls") or []):
        function = item.get("function") or {}
        tool_calls.append(
            ToolCall(
                id=str(item.get("id") or f"call_{index}"),
                name=str(function.get("name") or ""),
                arguments=_parse_arguments(function.get("arguments")),
            )
        )
    return VlmResponse(
        content=_content_to_text(message.get("content")),
        tool_calls=tuple(tool_calls),
        raw=payload,
    )


class VlmClient:
    """调用 `{base_url}/chat/completions` 的轻量客户端。"""

    def __init__(
        self,
        base_url: str,
        api_key: Optional[str],
        model: str,
        timeout_seconds: float = 60.0,
        temperature: float = 0.0,
        opener: Optional[Opener] = None,
    ) -> None:
        self.base_url = (base_url or "").rstrip("/")
        self.api_key = api_key
        self.model = model
        self.timeout_seconds = float(timeout_seconds)
        self.temperature = float(temperature)
        self._opener: Opener = opener or default_opener

    @property
    def endpoint(self) -> str:
        """完整的对话补全地址。"""

        return f"{self.base_url}/chat/completions"

    def chat(self, messages: Sequence[dict[str, Any]], tools: Optional[list[dict[str, Any]]] = None) -> VlmResponse:
        """发送一轮对话，返回文字与工具调用。"""

        if not self.api_key:
            raise RuntimeError("缺少 API key，请在 .env 或环境变量中设置 DEEPSEEK_API_KEY")

        body: dict[str, Any] = {
            "model": self.model,
            "messages": list(messages),
            "temperature": self.temperature,
        }
        if tools:
            body["tools"] = tools
            body["tool_choice"] = "auto"

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        payload_bytes = json.dumps(body, ensure_ascii=False).encode("utf-8")
        status, raw = self._opener(self.endpoint, headers, payload_bytes, self.timeout_seconds)
        text = raw.decode("utf-8", errors="replace") if isinstance(raw, (bytes, bytearray)) else str(raw)

        if status >= 400:
            raise RuntimeError(f"VLM 请求失败（HTTP {status}）：{_short(text)}")
        try:
            payload = json.loads(text)
        except ValueError as exc:
            raise RuntimeError(f"VLM 响应不是合法 JSON：{_short(text)}") from exc
        return parse_response(payload)
