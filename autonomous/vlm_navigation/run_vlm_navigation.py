#!/usr/bin/env python3
"""VLM 视觉导航独立入口。

默认 **dry-run**：只打印下发指令，不驱动真实小车。只有显式传入 `--hardware`
才会打开 ESP32 串口。

用 `--fake-vlm` 可以绕过真实模型，按固定脚本跑通「拍照 → 决策 → 执行 → 再拍照」
的整条链路，不消耗 API 费用，适合先在机器人上确认相机和串口是否通。
"""

from __future__ import annotations

import argparse
import threading
from typing import Any, Optional

import vlm_config
import vlm_tools
from ground_link import create_ground_controller
from motion import MotionExecutor
from navigation_loop import DEFAULT_GOAL, VlmNavigationLoop
from realsense_color_source import RealSenseColorSource
from vlm_client import ToolCall, VlmClient, VlmResponse

# 假模型的动作脚本，用于 --fake-vlm：前进 → 右转 → 前进 → 停车。
FAKE_SCRIPT: list[tuple[str, dict[str, float]]] = [
    ("run_forward", {"seconds": 1.0, "speed": 0.6}),
    ("rotate", {"seconds": 0.5, "speed": -0.6}),
    ("run_forward", {"seconds": 1.2, "speed": 0.6}),
    ("stop", {}),
]


class FakeVlmClient:
    """脚本化假模型：按固定序列返回工具调用，用于零成本链路验证。"""

    def __init__(self, script: Optional[list[tuple[str, dict[str, float]]]] = None) -> None:
        self._script = list(script if script is not None else FAKE_SCRIPT)
        self._index = 0

    def chat(self, messages: list[dict[str, Any]], tools: Optional[list[dict[str, Any]]] = None) -> VlmResponse:
        if self._index >= len(self._script):
            return VlmResponse(content="假模型脚本已执行完毕。")
        name, arguments = self._script[self._index]
        self._index += 1
        return VlmResponse(
            content=f"假模型：执行第 {self._index} 个动作 {name}。",
            tool_calls=(ToolCall(id=f"fake_{self._index}", name=name, arguments=dict(arguments)),),
        )


def build_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""

    camera = vlm_config.CameraConfig()
    motion = vlm_config.MotionConfig()
    vlm = vlm_config.VlmConfig()
    parser = argparse.ArgumentParser(description="CONTROL VLM 视觉导航（默认 dry-run）")
    parser.add_argument("--hardware", action="store_true", help="打开 ESP32 串口并真实驱动小车")
    parser.add_argument("--esp32-port", default="/dev/ttyUSB0", help="ESP32 串口设备")
    parser.add_argument("--ground-baud", type=int, default=115200, help="ESP32 波特率")
    parser.add_argument("--fake-vlm", action="store_true", help="使用脚本化假模型，不调用真实 API")
    parser.add_argument("--goal", default=vlm.goal, help=f"任务目标，默认：{vlm.goal}")
    parser.add_argument("--model", default=None, help="覆盖模型名（默认读 .env 或内置默认值）")
    parser.add_argument("--base-url", default=None, help="覆盖接口地址（默认读 .env 或内置默认值）")
    parser.add_argument("--max-steps", type=int, default=vlm.max_steps, help="最大决策步数")
    parser.add_argument("--max-action-seconds", type=float, default=motion.max_action_seconds, help="单次动作最长时长（秒）")
    parser.add_argument("--max-speed", type=float, default=motion.max_speed, help="归一化速度上限")
    parser.add_argument("--jpeg-quality", type=int, default=camera.jpeg_quality, help="上传图片的 JPEG 质量")
    parser.add_argument("--width", type=int, default=camera.width, help="彩色流宽度")
    parser.add_argument("--height", type=int, default=camera.height, help="彩色流高度")
    parser.add_argument("--fps", type=int, default=camera.fps, help="彩色流帧率")
    return parser


def build_client(args: argparse.Namespace) -> Any:
    """根据参数构造 VLM 客户端（假模型或真实客户端）。"""

    if args.fake_vlm:
        return FakeVlmClient()
    settings = vlm_config.load_vlm_settings()
    return VlmClient(
        base_url=args.base_url or settings["base_url"],
        api_key=settings["api_key"],
        model=args.model or settings["model"],
    )


def main() -> None:
    """程序入口。"""

    args = build_parser().parse_args()
    vlm = vlm_config.VlmConfig()

    camera = RealSenseColorSource(
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=args.jpeg_quality,
    )
    driver = create_ground_controller(args.esp32_port, args.ground_baud, dry_run=not args.hardware)
    stop_event = threading.Event()

    # 指令由执行器内部的 last_command 记录；这里不需要额外回调。
    executor = MotionExecutor(
        driver,
        max_action_seconds=args.max_action_seconds,
        max_speed=args.max_speed,
        stop_event=stop_event,
    )

    # 假模型脚本长度决定步数上限，保证脚本跑完即干净退出。
    max_steps = min(args.max_steps, len(FAKE_SCRIPT)) if args.fake_vlm else args.max_steps

    def on_status(status: dict[str, Any]) -> None:
        print(
            f"[VLMNAV] step={status['step']} action={status['action'] or '-'} "
            f"command={status['command']} reason={status['reason']}",
            flush=True,
        )

    loop = VlmNavigationLoop(
        camera=camera,
        client=build_client(args),
        executor=executor,
        stop_event=stop_event,
        goal=args.goal or vlm.goal,
        on_status=on_status,
        max_context_images=vlm.max_context_images,
        max_steps=max_steps,
        max_consecutive_text_only=vlm.max_consecutive_text_only,
        max_api_failures=vlm.max_api_failures,
        request_interval_seconds=vlm.request_interval_seconds,
    )

    print(f"[VLMNAV] 模式={'HARDWARE' if args.hardware else 'DRY-RUN'}，目标：{args.goal or vlm.goal}", flush=True)
    if args.fake_vlm:
        print("[VLMNAV] 使用脚本化假模型，不调用真实 API", flush=True)

    try:
        driver.connect()
        camera.start()
        result = loop.run()
    except KeyboardInterrupt:
        stop_event.set()
        result = loop.status()
        print("[VLMNAV] 收到停止信号", flush=True)
    finally:
        # 先释放相机，再关闭串口（close 内部会再发一次停车）。
        camera.close()
        driver.close()

    print(
        f"[VLMNAV] 结束：step={result.get('step')} command={result.get('command')} "
        f"reason={result.get('reason')} error={result.get('error')}",
        flush=True,
    )


if __name__ == "__main__":
    main()
