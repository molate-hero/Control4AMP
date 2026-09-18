"""VLM 视觉导航模块的默认参数与凭据读取。

集中存放相机、运动限幅和 VLM 接口的默认值，并提供一个极简的 .env 解析器。
刻意不引入 python-dotenv：仓库其他模块也没有这个依赖，为一个"键=值"文件
多装一个包不值得。

本文件不命名为 `config.py`，是因为 `manual_control/config` 是一个包，而本模块
会把 `manual_control` 目录注入 `sys.path` 以复用地面串口代码；两个同名 `config`
同时可导入会互相遮蔽。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# 目录约定：本模块根目录、CONTROL 根目录、manual_control 根目录。
MODULE_ROOT = Path(__file__).resolve().parent
CONTROL_ROOT = MODULE_ROOT.parents[1]
MANUAL_CONTROL_ROOT = CONTROL_ROOT / "manual_control"

# .env 查找顺序：优先本模块目录，其次 CONTROL 根目录；先找到的同名键优先。
DEFAULT_ENV_PATHS = (MODULE_ROOT / ".env", CONTROL_ROOT / ".env")


@dataclass(frozen=True)
class CameraConfig:
    """RealSense 彩色流的默认参数。"""

    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 80
    frame_timeout_ms: int = 3000


@dataclass(frozen=True)
class MotionConfig:
    """运动执行的默认限幅。"""

    # 指令重发周期必须明显小于 network_control 的 0.5 秒地面看门狗。
    tick_seconds: float = 0.05
    # 单次动作的最长时长，防止模型给出过长的运动。
    max_action_seconds: float = 2.0
    # 归一化速度上限。
    max_speed: float = 1.0


@dataclass(frozen=True)
class VlmConfig:
    """VLM 接口与主循环的默认参数。"""

    base_url: str = "https://api.deepseek.com/v1"
    model: str = "deepseek-v4.1-flash"
    timeout_seconds: float = 60.0
    goal: str = "向前行进并探索周围环境"
    # 上下文中最多保留几张历史图片，控制 token 与费用。
    max_context_images: int = 2
    max_steps: int = 40
    # 模型连续多次只回文字却不调用工具时，视为空转并停止。
    max_consecutive_text_only: int = 3
    # VLM 连续调用失败次数上限。
    max_api_failures: int = 3
    request_interval_seconds: float = 0.2


def parse_env_text(text: str) -> dict[str, str]:
    """解析 KEY=VALUE 形式的 .env 文本，忽略注释与空行。"""

    values: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        # 兼容 `export KEY=VALUE` 写法。
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        key, separator, value = line.partition("=")
        if not separator:
            continue
        key = key.strip()
        value = value.strip()
        # 去掉成对的引号，值内部的空格保留。
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        if key:
            values[key] = value
    return values


def load_env(env_paths: tuple[Path, ...] = DEFAULT_ENV_PATHS) -> dict[str, str]:
    """读取所有存在的 .env 文件并合并；不存在的文件直接跳过。"""

    merged: dict[str, str] = {}
    for path in env_paths:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            # 没有 .env 是正常情况（例如仿真模式），不应报错。
            continue
        merged.update(parse_env_text(text))
    return merged


def read_setting(name: str, default: Optional[str] = None, env: Optional[dict[str, str]] = None) -> Optional[str]:
    """读取一个配置项：先看进程环境变量，再看 .env 文件。"""

    if env is None:
        env = load_env()
    value = os.environ.get(name) or env.get(name)
    return value if value not in (None, "") else default


def load_vlm_settings(env: Optional[dict[str, str]] = None) -> dict[str, Optional[str]]:
    """读出 VLM 连接设置，缺省值来自 VlmConfig。"""

    if env is None:
        env = load_env()
    # API key 不放进 dataclass，避免在日志或状态输出里被顺带打印。
    api_key = read_setting("DEEPSEEK_API_KEY", None, env) or read_setting("VLM_API_KEY", None, env)
    return {
        "api_key": api_key,
        "base_url": read_setting("DEEPSEEK_BASE_URL", VlmConfig.base_url, env),
        "model": read_setting("DEEPSEEK_MODEL", VlmConfig.model, env),
    }
