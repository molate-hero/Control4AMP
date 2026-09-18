"""地面车串口连接：复用 manual_control 中已验证的地面部分。

本模块只导入 `manual_control` 的地面串口控制器和差速混合函数，**不导入任何
PX4 / 飞行相关代码**，因此不会给本模块带来 pymavlink 依赖，也不涉及飞控动作。

路径注入方式与 `network_control/server.py` 保持一致：把 `manual_control` 目录
插入 `sys.path`，再以顶层包名 `control.*` 导入。
"""

from __future__ import annotations

import sys

from vlm_config import MANUAL_CONTROL_ROOT

if str(MANUAL_CONTROL_ROOT) not in sys.path:
    sys.path.insert(0, str(MANUAL_CONTROL_ROOT))

from control.channels import build_axis_command
from control.ground_controller import GroundController

__all__ = ["GroundController", "build_axis_command", "create_ground_controller"]


def create_ground_controller(port: str, baud: int, dry_run: bool) -> GroundController:
    """创建地面车串口控制器；dry_run 为真时不打开串口，只打印指令。"""

    return GroundController(port, baud, dry_run=dry_run)
