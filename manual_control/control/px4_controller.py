"""PX4 MAVLink 通信和飞行状态监视模块。

手动飞行时，真正的飞行摇杆控制由遥控器接收机和 PX4 完成；本模块负责：
1. 接收 PX4 心跳、RC、姿态、高度和电池状态；
2. 为地面控制提供“飞控是否解锁”和 RC 通道数据；
3. 提供显式调用的模式切换、解锁、上锁和紧急降落接口。

本模块不会自动解锁、自动起飞，也不会在程序启动时主动改变飞行模式。
"""

from __future__ import annotations

import time
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Optional

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - 便于没有依赖的静态检查
    mavutil = None


@dataclass
class Px4Snapshot:
    """当前已收到的 PX4 状态快照。"""

    connected: bool = False
    armed: bool = False
    mode_name: str = "UNKNOWN"
    rc_channels: dict[int, int] = field(default_factory=dict)
    attitude: Optional[tuple[float, float, float]] = None
    relative_altitude: Optional[float] = None
    vertical_speed: Optional[float] = None
    battery_percent: Optional[int] = None
    battery_voltage: Optional[float] = None
    attitude_target_thrust: Optional[float] = None
    last_heartbeat: float = 0.0
    last_rc: float = 0.0


class Px4Controller:
    """单线程读取 PX4 MAVLink 数据，避免多个线程争抢串口。"""

    def __init__(self, device: str, baud: int) -> None:
        self.device = device
        self.baud = baud
        self._mav: Any = None
        # 串口读写可能分别来自遥测线程和网页请求线程，必须统一加锁。
        self._io_lock = threading.RLock()
        self.snapshot = Px4Snapshot()
        self._z0: Optional[float] = None

    def connect(self) -> None:
        """打开 MAVLink 串口并等待飞控心跳。"""

        if mavutil is None:
            raise RuntimeError("缺少 pymavlink，请安装 requirements.txt 中的依赖")

        print(f"[PX4] 正在连接：{self.device} @ {self.baud}")
        self._mav = mavutil.mavlink_connection(
            self.device,
            baud=self.baud,
            source_system=191,
        )
        heartbeat = self._mav.wait_heartbeat(timeout=10)
        if heartbeat is None:
            self._mav.close()
            self._mav = None
            raise TimeoutError("10 秒内没有收到 PX4 心跳")

        self.snapshot.connected = True
        self._handle_message(heartbeat)
        self._request_streams()
        print(
            f"[PX4] 已连接：mode={self.snapshot.mode_name}, "
            f"armed={self.snapshot.armed}"
        )

    def _request_streams(self) -> None:
        """请求手动控制所需的 MAVLink 消息频率。"""

        if self._mav is None:
            return

        requests = {
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: 20,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 2,
            # 回读 PX4 实际接受到的姿态/推力目标，便于诊断 Offboard 链路。
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET: 10,
        }
        for message_id, frequency in requests.items():
            try:
                self._mav.mav.command_long_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    int(1_000_000 / frequency),
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                print(f"[PX4] 请求消息流失败：{exc}")

    def poll(self, timeout: float) -> None:
        """读取一条 MAVLink 消息并更新快照。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")

        with self._io_lock:
            message = self._mav.recv_match(blocking=True, timeout=timeout)
            if message is not None:
                self._handle_message(message)

    def _handle_message(self, message: Any) -> None:
        """把单条 MAVLink 消息写入统一状态快照。"""

        message_type = message.get_type()
        now = time.monotonic()

        if message_type == "HEARTBEAT":
            self.snapshot.last_heartbeat = now
            self.snapshot.armed = bool(
                int(message.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.snapshot.mode_name = self._mode_name_from_heartbeat(message)
        elif message_type == "RC_CHANNELS":
            self.snapshot.last_rc = now
            self.snapshot.rc_channels = {
                1: int(message.chan1_raw),
                2: int(message.chan2_raw),
                7: int(message.chan7_raw),
                9: int(message.chan9_raw),
            }
        elif message_type == "ATTITUDE":
            self.snapshot.attitude = (message.roll, message.pitch, message.yaw)
        elif message_type == "LOCAL_POSITION_NED":
            if self._z0 is None:
                self._z0 = float(message.z)
            self.snapshot.relative_altitude = self._z0 - float(message.z)
            self.snapshot.vertical_speed = float(message.vz)
        elif message_type == "BATTERY_STATUS":
            remaining = int(message.battery_remaining)
            self.snapshot.battery_percent = remaining if remaining >= 0 else None
            voltage = message.voltages[0] if message.voltages else 0
            self.snapshot.battery_voltage = voltage / 1000.0 if voltage else None
        elif message_type == "ATTITUDE_TARGET":
            # 这是 PX4 输出的当前目标，不等同于电机最终 PWM，但可验证推力目标是否被接受。
            self.snapshot.attitude_target_thrust = float(message.thrust)

    def _mode_name_from_heartbeat(self, heartbeat: Any) -> str:
        """根据 PX4 自定义模式编号解析可读模式名。"""

        custom_mode = int(heartbeat.custom_mode)
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        mapping = self._mav.mode_mapping() if self._mav is not None else {}
        for name, (_, main, sub) in (mapping or {}).items():
            if int(main) == main_mode and int(sub) == sub_mode:
                return name
        return f"custom({main_mode},{sub_mode})"

    def heartbeat_age(self) -> float:
        """返回距离最近一次心跳的秒数。"""

        return time.monotonic() - self.snapshot.last_heartbeat

    def rc_age(self) -> float:
        """返回距离最近一次 RC 消息的秒数。"""

        return time.monotonic() - self.snapshot.last_rc

    def set_mode(self, mode_name: str, timeout: float = 5.0) -> bool:
        """显式请求切换 PX4 模式，不会被主循环自动调用。"""

        if self._mav is None:
            return False
        mapping = self._mav.mode_mapping() or {}
        mode = mapping.get(mode_name.upper())
        if mode is None:
            print(f"[PX4] 不支持模式：{mode_name}")
            return False

        base_mode, main_mode, sub_mode = (int(value) for value in mode)
        with self._io_lock:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                base_mode,
                main_mode,
                sub_mode,
                0,
                0,
                0,
                0,
            )

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll(0.1)
            if self.snapshot.mode_name.upper() == mode_name.upper():
                return True
        return False

    def arm(self) -> None:
        """显式发送解锁请求；手动控制主循环不会自动调用。"""

        self._send_arm_command(1)

    def disarm(self) -> None:
        """显式发送上锁请求；手动控制主循环不会自动调用。"""

        self._send_arm_command(0)

    def _send_arm_command(self, value: int) -> None:
        """发送 PX4 解锁/上锁命令。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")
        with self._io_lock:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                value,
                0,
                0,
                0,
                0,
                0,
                0,
            )

    def emergency_land(self) -> None:
        """显式发送紧急降落请求。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")
        with self._io_lock:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                0,
                0,
                float("nan"),
                float("nan"),
                float("nan"),
                0,
            )

    def send_attitude_setpoint(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        thrust: float,
    ) -> None:
        """发送一帧 OFFBOARD 姿态/油门目标。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")

        # 欧拉角转四元数，避免网络层直接接触 MAVLink 数据细节。
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        quaternion = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        )

        with self._io_lock:
            self._mav.mav.set_attitude_target_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self._mav.target_system,
                self._mav.target_component,
                mask,
                quaternion,
                0.0,
                0.0,
                0.0,
                max(0.0, min(0.72, float(thrust))),
            )

    def read_parameter(self, name: str, timeout: float = 3.0) -> float:
        """通过 MAVLink 读取一个 PX4 参数的当前值。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")

        parameter_id = name.encode("ascii")[:16]
        with self._io_lock:
            self._mav.mav.param_request_read_send(
                self._mav.target_system,
                self._mav.target_component,
                parameter_id,
                -1,
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                message = self._mav.recv_match(
                    type="PARAM_VALUE",
                    blocking=True,
                    timeout=min(0.2, max(0.0, deadline - time.monotonic())),
                )
                if message is None:
                    continue
                message_id = message.param_id
                if isinstance(message_id, bytes):
                    message_id = message_id.rstrip(b"\x00").decode("ascii", errors="ignore")
                if str(message_id).rstrip("\x00") == name:
                    return float(message.param_value)
        raise TimeoutError(f"读取 PX4 参数超时：{name}")

    def write_parameter(self, name: str, value: float, timeout: float = 3.0) -> float:
        """通过 MAVLink 写入一个 PX4 浮点参数并等待回读确认。"""

        if self._mav is None:
            raise RuntimeError("PX4 尚未连接")

        parameter_id = name.encode("ascii")[:16]
        with self._io_lock:
            self._mav.mav.param_set_send(
                self._mav.target_system,
                self._mav.target_component,
                parameter_id,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                message = self._mav.recv_match(
                    type="PARAM_VALUE",
                    blocking=True,
                    timeout=min(0.2, max(0.0, deadline - time.monotonic())),
                )
                if message is None:
                    continue
                message_id = message.param_id
                if isinstance(message_id, bytes):
                    message_id = message_id.rstrip(b"\x00").decode("ascii", errors="ignore")
                if str(message_id).rstrip("\x00") == name:
                    return float(message.param_value)
        raise TimeoutError(f"写入 PX4 参数未确认：{name}")

    def close(self) -> None:
        """关闭 PX4 串口。"""

        if self._mav is not None:
            self._mav.close()
            self._mav = None
        self.snapshot.connected = False
