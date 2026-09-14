"""CONTROL 网络控制服务。

本服务是 CONTROL 项目中的独立网络控制模块，不会自动启动 manual_control，
也不会在导入模块时占用串口。默认仿真模式，只有显式传入 --hardware 才连接
PX4 和 ESP32，避免多个控制模块同时争用同一套硬件。
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, render_template, request

# 复用手动遥控模块中已经验证过的串口控制、差速计算和 PX4 通信基础类。
MODULE_ROOT = Path(__file__).resolve().parents[1] / "manual_control"
AUTONOMY_ROOT = Path(__file__).resolve().parents[1] / "autonomous" / "ground_avoidance"
import sys

if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))
if str(AUTONOMY_ROOT) not in sys.path:
    sys.path.insert(0, str(AUTONOMY_ROOT))

from control.channels import build_axis_command
from control.ground_controller import GroundController
from control.px4_controller import Px4Controller


BASE_DIR = Path(__file__).resolve().parent
app = Flask(
    __name__,
    template_folder=str(BASE_DIR / "templates"),
    static_folder=str(BASE_DIR / "static"),
)


class NetworkController:
    """管理网络控制的硬件连接、网页状态和安全循环。"""

    GROUND_COMMAND_TIMEOUT = 0.5
    FLIGHT_SETPOINT_TIMEOUT = 0.5
    HEARTBEAT_TIMEOUT = 2.0
    MAX_RELATIVE_ALTITUDE = 0.60
    MAX_FLIGHT_TIME = 15.0
    PREFLIGHT_DISARM_PARAMETER = "COM_DISARM_PRFLT"

    def __init__(
        self,
        hardware: bool,
        ground_port: str,
        ground_baud: int,
        px4_device: str,
        px4_baud: int,
    ) -> None:
        self.hardware = hardware
        self.ground = GroundController(ground_port, ground_baud, dry_run=not hardware)
        self.px4 = Px4Controller(px4_device, px4_baud) if hardware else None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._ground_last_input = time.monotonic()
        self._ground_sent_count = 0
        self._flight_setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0}
        self._flight_last_input = time.monotonic()
        # 只有网页明确打开网络飞行控制后，才允许建立 Offboard setpoint 流。
        self._flight_control_enabled = False
        self._last_flight_setpoint_sent = 0.0
        self._flight_setpoint_sent_count = 0
        self._last_flight_sent_thrust: float | None = None
        self._last_logged_input_thrust: float | None = None
        self._last_logged_armed: bool | None = None
        self._last_logged_mode: str | None = None
        self._flight_started_at: float | None = None
        self._flight_thread: threading.Thread | None = None
        self._ground_safety_thread: threading.Thread | None = None
        self._last_flight_safety_action = 0.0
        self._preflight_disarm_original: float | None = None
        self._preflight_disarm_value: float | None = None
        self._preflight_disarm_disabled = False
        # 自主运行状态独立于手动/网络地面输入；真实启动前才延迟导入相机依赖。
        self._autonomy_enabled = False
        self._autonomy_error: str | None = None
        self._autonomy_command = "S"
        self._autonomy_reason = "未启动"
        self._autonomy_thread: threading.Thread | None = None
        self._autonomy_stop_event = threading.Event()
        self._autonomy_source: Any = None
        self._depth_preview_was_running = False

    def start(self) -> None:
        """启动连接和后台安全线程。"""

        self.ground.connect()
        if not self.hardware or self.px4 is None:
            return

        self.px4.connect()
        try:
            # 启动时读取真实参数，避免页面只显示进程缓存而与 PX4 不一致。
            current_timeout = self.px4.read_parameter(self.PREFLIGHT_DISARM_PARAMETER)
            with self._lock:
                self._preflight_disarm_value = current_timeout
                self._preflight_disarm_disabled = current_timeout < 0
                if current_timeout >= 0:
                    self._preflight_disarm_original = current_timeout
        except Exception as exc:
            print(f"[NETWORK] 读取 PX4 自动上锁参数失败：{exc}", flush=True)
        self._flight_thread = threading.Thread(
            target=self._flight_loop,
            name="px4-network-loop",
            daemon=True,
        )
        self._flight_thread.start()
        self._ground_safety_thread = threading.Thread(
            target=self._ground_safety_loop,
            name="ground-safety-loop",
            daemon=True,
        )
        self._ground_safety_thread.start()

    def close(self) -> None:
        """停止线程，先停车，再关闭硬件连接。"""

        self.stop_autonomy()
        self._stop_event.set()
        try:
            self.ground.stop()
        finally:
            self.ground.close()
            if self.px4 is not None:
                self.px4.close()

    @staticmethod
    def _process_command_line(pid: int) -> str:
        """读取指定进程的命令行；进程退出时返回空字符串。"""

        try:
            raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, OSError):
            return ""
        return raw.replace(b"\x00", b" ").decode(errors="replace").strip()

    def _pause_depth_preview(self) -> None:
        """暂停项目自带的深度预览，释放 RealSense 给自主线程。"""

        if not self.hardware:
            return
        pids: list[int] = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            command = self._process_command_line(int(entry.name))
            if "/home/orangepi/run_depth.sh" in command or "/home/orangepi/depth_web_server.py" in command:
                pids.append(int(entry.name))
        self._depth_preview_was_running = bool(pids)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        if pids:
            time.sleep(0.8)
            print("[AUTONOMY] 已暂停深度预览，释放 RealSense", flush=True)

    def _resume_depth_preview(self) -> None:
        """只在自主启动前确实存在预览服务时恢复它。"""

        if not self.hardware or not self._depth_preview_was_running:
            return
        self._depth_preview_was_running = False
        script = Path("/home/orangepi/run_depth.sh")
        if not script.exists():
            return
        subprocess.Popen(
            [str(script)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        print("[AUTONOMY] 已恢复深度预览", flush=True)

    def start_autonomy(self) -> bool:
        """启动独立的地面合力法避障线程。"""

        if not self.hardware:
            raise RuntimeError("当前为仿真模式，不能启动真实自主运行")
        if self.px4 is not None and self.px4.snapshot.armed:
            raise RuntimeError("飞控已解锁，不能启动地面自主运行")
        with self._lock:
            if self._autonomy_enabled:
                return True

        try:
            from potential_field import PotentialFieldAvoider
            from realsense_source import RealSenseDepthSource

            self._pause_depth_preview()
            source = RealSenseDepthSource()
            source.start()
        except Exception as exc:
            self._resume_depth_preview()
            raise RuntimeError(f"RealSense 未能启动：{exc}") from exc

        with self._lock:
            self._autonomy_source = source
            self._autonomy_error = None
            self._autonomy_command = "S"
            self._autonomy_reason = "正在初始化"
            self._autonomy_stop_event.clear()
            self._autonomy_enabled = True
            self._autonomy_thread = threading.Thread(
                target=self._autonomy_loop,
                name="ground-autonomy-loop",
                daemon=True,
            )
            self._autonomy_thread.start()
        print("[AUTONOMY] 地面自主运行已启动", flush=True)
        return True

    def stop_autonomy(self) -> bool:
        """停止自主线程、发送停车并恢复此前占用的深度预览。"""

        with self._lock:
            thread = self._autonomy_thread
            was_enabled = self._autonomy_enabled or thread is not None
            self._autonomy_stop_event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=4.0)
        with self._lock:
            self._autonomy_thread = None
            self._autonomy_enabled = False
            self._autonomy_command = "S"
            self._autonomy_reason = "已停止"
        self.ground.stop()
        self._resume_depth_preview()
        if was_enabled:
            print("[AUTONOMY] 地面自主运行已停止", flush=True)
        return False

    def autonomy_status(self) -> dict[str, Any]:
        """返回网页显示所需的自主运行状态。"""

        with self._lock:
            return {
                "enabled": self._autonomy_enabled,
                "error": self._autonomy_error,
                "command": self._autonomy_command,
                "reason": self._autonomy_reason,
            }

    def _autonomy_loop(self) -> None:
        """以固定频率读取深度并把合力结果发送给现有 ESP32 串口。"""

        from esp32_output import build_drive_command
        from potential_field import PotentialFieldAvoider

        source = self._autonomy_source
        avoider = PotentialFieldAvoider()
        try:
            while not self._autonomy_stop_event.wait(0.03):
                result = avoider.compute(source.read_depth())
                command = build_drive_command(result.throttle, result.steering)
                self.ground.send(command)
                with self._lock:
                    self._ground_last_input = time.monotonic()
                    self._ground_sent_count += 1
                    self._autonomy_command = command
                    self._autonomy_reason = result.reason
        except Exception as exc:
            with self._lock:
                self._autonomy_error = str(exc)
                self._autonomy_reason = "自主运行异常，已停车"
            print(f"[AUTONOMY] 运行异常：{exc}", flush=True)
        finally:
            try:
                if source is not None:
                    source.close()
            finally:
                self.ground.stop()
                with self._lock:
                    self._autonomy_enabled = False
                    self._autonomy_command = "S"
                    self._autonomy_thread = None
                self._resume_depth_preview()

    def ground_control(self, throttle: float, steering: float) -> str:
        """接收网页方向输入，转换并发送地面车命令。"""

        with self._lock:
            if self._autonomy_enabled:
                raise RuntimeError("自主运行中，不能同时接收手动地面控制")

        throttle = max(-1.0, min(1.0, float(throttle)))
        steering = max(-1.0, min(1.0, float(steering)))
        command = build_axis_command(throttle, steering)
        with self._lock:
            self._ground_last_input = time.monotonic()
            self._ground_sent_count += 1
        self.ground.send(command)
        return command

    def ground_stop(self) -> None:
        """立即停车并刷新地面控制看门狗。"""

        if self._autonomy_enabled:
            self.stop_autonomy()
            return
        with self._lock:
            self._ground_last_input = time.monotonic()
            self._ground_sent_count += 1
        self.ground.stop()

    def flight_setpoint(self, values: dict[str, Any]) -> dict[str, float]:
        """保存网页飞行目标，实际发送由单一 PX4 后台线程完成。"""

        try:
            roll = max(-1.0, min(1.0, float(values.get("roll", 0.0))))
            pitch = max(-1.0, min(1.0, float(values.get("pitch", 0.0))))
            yaw = max(-1.0, min(1.0, float(values.get("yaw", 0.0))))
            thrust = max(0.0, min(1.0, float(values.get("thrust", 0.0))))
        except (TypeError, ValueError) as exc:
            raise ValueError("飞行控制参数必须是数字") from exc

        with self._lock:
            self._flight_setpoint = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "thrust": thrust,
            }
            self._flight_last_input = time.monotonic()
            if (
                self._last_logged_input_thrust is None
                or abs(thrust - self._last_logged_input_thrust) >= 0.02
            ):
                print(f"[NETWORK TRACE] 网页推力输入：{thrust:.3f}", flush=True)
                self._last_logged_input_thrust = thrust
            return dict(self._flight_setpoint)

    def set_flight_control_enabled(self, enabled: bool) -> bool:
        """打开或关闭网络飞行控制 setpoint 流。"""

        with self._lock:
            self._flight_control_enabled = bool(enabled)
            if enabled:
                # 打开开关时先以零姿态、零推力建立安全的初始 setpoint。
                self._flight_setpoint = {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "thrust": 0.0,
                }
                self._flight_last_input = time.monotonic()
            else:
                # 关闭后不再发送非零目标；已在 Offboard 且解锁时由安全循环处理。
                self._flight_setpoint = {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "thrust": 0.0,
                }
        return bool(enabled)

    def offboard_ready(self) -> bool:
        """判断是否已经建立可供 PX4 使用的连续 setpoint 输入。"""

        with self._lock:
            fresh = time.monotonic() - self._flight_last_input <= self.FLIGHT_SETPOINT_TIMEOUT
            enabled = self._flight_control_enabled
        return enabled and fresh and (self.px4 is None or self.px4.snapshot.connected)

    def preflight_disarm_status(self) -> dict[str, Any]:
        """返回 PX4 未起飞自动上锁调试参数的缓存状态。"""

        with self._lock:
            value = self._preflight_disarm_value
            disabled = self._preflight_disarm_disabled
        return {
            "parameter": self.PREFLIGHT_DISARM_PARAMETER,
            "value": value,
            "disabled": disabled,
        }

    def set_preflight_disarm_disabled(self, disabled: bool) -> dict[str, Any]:
        """临时关闭或恢复 PX4 未起飞自动上锁参数。"""

        if self.px4 is None:
            raise RuntimeError("当前为仿真模式，未连接 PX4 参数")
        if self.px4.snapshot.armed:
            raise RuntimeError("飞控已解锁，必须先上锁再修改调试参数")

        current = self.px4.read_parameter(self.PREFLIGHT_DISARM_PARAMETER)
        if disabled:
            with self._lock:
                if self._preflight_disarm_original is None and current >= 0:
                    self._preflight_disarm_original = current
            value = self.px4.write_parameter(self.PREFLIGHT_DISARM_PARAMETER, -1.0)
        else:
            with self._lock:
                restore_value = self._preflight_disarm_original
            # 服务重启后没有缓存原值，恢复按钮采用 PX4 默认值 10 秒并明确返回。
            value = self.px4.write_parameter(
                self.PREFLIGHT_DISARM_PARAMETER,
                10.0 if restore_value is None else restore_value,
            )

        with self._lock:
            self._preflight_disarm_value = value
            self._preflight_disarm_disabled = value < 0
        return self.preflight_disarm_status()

    def flight_status(self) -> dict[str, Any]:
        """返回页面展示所需的 PX4 状态。"""

        if self.px4 is None:
            return {
                "connected": False,
                "mode": "SIMULATION",
                "armed": False,
                "battery": None,
                "battery_voltage": None,
                "altitude_relative": None,
                "heartbeat_age": None,
                "last_setpoint_age": None,
                "setpoint_sent_age": None,
                "setpoint_sent_count": 0,
                "control_enabled": self._flight_control_enabled,
                "offboard_ready": self.offboard_ready(),
                "preflight_disarm_disabled": False,
                "preflight_disarm_timeout": None,
                "sent_thrust": None,
                "px4_attitude_target_thrust": None,
            }

        snapshot = self.px4.snapshot
        with self._lock:
            now = time.monotonic()
            setpoint_age = now - self._flight_last_input
            sent_age = (
                None
                if self._last_flight_setpoint_sent == 0.0
                else now - self._last_flight_setpoint_sent
            )
            control_enabled = self._flight_control_enabled
            sent_count = self._flight_setpoint_sent_count
        return {
            "connected": snapshot.connected,
            "mode": snapshot.mode_name,
            "armed": snapshot.armed,
            "battery": snapshot.battery_percent,
            "battery_voltage": snapshot.battery_voltage,
            "altitude_relative": snapshot.relative_altitude,
            "heartbeat_age": self.px4.heartbeat_age(),
            "last_setpoint_age": setpoint_age,
            "setpoint_sent_age": sent_age,
            "setpoint_sent_count": sent_count,
            "control_enabled": control_enabled,
            "offboard_ready": self.offboard_ready(),
            "preflight_disarm_disabled": self.preflight_disarm_status()["disabled"],
            "preflight_disarm_timeout": self.preflight_disarm_status()["value"],
            "sent_thrust": self._last_flight_sent_thrust,
            "px4_attitude_target_thrust": snapshot.attitude_target_thrust,
        }

    def status(self) -> dict[str, Any]:
        """返回网络控制台综合状态。"""

        with self._lock:
            ground_age = time.monotonic() - self._ground_last_input
            sent_count = self._ground_sent_count
        return {
            "hardware": self.hardware,
            "ground": {
                "connected": self.ground._serial is not None or not self.hardware,
                "mode": "LIVE" if self.hardware else "SIMULATION",
                "port": self.ground.port,
                "last_command": self.ground.last_command,
                "last_input_age": ground_age,
                "sent_count": sent_count,
                "autonomy_enabled": self.autonomy_status()["enabled"],
                "autonomy_error": self.autonomy_status()["error"],
                "autonomy_command": self.autonomy_status()["command"],
                "autonomy_reason": self.autonomy_status()["reason"],
            },
            "flight": self.flight_status(),
        }

    def _ground_safety_loop(self) -> None:
        """网页地面控制断流时自动停车。"""

        while not self._stop_event.wait(0.05):
            with self._lock:
                expired = time.monotonic() - self._ground_last_input > self.GROUND_COMMAND_TIMEOUT
            if expired and self.ground.last_command != "S":
                self.ground.stop()

    def _flight_loop(self) -> None:
        """单一线程读取 PX4，并在 OFFBOARD 时持续发送 setpoint。"""

        assert self.px4 is not None
        while not self._stop_event.is_set():
            try:
                self.px4.poll(0.02)
                snapshot = self.px4.snapshot
                now = time.monotonic()

                with self._lock:
                    setpoint = dict(self._flight_setpoint)
                    setpoint_age = now - self._flight_last_input
                    control_enabled = self._flight_control_enabled

                offboard_active = snapshot.armed and snapshot.mode_name.upper() == "OFFBOARD"
                # Offboard 要求在切换模式和解锁前就持续收到 setpoint，因此这里不再
                # 把发送条件绑定到 armed；非 Offboard 模式下 PX4 会忽略这些目标。
                if control_enabled and setpoint_age <= self.FLIGHT_SETPOINT_TIMEOUT:
                    self.px4.send_attitude_setpoint(**setpoint)
                    with self._lock:
                        self._last_flight_setpoint_sent = now
                        self._flight_setpoint_sent_count += 1
                        self._last_flight_sent_thrust = setpoint["thrust"]

                mode_name = snapshot.mode_name.upper()
                if (
                    self._last_logged_armed != snapshot.armed
                    or self._last_logged_mode != mode_name
                ):
                    print(
                        "[NETWORK TRACE] 飞控状态："
                        f"armed={snapshot.armed}, mode={mode_name}, "
                        f"input_thrust={setpoint['thrust']:.3f}, "
                        f"sent_thrust={self._last_flight_sent_thrust}, "
                        f"px4_target_thrust={snapshot.attitude_target_thrust}, "
                        f"input_age={setpoint_age:.3f}s",
                        flush=True,
                    )
                    self._last_logged_armed = snapshot.armed
                    self._last_logged_mode = mode_name

                if offboard_active:
                    if self._flight_started_at is None:
                        self._flight_started_at = now

                    unsafe = (
                        not control_enabled
                        or setpoint_age > self.FLIGHT_SETPOINT_TIMEOUT
                        or self.px4.heartbeat_age() > self.HEARTBEAT_TIMEOUT
                        or (
                            snapshot.relative_altitude is not None
                            and snapshot.relative_altitude > self.MAX_RELATIVE_ALTITUDE
                        )
                        or (
                            self._flight_started_at is not None
                            and now - self._flight_started_at > self.MAX_FLIGHT_TIME
                        )
                    )
                    if unsafe and now - self._last_flight_safety_action > 1.0:
                        self._last_flight_safety_action = now
                        print("[NETWORK SAFE] 飞控保护触发，发送降落指令", flush=True)
                        self.px4.emergency_land()
                else:
                    self._flight_started_at = None
            except Exception as exc:
                print(f"[NETWORK] PX4 循环异常：{exc}", flush=True)
                time.sleep(0.5)


controller: NetworkController | None = None


def get_controller() -> NetworkController:
    """为 Flask 测试或直接导入场景提供默认仿真控制器。"""

    global controller
    if controller is None:
        controller = NetworkController(
            hardware=False,
            ground_port="none",
            ground_baud=115200,
            px4_device="/dev/ttyS5",
            px4_baud=115200,
        )
        controller.start()
    return controller


@app.get("/")
def index():
    """返回 MD3 风格控制台页面。"""

    return render_template("index.html")


@app.get("/api/status")
def api_status():
    """返回地面车和飞控状态。"""

    return jsonify(get_controller().status())


@app.post("/api/ground/control")
def api_ground_control():
    """接收地面车摇杆数据。"""

    data = request.get_json(silent=True) or {}
    try:
        command = get_controller().ground_control(data.get("throttle", 0.0), data.get("steering", 0.0))
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    return jsonify({"ok": True, "command": command})


@app.post("/api/ground/stop")
def api_ground_stop():
    """接收地面车停车请求；自主运行时同时停止自主线程。"""

    get_controller().ground_stop()
    return jsonify({"ok": True, "command": "S"})


@app.post("/api/ground/autonomy")
def api_ground_autonomy():
    """启动或停止地面合力法自动避障。"""

    data = request.get_json(silent=True) or {}
    if not isinstance(data.get("enabled"), bool):
        return jsonify({"ok": False, "error": "enabled 必须是布尔值"}), 400
    current = get_controller()
    try:
        enabled = current.start_autonomy() if data["enabled"] else current.stop_autonomy()
    except Exception as exc:
        app.logger.exception("切换地面自主运行失败")
        return jsonify({"ok": False, "error": f"自主运行切换失败：{exc}"}), 409
    return jsonify({"ok": True, "enabled": enabled, "autonomy": current.autonomy_status()})


@app.post("/api/fc/setpoint")
def api_fc_setpoint():
    """接收网页飞行 setpoint。"""

    try:
        values = get_controller().flight_setpoint(request.get_json(silent=True) or {})
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    return jsonify({"ok": True, "setpoint": values})


@app.post("/api/fc/control")
def api_fc_control():
    """显式开启或关闭网络飞行 setpoint 流。"""

    data = request.get_json(silent=True) or {}
    if not isinstance(data.get("enabled"), bool):
        return jsonify({"ok": False, "error": "enabled 必须是布尔值"}), 400
    enabled = get_controller().set_flight_control_enabled(data["enabled"])
    return jsonify({"ok": True, "enabled": enabled})


@app.post("/api/fc/debug/preflight-disarm")
def api_debug_preflight_disarm():
    """调试用：关闭或恢复 PX4 未起飞自动上锁。"""

    data = request.get_json(silent=True) or {}
    if not isinstance(data.get("disabled"), bool):
        return jsonify({"ok": False, "error": "disabled 必须是布尔值"}), 400
    if data["disabled"] and data.get("confirm") is not True:
        return jsonify({"ok": False, "error": "关闭自动上锁需要明确确认"}), 400
    try:
        result = get_controller().set_preflight_disarm_disabled(data["disabled"])
    except Exception as exc:
        app.logger.exception("修改 PX4 自动上锁参数失败")
        return jsonify({"ok": False, "error": f"修改 PX4 参数失败：{exc}"}), 502
    return jsonify({"ok": True, **result})


@app.post("/api/fc/arm")
def api_fc_arm():
    """需要网页显式确认后才发送解锁命令。"""

    current = get_controller()
    if not (request.get_json(silent=True) or {}).get("confirm"):
        return jsonify({"ok": False, "error": "需要显式确认解锁"}), 400
    if current.px4 is None:
        return jsonify({"ok": False, "error": "当前为仿真模式"}), 400
    if current.px4.snapshot.mode_name.upper() != "OFFBOARD":
        return jsonify({"ok": False, "error": "网络解锁前必须先切换到 OFFBOARD"}), 409
    if not current.offboard_ready():
        return jsonify({"ok": False, "error": "Offboard setpoint 流尚未就绪，请先开启网络飞行控制"}), 409
    try:
        current.px4.arm()
    except Exception as exc:
        app.logger.exception("发送解锁命令失败")
        return jsonify({"ok": False, "error": f"飞控解锁命令发送失败：{exc}"}), 502
    return jsonify({"ok": True, "command": "ARM"})


@app.post("/api/fc/disarm")
def api_fc_disarm():
    """发送显式上锁命令。"""

    current = get_controller()
    if current.px4 is None:
        return jsonify({"ok": False, "error": "当前为仿真模式"}), 400
    try:
        current.px4.disarm()
    except Exception as exc:
        app.logger.exception("发送上锁命令失败")
        return jsonify({"ok": False, "error": f"飞控上锁命令发送失败：{exc}"}), 502
    return jsonify({"ok": True, "command": "DISARM"})


@app.post("/api/fc/mode")
def api_fc_mode():
    """发送显式飞行模式切换命令。"""

    current = get_controller()
    if current.px4 is None:
        return jsonify({"ok": False, "error": "当前为仿真模式"}), 400
    mode = str((request.get_json(silent=True) or {}).get("mode", "")).upper()
    if not mode:
        return jsonify({"ok": False, "error": "缺少模式名称"}), 400
    if mode == "OFFBOARD" and not current.offboard_ready():
        return jsonify({"ok": False, "error": "请先开启网络飞行控制，建立连续 setpoint 流"}), 409
    try:
        changed = current.px4.set_mode(mode)
    except Exception as exc:
        app.logger.exception("发送模式切换命令失败")
        return jsonify({"ok": False, "error": f"飞行模式切换失败：{exc}"}), 502
    return jsonify({"ok": changed, "mode": mode})


@app.post("/api/fc/emergency")
def api_fc_emergency():
    """发送显式紧急降落命令。"""

    current = get_controller()
    if current.px4 is None:
        return jsonify({"ok": False, "error": "当前为仿真模式"}), 400
    try:
        current.px4.emergency_land()
    except Exception as exc:
        app.logger.exception("发送紧急降落命令失败")
        return jsonify({"ok": False, "error": f"紧急降落命令发送失败：{exc}"}), 502
    return jsonify({"ok": True, "command": "LAND"})


def main() -> None:
    """解析参数并启动网络控制服务。"""

    global controller
    parser = argparse.ArgumentParser(description="CONTROL 网络控制台")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--hardware", action="store_true", help="连接真实 PX4 和 ESP32")
    parser.add_argument("--ground-port", default="/dev/ttyUSB0")
    parser.add_argument("--ground-baud", type=int, default=115200)
    parser.add_argument("--px4-device", default="/dev/ttyS5")
    parser.add_argument("--px4-baud", type=int, default=115200)
    args = parser.parse_args()

    controller = NetworkController(
        hardware=args.hardware,
        ground_port=args.ground_port,
        ground_baud=args.ground_baud,
        px4_device=args.px4_device,
        px4_baud=args.px4_baud,
    )
    try:
        controller.start()
    except Exception:
        # 任一硬件连接失败时释放已经打开的串口，避免下次重试被占用。
        controller.close()
        raise
    print(f"[NETWORK] 控制台：http://{args.host}:{args.port}", flush=True)
    print(f"[NETWORK] 模式：{'LIVE' if args.hardware else 'SIMULATION'}", flush=True)
    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
