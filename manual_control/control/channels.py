"""RC 通道解析和地面车差速指令生成。

本模块只包含纯逻辑，不访问串口、GPIO 或飞控，因此可以在没有硬件的
情况下单独测试。这样可以把“通道含义”和“硬件通信”明确分开。
"""

from dataclasses import dataclass
from enum import Enum


# 常见 RC 接收机 PWM 范围。
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
RC_VALID_MIN = 800
RC_VALID_MAX = 2200
RC_DEAD_ZONE = 50

# CH7：低位为空中模式，高位为地面模式。
AIR_GROUND_THRESHOLD = 1500

# CH9：低位为手动控制，高位为自动待机。
MANUAL_AUTO_THRESHOLD = 1500

# ESP32 电机协议的最大量程。
MAX_MOTOR_COMMAND = 1000


class SystemMode(str, Enum):
    """系统当前由哪一类控制逻辑接管。"""

    AIR_MANUAL = "AIR_MANUAL"
    AIR_AUTO = "AIR_AUTO"
    GROUND_MANUAL = "GROUND_MANUAL"
    GROUND_AUTO_STANDBY = "GROUND_AUTO_STANDBY"
    GROUND_REJECTED_ARMED = "GROUND_REJECTED_ARMED"
    FAILSAFE = "FAILSAFE"


@dataclass(frozen=True)
class ControlDecision:
    """一次 RC 输入经过安全策略后的地面输出结果。"""

    mode: SystemMode
    command: str
    reason: str


def clamp(value: int, minimum: int, maximum: int) -> int:
    """将整数限制在指定范围内。"""

    return max(minimum, min(maximum, value))


def rc_value_valid(value: int) -> bool:
    """判断一个 PWM 通道值是否处于合理范围。"""

    return RC_VALID_MIN <= value <= RC_VALID_MAX


def pwm_to_axis(pwm: int) -> int:
    """将 1000~2000 PWM 值转换成 -1000~1000 的控制量。"""

    pwm = clamp(pwm, RC_MIN, RC_MAX)
    difference = pwm - RC_MID

    # 中位附近设置死区，避免摇杆轻微抖动造成电机动作。
    if abs(difference) <= RC_DEAD_ZONE:
        return 0

    if difference > 0:
        value = int(
            (difference - RC_DEAD_ZONE)
            * MAX_MOTOR_COMMAND
            / (RC_MAX - RC_MID - RC_DEAD_ZONE)
        )
    else:
        value = int(
            (difference + RC_DEAD_ZONE)
            * MAX_MOTOR_COMMAND
            / (RC_MID - RC_MIN - RC_DEAD_ZONE)
        )

    return clamp(value, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND)


def mix_drive(throttle: int, steering: int) -> tuple[int, int]:
    """将前进/后退和转向混合成左右轮速度。"""

    left = throttle + steering
    right = throttle - steering
    largest = max(abs(left), abs(right))

    # 混合后如果超出电机量程，按比例缩放而不是截断一侧。
    if largest > MAX_MOTOR_COMMAND:
        left = int(left * MAX_MOTOR_COMMAND / largest)
        right = int(right * MAX_MOTOR_COMMAND / largest)

    return left, right


def build_ground_command(channel_1: int, channel_2: int) -> str:
    """将 RC CH1/CH2 转换成 ESP32 的停车或差速指令。"""

    # CH1 为转向，CH2 在常见遥控器上“前推”时数值下降，因此取反。
    steering = pwm_to_axis(channel_1)
    throttle = -pwm_to_axis(channel_2)
    left, right = mix_drive(throttle, steering)

    if left == 0 and right == 0:
        return "S"
    return f"D,{left},{right}"


def build_axis_command(throttle: float, steering: float) -> str:
    """将网页摇杆的 -1~1 归一化输入转换成 ESP32 差速指令。"""

    # 网页输入采用浮点归一化值，先转换到与 RC 逻辑相同的电机量程。
    throttle_axis = clamp(round(throttle * MAX_MOTOR_COMMAND), -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND)
    steering_axis = clamp(round(steering * MAX_MOTOR_COMMAND), -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND)
    left, right = mix_drive(throttle_axis, steering_axis)

    if left == 0 and right == 0:
        return "S"
    return f"D,{left},{right}"


def resolve_decision(
    channel_1: int,
    channel_2: int,
    channel_7: int,
    channel_9: int,
    pixhawk_armed: bool,
) -> ControlDecision:
    """根据 RC 开关和飞控状态生成安全的地面控制结果。

    这里明确规定：
    1. 无效 RC 数据直接停车；
    2. 空中模式时地面车停车，飞行控制仍由 PX4/遥控器负责；
    3. 飞控已解锁时拒绝地面模式，避免空中误切换导致地面电机动作；
    4. 自动模式在本项目中只进入安全待机，不启动自主驾驶。
    """

    channels = (channel_1, channel_2, channel_7, channel_9)
    if not all(rc_value_valid(value) for value in channels):
        return ControlDecision(SystemMode.FAILSAFE, "S", "RC 通道值无效")

    requested_ground = channel_7 >= AIR_GROUND_THRESHOLD
    requested_auto = channel_9 >= MANUAL_AUTO_THRESHOLD

    if not requested_ground:
        mode = SystemMode.AIR_AUTO if requested_auto else SystemMode.AIR_MANUAL
        return ControlDecision(mode, "S", "空中模式，地面车停车")

    if pixhawk_armed:
        return ControlDecision(
            SystemMode.GROUND_REJECTED_ARMED,
            "S",
            "飞控已解锁，拒绝进入地面模式",
        )

    if requested_auto:
        return ControlDecision(
            SystemMode.GROUND_AUTO_STANDBY,
            "S",
            "自动模式尚未接入，地面车安全待机",
        )

    return ControlDecision(
        SystemMode.GROUND_MANUAL,
        build_ground_command(channel_1, channel_2),
        "地面手动遥控",
    )


def run_logic_self_test() -> None:
    """执行不依赖硬件的核心逻辑自检。"""

    assert build_ground_command(1500, 1500) == "S"
    assert build_ground_command(1500, 1225) == "D,500,500"
    assert build_ground_command(1775, 1500) == "D,500,-500"

    assert resolve_decision(1500, 1225, 1000, 1000, False).command == "S"
    assert resolve_decision(1500, 1225, 2000, 1000, False).command == "D,500,500"
    assert resolve_decision(1500, 1225, 2000, 1000, True).mode == SystemMode.GROUND_REJECTED_ARMED
    assert resolve_decision(0, 1500, 2000, 1000, False).mode == SystemMode.FAILSAFE
