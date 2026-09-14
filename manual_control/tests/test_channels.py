"""RC 解析和地面差速控制的单元测试。"""

import unittest

from control.channels import SystemMode, build_ground_command, resolve_decision


class ChannelLogicTest(unittest.TestCase):
    """验证不依赖硬件的安全控制规则。"""

    def test_center_stick_stops(self) -> None:
        """摇杆回中时必须停车。"""

        self.assertEqual(build_ground_command(1500, 1500), "S")

    def test_forward_command(self) -> None:
        """油门前推时左右轮应同向前进。"""

        self.assertEqual(build_ground_command(1500, 1225), "D,500,500")

    def test_right_turn_command(self) -> None:
        """只打转向时应生成左右轮反向的原地转向指令。"""

        self.assertEqual(build_ground_command(1775, 1500), "D,500,-500")

    def test_air_mode_stops_ground_vehicle(self) -> None:
        """空中模式必须让地面车停车。"""

        decision = resolve_decision(1500, 1225, 1000, 1000, False)
        self.assertEqual(decision.mode, SystemMode.AIR_MANUAL)
        self.assertEqual(decision.command, "S")

    def test_armed_vehicle_rejects_ground_mode(self) -> None:
        """飞控解锁时必须拒绝地面模式。"""

        decision = resolve_decision(1500, 1225, 2000, 1000, True)
        self.assertEqual(decision.mode, SystemMode.GROUND_REJECTED_ARMED)
        self.assertEqual(decision.command, "S")

    def test_invalid_rc_failsafe(self) -> None:
        """RC 通道异常时必须进入失效保护。"""

        decision = resolve_decision(0, 1500, 2000, 1000, False)
        self.assertEqual(decision.mode, SystemMode.FAILSAFE)
        self.assertEqual(decision.command, "S")


if __name__ == "__main__":
    unittest.main()
