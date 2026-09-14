"""网络控制 API 的仿真测试，不连接真实串口。"""

import unittest

import server


class NetworkApiTest(unittest.TestCase):
    """验证网页、状态、地面控制和飞行参数接口。"""

    def setUp(self) -> None:
        # 每个测试前清理全局控制器，确保测试之间互不影响。
        if server.controller is not None:
            server.controller.close()
        server.controller = None
        self.client = server.app.test_client()

    def tearDown(self) -> None:
        # 测试结束后释放仿真控制器；真实模式也可复用同一清理路径。
        if server.controller is not None:
            server.controller.close()
        server.controller = None

    def test_page_and_status(self) -> None:
        """首页和状态接口应正常返回仿真状态。"""

        self.assertEqual(self.client.get("/").status_code, 200)
        payload = self.client.get("/api/status").get_json()
        self.assertFalse(payload["hardware"])
        self.assertEqual(payload["ground"]["mode"], "SIMULATION")

    def test_ground_and_flight_input(self) -> None:
        """地面摇杆和飞行目标应被校验、限幅并返回。"""

        ground = self.client.post(
            "/api/ground/control",
            json={"throttle": 0.5, "steering": 0},
        )
        self.assertEqual(ground.get_json()["command"], "D,500,500")
        stop = self.client.post("/api/ground/stop", json={})
        self.assertEqual(stop.get_json()["command"], "S")

        flight = self.client.post(
            "/api/fc/setpoint",
            json={"roll": 2, "pitch": -2, "yaw": 0.2, "thrust": 0.8},
        )
        self.assertEqual(flight.get_json()["setpoint"]["roll"], 1.0)
        self.assertEqual(flight.get_json()["setpoint"]["pitch"], -1.0)

        enabled = self.client.post("/api/fc/control", json={"enabled": True})
        self.assertEqual(enabled.get_json()["enabled"], True)
        self.assertTrue(self.client.get("/api/status").get_json()["flight"]["control_enabled"])
        disabled = self.client.post("/api/fc/control", json={"enabled": False})
        self.assertEqual(disabled.get_json()["enabled"], False)

    def test_simulation_rejects_destructive_flight_commands(self) -> None:
        """仿真模式不应发送解锁、上锁或紧急降落命令。"""

        response = self.client.post("/api/fc/arm", json={"confirm": True})
        self.assertEqual(response.status_code, 400)

    def test_simulation_rejects_real_autonomy(self) -> None:
        """仿真服务不能误启动真实相机和地面车自主线程。"""

        response = self.client.post("/api/ground/autonomy", json={"enabled": True})
        self.assertEqual(response.status_code, 409)


if __name__ == "__main__":
    unittest.main()
