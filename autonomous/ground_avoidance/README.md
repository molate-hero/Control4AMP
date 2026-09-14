# ground_avoidance 地面自动避障

本模块独立于 `manual_control` 和 `network_control`，只负责 RealSense 深度感知、合力法决策和可选的 ESP32 差速输出。

## 运行方式

默认 dry-run，只打印差速命令，不会驱动车辆：

```bash
cd /home/orangepi/CONTROL/autonomous/ground_avoidance
./scripts/start_ground_avoidance.sh
```

确认已经停止手动遥控、网页地面控制和深度预览服务，并且拆除车轮或确认环境安全后，才允许真实输出：

```bash
./scripts/start_ground_avoidance.sh --hardware --esp32-port /dev/ttyUSB0
```

当前算法把相机正前方定义为默认目标方向：没有外部目标点时会自动向前行进。侧面障碍物只产生横向绕行力，不会把车辆推成倒车；正面障碍物只负责减速，进入安全距离后会停车并向左右剩余空间更大的一侧转向。
