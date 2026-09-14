# CONTROL 项目

`CONTROL` 是 Orange Pi 上的总控制项目目录，后续会逐步容纳手动遥控、传感器接入、自主决策、任务状态机和公共通信基础设施等模块。

当前第一阶段已实现：

```text
manual_control/        # 手动遥控子模块：PX4 飞行状态 + ESP32 地面车
network_control/       # 网络控制子模块：浏览器控制台 + 地面/飞行 API
autonomous/            # 自主控制子模块：当前包含地面合力法自动避障
```

其他能力暂不放入本模块，后续建议按下面的边界扩展：

```text
CONTROL/
├── README.md                  # 总项目架构和模块说明
├── manual_control/            # 手动遥控模块
├── network_control/           # 网络控制模块
├── autonomous/                # 自主感知、决策和任务状态机
│   └── ground_avoidance/      # RealSense 地面合力法避障，独立于其他控制模块
├── sensors/                   # 后续统一传感器接口
├── common/                    # 后续公共数据结构、日志和安全策略
└── tools/                     # 后续诊断、标定和运维工具
```

## 当前模块：manual_control

该模块负责：

1. 读取 PX4 的 MAVLink 心跳、RC 通道和飞行状态；
2. 在 AIR 模式下让 PX4 继续处理遥控器飞行控制；
3. 在 GROUND+MANUAL 模式下把 RC 通道转换成 ESP32 差速指令；
4. 在 RC 失联、PX4 心跳失联或飞控已解锁时保护地面车停车；
5. 提供显式的 PX4 模式切换、解锁、上锁和紧急降落接口，但默认不会自动调用。

详细目录、接线、运行方法和安全策略见：

```text
/home/orangepi/CONTROL/manual_control/README.md
```

## 快速使用

```bash
cd /home/orangepi/CONTROL/manual_control
python3 run_manual_control.py --self-test
./scripts/start_manual_control.sh
```

手动遥控模块不会自动启动其他传感器或自主驾驶功能；后续模块应通过明确的接口接入，避免多个模块同时控制同一硬件。

## 当前模块：autonomous/ground_avoidance

该模块使用 RealSense 深度帧，把前方视野裁剪为左、中、右三个区域，分别估计可通行距离，再将“默认向前目标力”和“远离障碍物斥力”合成为地面车的 `throttle/steering`。侧面障碍物只影响转向，正面障碍物负责减速，算法不会自动倒车。它不导入 `manual_control` 或 `network_control`，默认只计算并打印结果；只有显式传入 `--hardware` 才会向 ESP32 `/dev/ttyUSB0` 发送差速命令。

运行前必须停止其他占用 RealSense 或 ESP32 串口的程序，当前版本只做前向避障，正面近障碍时停车并选择左右更宽的一侧转向，不会自动倒车。详细参数、接口和测试方法见：

```text
/home/orangepi/CONTROL/autonomous/ground_avoidance/README.md
```

## 当前模块：network_control

该模块提供 MD3 风格浏览器控制台、地面车四键/WASD 控制、RealSense 合力法自主避障、PX4 飞行目标控制、状态监视和超时安全保护。详细说明见：

```text
/home/orangepi/CONTROL/network_control/README.md
```

```bash
cd /home/orangepi/CONTROL/network_control
./scripts/start_network_control.sh
```
