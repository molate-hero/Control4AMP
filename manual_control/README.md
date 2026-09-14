# CONTROL/manual_control：Orange Pi 手动遥控子模块

本子模块用于 Orange Pi Zero 3 上的陆空两栖设备手动遥控，包含：

- PX4 飞行控制状态读取和显式飞控命令接口；
- 通过 PX4 RC 通道控制 ESP32 地面车；
- AIR/GROUND、MANUAL/AUTO 控制权分配；
- RC 失效、PX4 心跳失效和飞控已解锁时的地面车安全停车。

程序不会自动解锁、自动起飞、自动切换飞行模式，也不会在启动时改变 PX4 的飞行状态。

## 一、目录结构

```text
manual_control/
├── README.md                         # 项目架构、接线约定和使用方法
├── requirements.txt                  # Python 依赖
├── run_manual_control.py             # 统一命令行入口
├── config/
│   ├── __init__.py
│   └── defaults.py                   # 默认串口和安全参数
├── control/
│   ├── __init__.py
│   ├── channels.py                    # RC 通道解析、差速混合和安全决策
│   ├── ground_controller.py           # ESP32 串口及 S/D 指令发送
│   ├── px4_controller.py              # PX4 MAVLink 通信和飞控状态
│   └── manual_bridge.py               # 飞控 RC 与地面车之间的控制桥
├── tests/
│   ├── __init__.py
│   └── test_channels.py               # 不依赖硬件的逻辑测试
└── scripts/
    └── start_manual_control.sh        # 使用现有虚拟环境启动程序
```

## 二、整体架构

```text
遥控器
  │
  ▼
PX4 飞控 ── MAVLink /dev/ttyS5 ──► px4_controller.py
                                      │
                                      ├─ 读取心跳、解锁状态、RC 通道
                                      │
                                      ▼
                              channels.py
                           RC 通道和安全决策
                                      │
                         ┌────────────┴────────────┐
                         │                         │
                   AIR 模式                  GROUND 模式
                地面车发送 S                 手动生成 D,L,R
                                                   │
                                                   ▼
                                  ground_controller.py
                                      │
                                      ▼
                              ESP32 /dev/ttyUSB0
```

飞行时，遥控器的飞行通道直接由 PX4 处理；本程序只监视 PX4 状态，并确保地面车不会同时动作。

## 三、RC 通道约定

| 通道 | 作用 | 约定 |
|---|---|---|
| CH1 | 地面车转向 | 左右摇杆 |
| CH2 | 地面车前进/后退 | 前推为前进，程序会自动取反 |
| CH7 | 空中/地面模式 | 低位 AIR，高位 GROUND |
| CH9 | 手动/自动选择 | 低位 MANUAL，高位 AUTO |

当前版本的 AUTO 只进入安全待机，不会启动自主驾驶程序。

## 四、硬件连接

默认设备参数如下：

```text
PX4 飞控：/dev/ttyS5，115200
ESP32：  /dev/ttyUSB0，115200
```

如果设备名称不同，可以通过命令行参数覆盖。

## 五、ESP32 指令协议

```text
S
```

表示停车。

```text
D,left,right
```

表示差速行驶，其中 `left` 和 `right` 范围为 `-1000` 到 `1000`。

示例：

```text
D,500,500       # 直行
D,-500,-500     # 后退
D,500,-500      # 原地右转
```

## 六、安装依赖

项目优先复用当前系统已有的 `flydrive-env` 虚拟环境，也可以手动安装依赖：

```bash
cd /home/orangepi/CONTROL/manual_control
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## 七、运行方法

### 1. 先运行纯逻辑自检

该命令不会访问 PX4、ESP32 或任何硬件：

```bash
cd /home/orangepi/CONTROL/manual_control
python3 run_manual_control.py --self-test
```

### 2. 连接真实设备运行

```bash
cd /home/orangepi/CONTROL/manual_control
./scripts/start_manual_control.sh
```

也可以直接指定设备：

```bash
./scripts/start_manual_control.sh \
  --px4-device /dev/ttyS5 \
  --ground-port /dev/ttyUSB0
```

### 3. 地面车仿真输出

该模式不会向真实 ESP32 写数据，但仍需要 PX4 提供 RC 通道：

```bash
./scripts/start_manual_control.sh --ground-port none
```

也可以使用：

```bash
python3 run_manual_control.py --dry-run
```

## 八、安全策略

1. PX4 心跳超过 2 秒未更新，向 ESP32 发送停车指令。
2. RC 通道超过 0.5 秒未更新，向 ESP32 发送停车指令。
3. RC 通道值超出合理范围，进入 `FAILSAFE` 并停车。
4. AIR 模式下地面车始终保持停车。
5. PX4 已解锁时拒绝进入 GROUND 模式。
6. GROUND+AUTO 当前只停车，不调用其他自主驾驶程序。
7. 程序退出、异常或键盘中断时都会先发送停车指令。
8. 程序不会自动调用解锁、起飞、模式切换或降落命令。

## 九、状态输出

程序会输出类似下面的状态：

```text
[CONTROL] mode=GROUND_MANUAL command=D,500,500 reason=地面手动遥控 px4_mode=MANUAL armed=False alt=0.0m
```

常见模式：

```text
AIR_MANUAL             空中手动模式，地面车停车
AIR_AUTO               空中自动选择，地面车停车
GROUND_MANUAL         地面手动遥控
GROUND_AUTO_STANDBY   地面自动待机
GROUND_REJECTED_ARMED 飞控已解锁，拒绝地面模式
FAILSAFE              数据失效保护
```

## 十、故障排查

### PX4 没有心跳

检查设备是否存在：

```bash
ls -l /dev/ttyS5
```

检查飞控串口波特率、TX/RX/GND 接线和 PX4 是否已经启动。

### 找不到 ESP32

检查 USB 串口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

然后使用 `--ground-port` 指定实际设备。

### 地面车不动作

确认 CH7 位于 GROUND、CH9 位于 MANUAL、飞控未解锁，并检查控制台是否持续收到 RC 通道。

### 只想测试逻辑

使用 `--self-test` 或运行：

```bash
python3 -m unittest discover -s tests -v
```
