# CONTROL 项目

`CONTROL` 是 Orange Pi 上的总控制项目目录，后续会逐步容纳手动遥控、传感器接入、自主决策、任务状态机和公共通信基础设施等模块。

当前第一阶段已实现：

```text
manual_control/        # 手动遥控子模块：PX4 飞行状态 + ESP32 地面车
network_control/       # 网络控制子模块：浏览器控制台 + 地面/飞行 API
autonomous/            # 自主控制子模块：地面合力法避障 + VLM 视觉导航
mechanical/            # 机械设计子模块：当前包含 Blender 参数化生成的小车外壳
```

其他能力暂不放入本模块，后续建议按下面的边界扩展：

```text
CONTROL/
├── README.md                  # 总项目架构和模块说明
├── manual_control/            # 手动遥控模块
├── network_control/           # 网络控制模块
├── autonomous/                # 自主感知、决策和任务状态机
│   ├── ground_avoidance/      # RealSense 地面合力法避障，独立于其他控制模块
│   └── vlm_navigation/        # VLM 视觉导航闭环，独立于其他控制模块
├── mechanical/                # 机械设计模块
│   └── car_shell/             # 一体式小车外壳 + 后部滑盖，Blender 脚本参数化生成
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

该模块使用 RealSense 深度帧，把前方视野裁剪为左、中、右三个区域，分别估计可通行距离，再将“默认向前目标力”和“远离障碍物斥力”合成为地面车的 `throttle/steering`。算法会按地面平面模型剔除近处地面像素，避免把地面误判为障碍；前进速度只由正前方距离决定并从 1.8 米开始连续减速，侧面障碍物只影响转向，算法不会自动倒车。正前方深度无效时判定为危险并停车，不会盲目前进。它不导入 `manual_control` 或 `network_control`，默认只计算并打印结果；只有显式传入 `--hardware` 才会向 ESP32 `/dev/ttyUSB0` 发送差速命令。

运行前必须停止其他占用 RealSense 或 ESP32 串口的程序，当前版本只做前向避障，正面近障碍时停车并选择左右更宽的一侧转向，不会自动倒车。相机安装姿态改变后需用 `calibrate_ground.py` 重新标定地面参数。详细参数、接口、标定方法、已知限制和测试方法见：

```text
/home/orangepi/CONTROL/autonomous/ground_avoidance/README.md
```

## 当前模块：autonomous/vlm_navigation

该模块用视觉大模型（VLM）驱动地面车闭环行驶：用 RealSense 拍一张彩色画面发给模型，模型通过 function calling 调用「前进/后退」「旋转」工具，小车按指定时长和速度执行后停车、再拍下一张，如此循环；默认目标是向前行进并探索周围环境。速度与时长的符号约定为：前进速度为正、后退为负，旋转速度为正表示逆时针、为负表示顺时针。它只控制地面车、不导入任何 PX4 或飞行代码，默认 dry-run，只有显式传入 `--hardware` 才会向 ESP32 发送差速命令。网页控制台的地面卡片提供 `AI auto` 按钮启动同一套循环，并与深度避障互斥（两者都要独占 RealSense）。

详细工具语义、`.env` 配置、参数、运行与测试方法、安全约束见：

```text
/home/orangepi/CONTROL/autonomous/vlm_navigation/README.md
```

```bash
cd /home/orangepi/CONTROL/autonomous/vlm_navigation
python3 -m unittest discover -s tests -v        # 纯逻辑测试，无需硬件
./scripts/start_vlm_navigation.sh --fake-vlm    # 假模型跑通链路，不消耗 API
```

## 当前模块：network_control

该模块提供 MD3 风格浏览器控制台、地面车四键/WASD 控制、RealSense 合力法自主避障、VLM 视觉导航（`AI auto` 按钮）、PX4 飞行目标控制、状态监视和超时安全保护。详细说明见：

```text
/home/orangepi/CONTROL/network_control/README.md
```

```bash
cd /home/orangepi/CONTROL/network_control
./scripts/start_network_control.sh
```

## 当前模块：mechanical/car_shell

该模块用 Blender 脚本参数化生成小车外壳，采用「一体式主壳体 + 独立后部滑盖」结构：除滑盖外的整个车身为一个连续的一体化零件，含左右半包覆轮罩、后部设备舱、顶部维护开口、滑槽与内部安装结构。整体尺寸 305 × 210 × 72 mm，壁厚 2.5 mm，为电池、Jetson Orin Nano 和电机预留了安装与走线空间。

脚本每次运行都会自动校核尺寸、间隙、内部结构与装配干涉，并在 `renders/` 下输出多视角预览图。详细尺寸推导、与任务书要求的对应关系和推定尺寸清单见：

```text
/home/molate/Projects/AmphibiousRobot/CONTROL/mechanical/car_shell/README.md
```

```bash
cd /home/molate/Projects/AmphibiousRobot/CONTROL/mechanical/car_shell
blender --background --factory-startup --python build_car_shell.py
```
