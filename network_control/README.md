# network_control 网络控制子模块

本模块是 `CONTROL` 项目中的网络控制部分，提供浏览器控制台、地面车网络遥控和 PX4 飞行 setpoint 转发。它与 `manual_control` 平级，不能代表整个 `CONTROL` 项目。

## 目录结构

```text
network_control/
├── __init__.py                  # 网络控制模块标识
├── README.md                    # 网络控制架构、接口和使用方法
├── requirements.txt             # Flask、pyserial、pymavlink 依赖
├── run_network_control.py       # 服务启动入口
├── server.py                    # Flask API、状态管理和安全循环
├── templates/index.html         # MD3 风格控制页面
├── static/style.css             # 页面布局、MD3 形状和色彩令牌
├── static/app.js                # 方向键、键盘输入、自主运行、滑块和确认交互
├── tests/test_server.py         # 仿真 API 回归测试
└── scripts/start_network_control.sh # Orange Pi 启动脚本
```

## 架构说明

浏览器通过 REST API 将地面方向输入和飞行 setpoint 发送到 `server.py`；服务复用 `manual_control/control` 中的 ESP32 差速协议和 PX4 MAVLink 类，并由后台线程统一读取 PX4、发送目标和执行看门狗保护。

页面采用 MD3 风格组件，主色为参考图的 `#769CDF`，同时使用浅冷色 surface、圆角容器、tonal button、switch、dialog 和 snackbar。

地面控制使用“前、后、左、右”四个按钮：鼠标或触摸按住按钮会持续发送差速指令，松开后停车；方向键和 `W/A/S/D` 与按钮等价，也支持同时按下形成组合方向。快速连续两次按下“前”（间隔不超过约 350 毫秒）会进入疾跑，前进保持期间使用满速，松开“前”后自动退出疾跑。普通地面行驶限速约为 65%，用于降低误触时的初始速度。

地面卡片中的“自主运行”按钮会启动 `autonomous/ground_avoidance` 的 RealSense 合力法避障线程。启动前服务会确认飞控未解锁，并暂时暂停占用相机的 `/home/orangepi/run_depth.sh` 深度预览；停止自主运行时会停车并恢复原来正在运行的预览服务。自主运行期间后端拒绝手动地面输入，网页按钮再次点击即可停止。

地面卡片中的“AI auto”按钮会启动 `autonomous/vlm_navigation` 的 VLM 视觉导航循环：连续拍摄彩色画面交给视觉大模型，由模型通过 function calling 下发前进/后退/旋转动作，默认目标是向前探索。它与“自主运行”互斥（两者都要独占 RealSense，同时只能运行一个），同样会暂停并恢复深度预览、启动前确认飞控未解锁、运行期间拒绝手动地面输入；提示行会显示当前步数、动作和原因。AI auto 需要在本模块侧配置 VLM 的 API key（见 `autonomous/vlm_navigation/.env.example`）。

## 使用方法

```bash
cd /home/orangepi/CONTROL/network_control
python3 -m pip install -r requirements.txt
./scripts/start_network_control.sh
```

默认监听 `0.0.0.0:5001`，浏览器访问 `http://<香橙派IP>:5001/`。默认启动为仿真模式，不连接串口；仿真模式适合先验证页面和 API。

运行仿真测试：

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

需要实际连接 PX4 `/dev/ttyS5` 与地面车 ESP32 `/dev/ttyUSB0` 时，必须明确使用：

```bash
./scripts/start_network_control.sh --hardware
```

可用参数包括 `--host`、`--port`、`--ground-port`、`--ground-baud`、`--px4-device` 和 `--px4-baud`。

## API 概览

`GET /api/status` 获取服务、地面车和飞控状态；`POST /api/ground/control` 与 `/api/ground/stop` 控制地面车；`POST /api/ground/autonomy` 启动或停止地面自动避障；`POST /api/ground/ai-auto` 启动或停止 VLM 视觉导航（`AI auto`）；`POST /api/fc/control` 显式建立或关闭网络 setpoint 流；`POST /api/fc/setpoint` 发送飞行目标；`POST /api/fc/mode`、`/api/fc/arm`、`/api/fc/disarm` 和 `/api/fc/emergency` 执行需要明确操作的飞控命令；`POST /api/fc/debug/preflight-disarm` 是拆桨调试用的 PX4 自动上锁参数开关。

网页操作顺序是：打开网络飞行控制开关，等待状态显示 `Offboard 就绪`，再切换到 `OFFBOARD`，最后在确认安全后解锁。服务端会拒绝没有连续 setpoint 流时的 OFFBOARD 切换和网络解锁。

PX4 还可能根据自身参数自动上锁，例如 `COM_DISARM_PRFLT` 控制解锁后长时间未起飞的自动上锁时间；网络控制页面会实时显示 `已解锁` 或 `已上锁`，便于区分“推力目标为零”和“飞控实际上锁”。

调试按钮关闭的是 `COM_DISARM_PRFLT`（写入 `-1`），不是关闭全部安全机制；Offboard 失联、心跳超时、最大高度和最大飞行时长保护仍然有效。恢复按钮会恢复服务启动时记录的值；如果服务重启后原值未知，则恢复为 PX4 默认的 10 秒。该参数可能写入 PX4 参数存储，调试结束必须恢复。

## 安全约束

地面车控制超过 0.5 秒没有新输入会自动停车；飞行 setpoint 超过 0.5 秒、PX4 心跳超过 2 秒、相对高度超过 0.60 米或连续飞行超过 15 秒时触发紧急降落；服务不会自动解锁或自动切换飞行模式。真实硬件测试前必须卸载或停止其他占用 `/dev/ttyS5`、`/dev/ttyUSB0` 的控制程序。
