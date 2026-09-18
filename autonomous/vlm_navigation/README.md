# vlm_navigation VLM 视觉导航

本模块用视觉大模型（VLM）驱动地面车闭环行驶：**拍照 → 发给模型 → 模型调用工具下发动作 → 执行 → 再拍照**，如此循环。默认目标是向前行进并探索周围环境。

它与 `autonomous/ground_avoidance` 平级且相互独立：`ground_avoidance` 用深度相机做几何避障，本模块用彩色图像交给大模型做语义决策。**本模块只控制地面车，完全不涉及 PX4 和飞行。**

## 一、目录结构

```text
vlm_navigation/
├── README.md                    # 本文件：架构、工具、参数、运行与安全
├── requirements.txt             # numpy / Pillow / pyrealsense2 / pyserial
├── .env.example                 # 凭据模板（复制为 .env 后填写）
├── __init__.py
├── vlm_config.py                # 默认参数 + 极简 .env 解析
├── ground_link.py               # 复用 manual_control 的地面串口控制器与差速混合
├── realsense_color_source.py    # RealSense 彩色帧采集 + JPEG 编码
├── motion.py                    # 基于时长的运动执行（前进/后退/旋转/停车）
├── vlm_tools.py                 # 供模型调用的工具定义与派发
├── vlm_client.py                # OpenAI 兼容的 VLM 客户端（标准库 urllib）
├── navigation_loop.py           # 主循环编排
├── run_vlm_navigation.py        # 独立入口（默认 dry-run）
├── scripts/start_vlm_navigation.sh
└── tests/                       # 纯逻辑测试，不需要硬件与网络
    ├── test_vlm_config.py
    ├── test_motion.py
    ├── test_vlm_tools.py
    ├── test_vlm_client.py
    ├── test_color_source.py
    └── test_navigation_loop.py
```

## 二、整体架构

```text
RealSense 彩色流
      │  capture_jpeg()
      ▼
navigation_loop ── 图像 + 目标 ──► VlmClient ── HTTP ──► 视觉大模型
      ▲                                                    │
      │                                            工具调用（function calling）
      │                                                    ▼
      └──── 执行完成后重新拍照 ◄── MotionExecutor ◄── vlm_tools.dispatch_tool
                                      │
                                      ▼
                         GroundController（复用 manual_control）
                                      │
                                      ▼
                              ESP32 /dev/ttyUSB0
```

每一轮：拍照 → 把图像和任务目标发给模型 → 模型返回若干工具调用 → 逐个执行 → 重新拍照进入下一轮，直到停止、达到最大步数或出现不可恢复错误。

## 三、模型可调用的工具

| 工具 | 参数 | 语义 |
|---|---|---|
| `run_forward(seconds, speed)` | `speed > 0` 前进，`< 0` 后退 | 沿车头方向直线行驶指定秒数 |
| `rotate(seconds, speed)` | `speed > 0` 逆时针（左转），`< 0` 顺时针（右转） | 原地旋转指定秒数 |
| `stop()` | 无 | 立即停车 |

旋转的符号约定与既有协议一致：`D,500,-500` 是原地右转（见 `manual_control/README.md`）。

**为什么要"时长 + 速度"而不是"直接给电机值"**：模型更容易判断"往前走一小段"这类语义量；时长和速度都由程序统一限幅，模型给不出危险参数。

**ESP32 指令不上锁存**：一条 `D,left,right` 只在收到后生效，`network_control` 还有 0.5 秒地面看门狗。因此 `MotionExecutor` 会在动作时长内按 0.05 秒周期**反复重发**同一条指令，时长到点后一定发送停车指令 `S`。两次动作之间的"思考"时间里没有指令，看门狗会让车停下等待——这是有意的安全设计。

## 四、配置

复制 `.env.example` 为同目录的 `.env` 并填写：

```text
DEEPSEEK_API_KEY=sk-...
DEEPSEEK_BASE_URL=https://api.deepseek.com/v1
DEEPSEEK_MODEL=deepseek-v4.1-flash
```

读取顺序是"进程环境变量优先，其次 .env 文件"，`.env` 已被 `.gitignore` 忽略，不会入库。接口按 OpenAI 的 `chat/completions` 规范调用，因此换成其他兼容服务商时只需改 `BASE_URL` 和 `MODEL`。

主要参数集中在 `vlm_config.py`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `CameraConfig.width/height/fps` | 640 / 480 / 30 | 彩色流分辨率与帧率 |
| `CameraConfig.jpeg_quality` | 80 | 上传图片的 JPEG 质量 |
| `MotionConfig.tick_seconds` | 0.05 | 指令重发周期，须远小于 0.5 秒看门狗 |
| `MotionConfig.max_action_seconds` | 2.0 | 单次动作最长时长 |
| `MotionConfig.max_speed` | 1.0 | 归一化速度上限 |
| `VlmConfig.goal` | 向前行进并探索周围环境 | 默认任务目标 |
| `VlmConfig.max_context_images` | 2 | 上下文中保留的历史图片数 |
| `VlmConfig.max_steps` | 40 | 最大决策步数 |
| `VlmConfig.max_consecutive_text_only` | 3 | 连续多少次只回文字就停止 |
| `VlmConfig.max_api_failures` | 3 | 连续调用失败多少次就退出 |

## 五、运行方法

### 1. 本地纯逻辑测试（无需硬件与网络）

```bash
cd autonomous/vlm_navigation
python3 -m unittest discover -s tests -v
```

### 2. 假模型闭环（dry-run，不驱动车辆、不消耗 API）

用脚本化假模型跑通整条链路，适合先在机器人上确认相机和串口是否正常：

```bash
./scripts/start_vlm_navigation.sh --fake-vlm
```

### 3. 真实模型 + 仿真输出

```bash
cp .env.example .env   # 填入 API key
./scripts/start_vlm_navigation.sh
```

### 4. 真实驱动小车

必须先停止其他占用 RealSense 或 `/dev/ttyUSB0` 的程序，并确认环境安全：

```bash
./scripts/start_vlm_navigation.sh --hardware
```

常用参数：`--goal`（任务目标）、`--max-action-seconds`、`--max-speed`、`--max-steps`、`--model`、`--base-url`、`--esp32-port`、`--fps`。

**必须用启动脚本**：`pyserial` 在 `flydrive-env` 虚拟环境里，直接 `python3 run_vlm_navigation.py` 会报缺依赖；RealSense SDK 和 numpy 在系统用户 site-packages，脚本会一并加入 `PYTHONPATH`。

## 六、网页控制台集成（AI auto）

`network_control` 的地面卡片提供 **AI auto** 按钮，点击后在服务端启动本模块的主循环，并在提示行显示当前步数、动作和原因；再次点击或按"立即停车"即可停止。

- 与**深度避障**（自主运行）互斥：两者都要独占 RealSense，同时只能运行一个；
- 启动前会沿用既有的机制暂停 `/home/orangepi/run_depth.sh` 深度预览，停止时恢复；
- 启动前会确认飞控未解锁；AI auto 运行期间拒绝手动地面输入；
- 动作执行期间会刷新 0.5 秒地面看门狗。

## 七、安全约束

1. VLM 无法下发任意电机值：只能通过三个工具、且时长与速度都被限幅。
2. 每次动作结束都强制停车，动作之间车辆保持静止等待下一步决策。
3. 循环因任何原因结束（停止请求、步数上限、连续 API 失败、相机异常）都会执行停车。
4. 相机读取失败、连续 API 失败会停车退出，不会带着旧动作继续跑。
5. 模型连续多次只回文字（不调用工具）会被判定为空转并停止。
6. 飞控已解锁时拒绝启动。
7. 本模块不导入任何 PX4 / 飞行代码。

## 八、已知限制

- **没有里程计**：`seconds × speed` 是开环控制，只能按时间近似位移和转角，没有位置/朝向反馈。现场需要根据实际地面摩擦调 `max_action_seconds` 和 `max_speed`。
- **相机独占**：RGB 与深度预览、深度避障不能同时使用 RealSense，切换需要释放相机。
- **依赖模型能力**：模型必须同时支持图像输入和 function calling；若服务商或模型不支持，接口会返回明确错误。
- **费用与延迟**：每一轮都要上传一张图并等待模型回复，走的是公网 API，延迟和成本随步数增长；上下文只保留最近 `max_context_images` 张图片以控制开销。
- **尚未在真实硬件上行驶验证**：目前只完成了离线逻辑测试与仿真/假模型链路验证。
