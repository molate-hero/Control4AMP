# 陆空两栖机器人
这是一款支持飞行与陆地移动的机器车，其通过四旋翼实现飞行，总尺寸为37cm * 37cm * 27cm ，车体尺寸为21cm * 19 cm * 7cm。

## 设备连接
该设备通过OPi连接ESP32控制飞控，同时通过OPi连接RealSense深度相机做深度图和RGB图像采集。
OPi连接地址为：
- 有线连接：`ssh opi-eth`
- 无线连接：`ssh opi-wifi`
- frp转发连接：`ssh opi-frp`
优先使用有线连接，若无法使用则使用无线连接，若仍无法使用则使用frp转发连接。

## 代码规范
所有代码都需要遵循规范：
- 代码中必须有中文注释，且注释内容必须清晰、准确、完整。
- 代码按照模块化组织
- 模块中必须有README.md文件
- 注册新的模块必须在整个项目的README.md中注册，并在AGENTS.md中注册
- 待完成的任务必须写入 `.ai/TODO.md`中
- 当前做到哪里、下一步做什么写入 `.ai/HANDOFF.md`中
- 重要技术决策写入 `.ai/DECISIONS.md`中

## 现有模块
- `manual_control`：遥控器经 PX4 RC 通道控制 ESP32 地面车的手动遥控桥，含 AIR/GROUND 控制权分配和安全停车。
- `network_control`：Flask 浏览器控制台，提供地面车网络遥控、PX4 飞行 setpoint 和自主避障 / `AI auto` 视觉导航启动入口。
- `autonomous`：自主控制；当前包含 `ground_avoidance`（RealSense 深度合力法地面避障）与 `vlm_navigation`（VLM 视觉导航闭环，拍照→大模型决策→地面动作，只控制地面车）。
- `mechanical`：机械设计；当前包含 `car_shell`，即用 Blender 脚本参数化生成的一体式小车外壳与后部滑盖。