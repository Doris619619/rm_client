# rm_client

RoboMaster 客户端 ROS2 包。该仓库实现了以下核心能力：

- 通过 MQTT + Protobuf 接入裁判系统数据，并发布到 ROS2 话题。
- 通过 ROS2 话题发送半自动控制指令到裁判系统。
- 将关键状态桥接为 HTTP JSON 接口，供前端页面展示。
- 支持辅助图像链路（模拟源或海康相机源）并在前端实时显示。

## 1. 功能概览

当前主要可执行节点：

- judge_receiver：订阅裁判系统 MQTT 话题，解码后发布到 /judge/*。
- judge_sender：订阅 /semi_auto/command_id，将 command_id 封装为 GuardCtrlCommand 后发到 MQTT。
- semi_auto_keyboard：在终端采集按键并发布 command_id。
- mock_custom_image_pub：发布测试辅助图像数据（用于联调）。
- camera_custom_image_pub：采集海康相机图像，压缩/裁剪后发布到 /judge/custom_byte_block。
- state_bridge.py：订阅关键 ROS2 话题，提供 HTTP 接口 http://127.0.0.1:8000/api/state。
- frontend：纯静态页面，通过轮询 API 展示比赛态势、链路新鲜度和辅助图像。

## 2. 目录结构

- config/
  - judge_params.yaml：receiver/sender/keyboard 的默认参数。
- proto/
  - guard_ctrl.proto
  - judge_system.proto
- src/
  - judge_receiver.cpp
  - judge_sender.cpp
  - semi_auto_keyboard.cpp
  - mock_custom_image_pub.cpp
  - camera_custom_image_pub.cpp
- scripts/
  - run_client_dev.sh：开发联调一键启动脚本。
  - run_client_match.sh：比赛模式一键启动脚本。
  - stop_client.sh：按 pid 文件安全停进程。
  - state_bridge.py：状态桥接 HTTP 服务。
- frontend/
  - index.html + app.js + style.css

## 3. 环境依赖

建议环境：

- Ubuntu 22.04+
- ROS2（humble / iron / jazzy / rolling 之一）
- C++17

系统库依赖（按实际系统版本调整包名）：

- Protobuf（编译与运行）
- Eclipse Paho MQTT C 与 C++
- OpenCV
- 海康 MVS SDK（仅 camera_custom_image_pub 需要）

说明：

- CMake 默认从 /opt/MVS 查找海康 SDK 头文件与库。
- 若缺少 paho-mqttpp3 或 paho-mqtt3as/paho-mqtt3a，配置阶段会直接报错。

## 4. 构建

在工作空间根目录执行：

1. 构建

	colcon build --packages-select rm_client

2. 载入环境

	source install/setup.bash

## 5. 快速启动（推荐）

进入包目录后使用脚本：

1. 开发联调模式

	./scripts/run_client_dev.sh

2. 比赛模式

	./scripts/run_client_match.sh

3. 停止全部相关进程

	./scripts/stop_client.sh

启动后默认端口：

- 状态 API：http://127.0.0.1:8000/api/state
- 前端页面：http://127.0.0.1:8081

日志与 PID 文件：

- 日志目录：logs/
- PID 目录：logs/pids/

## 6. 手动分模块运行

如果需要拆分调试，可分别运行：

1. 接收裁判系统数据

	ros2 run rm_client judge_receiver --ros-args --params-file src/rm_client/config/judge_params.yaml

2. 启动状态桥接服务

	python3 src/rm_client/scripts/state_bridge.py

3. 启动前端静态服务

	cd src/rm_client/frontend && python3 -m http.server 8081

4. 键盘半自动输入

	ros2 run rm_client semi_auto_keyboard --ros-args --params-file src/rm_client/config/judge_params.yaml

5. 指令发送器

	ros2 run rm_client judge_sender --ros-args --params-file src/rm_client/config/judge_params.yaml

## 7. 参数说明

主要参数文件：config/judge_params.yaml

- judge_receiver
  - judge_host / judge_port：MQTT 地址
  - client_id：接收端客户端 ID
  - qos：订阅 QoS
  - reconnect_period_sec：重连周期
  - ros_prefix：ROS 话题前缀（默认 /judge）
- judge_sender
  - judge_host / judge_port：MQTT 地址
  - client_id：发送端客户端 ID
  - topic_cmd：控制指令 MQTT topic（默认 GuardCtrlCommand）
  - ros_command_topic：输入指令 ROS 话题（默认 /semi_auto/command_id）
- semi_auto_keyboard
  - ros_command_topic：输出指令话题
  - cooldown_ms / rate_limit_ms：按键发布限频

## 8. 关键数据流

1. 裁判系统 MQTT -> judge_receiver -> ROS2 /judge/*
2. /semi_auto/command_id -> judge_sender -> GuardCtrlCommand(MQTT)
3. /judge/{game_status,global_unit_status,robot_dynamic_status,guard_ctrl_result,custom_byte_block}
	-> state_bridge.py -> /api/state(JSON)
4. frontend/app.js 轮询 /api/state 并渲染 HUD 页面

## 9. 辅助图像链路

两种输入源：

- mock_custom_image_pub：用于无相机环境联调。
- camera_custom_image_pub：用于真实海康相机。

camera_custom_image_pub 默认输出协议：

- 固定 160x120
- GRAY8
- 发布到 /judge/custom_byte_block

常用参数示例（按需通过 --ros-args -p 覆盖）：

- camera_index
- camera_serial
- publish_fps
- roi_size
- enable_clahe
- clahe_clip_limit
- chunk_payload_size

## 10. 前端说明

前端为静态页面，默认每 400ms 轮询 API。

页面主要显示：

- 比分、回合、阶段、剩余时间
- 机器人状态与全局关键状态
- 指令回执与链路新鲜度（ONLINE/STALE）
- 辅助图像窗口与源端帧率估计

## 11. 常见问题

1) 启动脚本报 ROS2 setup 找不到

- 确认已安装 ROS2，并且 /opt/ros/<distro>/setup.bash 存在。

2) CMake 报找不到 Paho 或 MVS

- 检查 paho-mqttpp3、paho-mqtt3as（或 paho-mqtt3a）是否安装。
- 若使用相机节点，检查 /opt/MVS/include 与 /opt/MVS/lib/64。

3) 前端显示 OFFLINE

- 检查 state_bridge.py 是否在运行。
- 检查 8000 端口是否被占用。
- 检查 /judge 相关话题是否有数据。

4) 辅助图像长期 NO DATA

- 联调场景先启动 mock_custom_image_pub。
- 相机场景确认相机可被 MVS 枚举并成功抓帧。

## 12. 开发建议

- 比赛与开发环境建议使用不同参数文件，避免误连现场网络。
- 先通过 mock 跑通链路，再切真实相机与真实裁判系统。
- 调试时优先查看 logs/ 下对应 .log 文件定位问题。
