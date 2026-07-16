# 青青草原队

## 1. 参赛队伍基本信息

| 项目 | 内容 |
|---|---|
| 赛队名称 | 青青草原队 |
| 学校名称 | 杭州电子科技大学信息工程学院 |
| 队长 | 韩佳杰 |
| 参赛模式 | 全自动模式 |
| 软件框架 | ROS 2 |
| 主要运行平台 | RDK X5|
| 主控设备与底盘 | 阿克曼底盘，串口协议桥接控制 |

## 2. 方案概述
本方案面向赛道巡线、障碍物规避、二维码识别、路线选择、视觉任务识别和终点目标停车等任务，采用“视觉感知 + 惯性融合定位 + 航点导航 + 视觉控制接管”的混合式自动驾驶架构。
系统启动后由 ROS 2 Launch 统一拉起相机、图像编解码、视觉感知、二维码识别、惯性融合、航点导航和底盘控制节点。机器人前期根据融合里程计执行航点导航；在二维码或指定路线阶段完成路线切换；在最终返程或视觉接管点切换至视觉巡线和视觉避障控制；最终通过 P 点检测与对准逻辑完成停车。

## 3. 整体系统架构

```text
USB 摄像头
    │
    ▼
RDK 相机节点 / hobot_codec
    │ 共享内存 NV12 图像：/nv12_img
    ├──────────────► ResNet 巡线检测 ─────► /racing_track_center_detection
    ├──────────────► YOLO 障碍物/P点检测 ─► /racing_obstacle_detection
    └──────────────► ROI 二维码识别 ───────► /qrcode_result、/qrcode_bottom

底盘串口 / IMU
    │
    ▼
V2 串口协议桥 ─► Madgwick IMU ─► EKF/ZUPT/非完整约束
    │
    └────────────────────────────► /odom_combined

/odom_combined + /nav_command
    │
    ▼
waypoint_nav_node
    │
    ├─ 航点控制指令：/nav_cmd_vel
    ├─ 视觉接管信号：/vision_enable、/car_go
    └─ 图像任务请求：/get_picture

巡线/障碍物检测
    │
    ▼
racing_control ─► /racing
                         │
航点控制 /nav_cmd_vel ───┼──► control_master ─► /cmd_vel ─► 底盘
                         │
                 LiDAR 避障可选覆盖

/get_picture ─► 图像压缩 ─► VLM 客户端 ─► /vision_language_model
```

## 4. 硬件选型与连接方式

### 4.1 计算平台

- RDK X5 作为主计算平台。
- 使用 ROS 2 运行节点、Launch 文件和消息通信。
- 使用地平线 DNN 节点和 BPU 推理能力执行巡线及障碍物检测。
- 使用共享内存方式传输 NV12 图像，降低图像复制和传输开销。

### 4.2 传感器与执行机构

- USB 摄像头：采集赛道、障碍物、二维码和终点目标图像。
- IMU：提供姿态与角速度信息，接入 Madgwick 滤波和 EKF。
- 底盘里程/串口：通过 V2 协议桥接节点接收底盘状态并发送控制指令。
- 激光雷达：作为可选避障传感器，开启 `enable_lidar_avoid` 后接入 `/scan`。
- 阿克曼底盘：通过 `/cmd_vel` 接收统一控制输出。

### 4.3 主要接口

| 接口 | 作用 |
|---|---|
| `/nv12_img` | RDK 共享内存 NV12 图像 |
| `/racing_track_center_detection` | 巡线中心检测结果 |
| `/racing_obstacle_detection` | 障碍物和 P 点检测结果 |
| `/qrcode_result` | 二维码内容和路线信息 |
| `/qrcode_bottom` | 二维码在图像中的底部位置，用于接近减速 |
| `/odom_combined` | EKF 融合里程计 |
| `/nav_cmd_vel` | 航点导航控制指令 |
| `/racing` | 视觉巡线控制指令 |
| `/vision_enable` | 视觉控制接管开关 |
| `/cmd_vel` | 主控统一输出到底盘的速度指令 |
| `/vision_language_model` | 场景识别结果 |

## 5. 软件系统设计

### 5.1 启动方式

方案使用混合导航 Launch：

```bash
ros2 launch origincar_v2_bridge hybrid_nav.launch.py
```

启动后发送自动开始命令：

```bash
ros2 topic pub /nav_command std_msgs/msg/String "data: 'start'" --once
```

常用控制命令：

```bash
# 暂停
ros2 topic pub /nav_command std_msgs/msg/String "data: 'pause'" --once

# 恢复
ros2 topic pub /nav_command std_msgs/msg/String "data: 'resume'" --once

# 重置
ros2 topic pub /nav_command std_msgs/msg/String "data: 'reset'" --once
```

### 5.2 感知模块

#### 巡线检测

使用 ResNet 巡线检测节点从 NV12 图像中提取赛道中心点，输出 `track_center` 类型结果。控制节点根据图像中心与赛道中心偏差计算角速度，并对控制输出进行限幅和滤波。

#### 障碍物检测

使用 YOLO 检测赛道障碍物和 P 点目标。系统根据目标置信度、检测框面积和目标底部位置选择主要目标，在接近障碍物时执行方向调整，在接近 P 点时进入对准、盲进和停车阶段。

#### 二维码识别

二维码识别节点输出二维码内容和二维码位置。二维码内容用于确定顺时针或逆时针路线；二维码底部位置用于视觉控制阶段的减速和停车控制。

#### 场景视觉语言模型

在yolo识别到person以后，通过 `/get_picture` 请求采集一帧图像，发送给视觉语言模型进行场景描述，并将结果发布到 `/vision_language_model`，API 密钥通过环境变量提供，不写入源代码。

### 5.3 惯性融合与航点导航

底盘串口、IMU 和速度信息经过以下处理：

1. V2 串口协议桥接节点接收底盘状态。
2. Madgwick 节点对 IMU 姿态进行滤波。
3. 非完整约束节点向 EKF 提供阿克曼车辆 `vy=0` 约束。
4. ZUPT 节点在静止状态提供零速更新。
5. robot_localization EKF 输出 `/odom_combined`。
6. `waypoint_nav_node` 根据航点、位置和航向误差生成 `/nav_cmd_vel`。

航点控制采用前视目标、航向 PID、速度分级、曲线减速、局部重定位和必要时的倒车/U 型转向策略。

### 5.4 控制权管理

系统使用 `control_master` 统一向底盘发布 `/cmd_vel`，避免航点节点和视觉节点同时直接控制底盘：

- 未启用视觉接管时，转发 `/nav_cmd_vel`。
- 启用视觉接管后，转发 `/racing`。
- 启用 LiDAR 避障时，在基础控制指令上叠加局部避障策略。
- 二维码或 P 点停车标志触发后，输出零速度。
- 程序退出时重复发布零速度，降低底盘保持最后速度的风险。

## 6. 关键任务实现策略

| 任务 | 实现策略 |
|---|---|
| 赛道巡线 | ResNet 提取赛道中心，比例控制与滤波生成视觉转向 |
| 障碍物规避 | YOLO 检测障碍物；视觉控制执行方向规避；可选 LiDAR 进行局部避障 |
| 二维码识别 | ROI 二维码识别，连续检测后确认，输出路线方向 |
| 路线选择 | 根据二维码内容切换顺时针/逆时针航点路线 |
| 惯性导航 | 串口、IMU、EKF、ZUPT 和阿克曼非完整约束融合 |
| 航点跟踪 | 前视航点、航向 PID、曲线减速和到点任务状态机 |
| VLM 任务 | 到达指定航点后采图，异步调用视觉语言模型 |
| P 点停车 | 目标横向对准、接近减速、盲进计时和最终零速度输出 |
| 控制安全 | 由 `control_master` 统一输出到底盘，减少多节点抢占控制权 |

## 7. 规则适配说明
- 参赛模式采用全自动模式，机器人根据预设航点、视觉识别和融合定位自主运行。
- 通过二维码内容选择对应路线，适配不同路线方向任务。
- 通过巡线和障碍物检测完成赛道跟踪与障碍规避。
- 通过 P 点检测和停车状态机完成终点任务。
- 通过 ROS 2 节点化设计划分感知、定位、规划和控制模块，便于现场调试和故障定位。
- 具体评分规则、赛道尺寸、障碍物位置和任务触发条件以省赛正式规则为准。


## 8.已删除内容及模型
#### 删除内容说明
```
├── models/
│   ├── converted_model.bin          # YOLO 障碍物检测模型
│   └── race_track_detection.bin     # ResNet 巡线检测模型
└── config/
    ├── waypoints.yaml               # 实际赛道航点（500个）与控制参数
    ├── ekf_akm.yaml                 # EKF 融合与阿克曼标定参数
    └── imu_madgwick.yaml            # IMU Madgwick 滤波参数
```





