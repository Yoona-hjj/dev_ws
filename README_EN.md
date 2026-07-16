# Team Qingqing Grassland

## 1. Team Basic Information

| Item | Details |
|---|---|
| Team Name | Qingqing Grassland Team |
| University | College of Information Engineering, Hangzhou Dianzi University |
| Team Leader | Han Jiajie |
| Competition Mode | Fully Autonomous Mode |
| Software Framework | ROS 2 |
| Main Platform | RDK X5 |
| Controller & Chassis | Ackermann chassis, controlled via serial protocol bridge |

## 2. Solution Overview

This solution addresses tasks including track following, obstacle avoidance, QR code recognition, route selection, visual task recognition, and end-point parking. It adopts a hybrid autonomous driving architecture combining "visual perception + inertial fusion localization + waypoint navigation + visual control takeover".

Upon system startup, ROS 2 Launch brings up the camera, image codec, visual perception, QR code recognition, inertial fusion, waypoint navigation, and chassis control nodes together. The robot initially follows waypoints based on fused odometry; switches routes upon detecting QR codes or reaching designated waypoints; transitions to visual line-following and obstacle avoidance during the final return leg or at visual takeover points; and completes parking through P-point detection and alignment logic.

## 3. Overall System Architecture

```text
USB Camera
    │
    ▼
RDK Camera Node / hobot_codec
    │ Shared memory NV12 image: /nv12_img
    ├──────────────► ResNet Track Detection ─────► /racing_track_center_detection
    ├──────────────► YOLO Obstacle/P-point Detection ─► /racing_obstacle_detection
    └──────────────► ROI QR Code Recognition ───────► /qrcode_result, /qrcode_bottom

Chassis Serial / IMU
    │
    ▼
V2 Serial Protocol Bridge ─► Madgwick IMU ─► EKF/ZUPT/Non-holonomic Constraint
    │
    └────────────────────────────► /odom_combined

/odom_combined + /nav_command
    │
    ▼
waypoint_nav_node
    │
    ├─ Waypoint control: /nav_cmd_vel
    ├─ Visual takeover signals: /vision_enable, /car_go
    └─ Image task request: /get_picture

Line-following / Obstacle Detection
    │
    ▼
racing_control ─► /racing
                         │
Waypoint control /nav_cmd_vel ───┼──► control_master ─► /cmd_vel ─► Chassis
                         │
                 LiDAR Obstacle Avoidance (optional overlay)

/get_picture ─► Image Compression ─► VLM Client ─► /vision_language_model
```

## 4. Hardware Selection and Interfaces

### 4.1 Computing Platform

- RDK X5 as the main computing platform.
- ROS 2 for node execution, launch files, and message communication.
- Horizon DNN nodes and BPU inference for track detection and obstacle detection.
- Shared-memory NV12 image transport to reduce copying and transmission overhead.

### 4.2 Sensors and Actuators

- USB Camera: captures images of the track, obstacles, QR codes, and end-point targets.
- IMU: provides attitude and angular velocity data, fed into Madgwick filter and EKF.
- Chassis Odometry / Serial: receives chassis status and sends control commands through the V2 protocol bridge node.
- LiDAR: optional obstacle-avoidance sensor; subscribes to `/scan` when `enable_lidar_avoid` is enabled.
- Ackermann Chassis: receives unified control output via `/cmd_vel`.

### 4.3 Main Interfaces

| Topic | Purpose |
|---|---|
| `/nv12_img` | RDK shared-memory NV12 image |
| `/racing_track_center_detection` | Track center detection result |
| `/racing_obstacle_detection` | Obstacle and P-point detection results |
| `/qrcode_result` | QR code content and route information |
| `/qrcode_bottom` | Bottom position of QR code in image, used for approach deceleration |
| `/odom_combined` | EKF fused odometry |
| `/nav_cmd_vel` | Waypoint navigation control command |
| `/racing` | Visual line-following control command |
| `/vision_enable` | Visual control takeover switch |
| `/cmd_vel` | Unified velocity command output to chassis |
| `/vision_language_model` | Scene recognition result |

## 5. Software System Design

### 5.1 Startup Method

The solution uses a hybrid navigation launch file:

```bash
ros2 launch origincar_v2_bridge hybrid_nav.launch.py
```

After startup, send the auto-start command:

```bash
ros2 topic pub /nav_command std_msgs/msg/String "data: 'start'" --once
```

Common control commands:

```bash
# Pause
ros2 topic pub /nav_command std_msgs/msg/String "data: 'pause'" --once

# Resume
ros2 topic pub /nav_command std_msgs/msg/String "data: 'resume'" --once

# Reset
ros2 topic pub /nav_command std_msgs/msg/String "data: 'reset'" --once
```

### 5.2 Perception Modules

#### Track Detection

The ResNet track detection node extracts the track center point from NV12 images and outputs `track_center` type results. The control node computes angular velocity based on the deviation between the image center and the track center, with clamping and filtering applied to the control output.

#### Obstacle Detection

YOLO is used to detect track obstacles and P-point targets. The system selects the primary target based on confidence, bounding box area, and bottom position; adjusts direction when approaching obstacles; and enters alignment, blind-approach, and parking phases when approaching the P-point.

#### QR Code Recognition

The QR code recognition node outputs QR content and position. The content determines clockwise or counter-clockwise routing; the bottom position is used for deceleration and parking control during the visual phase.

#### Visual Language Model (VLM) for Scenes

When YOLO detects a person, a frame is captured via the `/get_picture` request, sent to the visual language model for scene description, and the result is published to `/vision_language_model`. API keys are provided via environment variables and are not hard-coded in source files.

### 5.3 Inertial Fusion and Waypoint Navigation

Chassis serial data, IMU, and velocity information are processed as follows:

1. The V2 serial protocol bridge node receives chassis status.
2. The Madgwick node filters IMU attitude.
3. The non-holonomic constraint node provides the Ackermann `vy=0` constraint to the EKF.
4. The ZUPT node provides zero-velocity updates during stationary states.
5. robot_localization EKF outputs `/odom_combined`.
6. `waypoint_nav_node` generates `/nav_cmd_vel` based on waypoints, position, and heading error.

Waypoint control employs lookahead targeting, heading PID, speed grading, curve deceleration, local relocalization, and reverse / U-turn strategies when necessary.

### 5.4 Control Authority Management

The system uses `control_master` to publish `/cmd_vel` to the chassis in a unified manner, preventing the waypoint node and visual node from directly controlling the chassis simultaneously:

- When visual takeover is disabled, forwards `/nav_cmd_vel`.
- When visual takeover is enabled, forwards `/racing`.
- When LiDAR obstacle avoidance is enabled, overlays local avoidance strategies on top of the base command.
- Upon QR code or P-point parking trigger, outputs zero velocity.
- Repeatedly publishes zero velocity on program exit to reduce the risk of the chassis maintaining the last speed.

## 6. Key Task Implementation Strategies

| Task | Implementation Strategy |
|---|---|
| Track Following | ResNet extracts track center; proportional control and filtering generate visual steering |
| Obstacle Avoidance | YOLO detects obstacles; visual control performs directional avoidance; optional LiDAR for local obstacle avoidance |
| QR Code Recognition | ROI-based QR detection, confirmed after consecutive frames, outputs route direction |
| Route Selection | Switches clockwise / counter-clockwise waypoint routes based on QR content |
| Inertial Navigation | Fusion of serial, IMU, EKF, ZUPT, and Ackermann non-holonomic constraints |
| Waypoint Tracking | Lookahead waypoints, heading PID, curve deceleration, and waypoint-reached state machine |
| VLM Task | Captures image at designated waypoint, asynchronously calls visual language model |
| P-point Parking | Lateral target alignment, approach deceleration, blind-approach timer, and final zero-velocity output |
| Control Safety | Unified chassis output via `control_master`, reducing multi-node control contention |

## 7. Rule Adaptation Notes

- Fully autonomous competition mode — the robot operates autonomously based on pre-set waypoints, visual recognition, and fusion localization.
- Route selection via QR code content, adapting to different route direction tasks.
- Track following and obstacle avoidance completed through line detection and obstacle detection.
- End-point task completed through P-point detection and parking state machine.
- ROS 2 node-based design separates perception, localization, planning, and control modules for convenient on-site debugging and fault isolation.
- Specific scoring rules, track dimensions, obstacle positions, and task trigger conditions follow the official provincial competition rules.

## 8. Removed Content and Models

### Removed Content Description

```
├── models/
│   ├── converted_model.bin          # YOLO obstacle detection model
│   └── race_track_detection.bin     # ResNet track detection model
└── config/
    ├── waypoints.yaml               # Actual track waypoints (500) and control parameters
    ├── ekf_akm.yaml                 # EKF fusion and Ackermann calibration parameters
    └── imu_madgwick.yaml            # IMU Madgwick filter parameters
```
