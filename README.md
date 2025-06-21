# Keyboard Control Node ✈️🕹️

A lightweight ROS 2 tele‑operation node for **PX4 SITL + Gazebo** that sends body‑frame velocity commands and logs way‑points on demand.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select utilities_pkg
source install/setup.bash
```

---

## Quick Start

1. **Launch QGroundControl (QGC)**  → keeps PX4 supplied with the required MAVLink heartbeat.
2. **Start the keyboard tele‑op node**

   ```bash
   ros2 run utilities_pkg keyboard_control_node
   ```
3. **Bring up the project interfaces** (bridges, TF, RViz, XRCE‑DDS Agent, etc.)

   ```bash
   ros2 launch gazebo_env_setup turn_interfaces.launch.py
   ```

> Without QGC (or an equivalent 1 Hz MAVLink heartbeat) PX4 v1.14+ will refuse to ARM or enter OFFBOARD mode.
---
## Features


For gimbal control, send `/gimbal_pitch_degree` Topic to `offboard_control` node.
The data is `std_msgs::msg::Float32`.


---

## Features

| Key       | Action                          | Topic / Command                    | Notes                               |
| --------- | ------------------------------- | ---------------------------------- | ----------------------------------- |
| **w / x** | Forward / Backward (Body X)     | `/command/twist`                   | ± 0.5 m/s per tap (clamped ± 3 m/s) |
| **a / d** | Left / Right (Body Y)           | 〃                                 | 〃                                  |
| **q / e** | Up / Down (Body Z)              | 〃                                 | 〃                                  |
| **z / c** | Yaw CCW / CW                    | 〃                                 | ± 0.1 rad/s per tap                 |
| **s**     | Hard stop (zero twist)          | 〃                                 |                                     |
| **0**     | Gimbal pitch 0° (level)         | `VEHICLE_CMD_DO_MOUNT_CONTROL`     | sends `204` + `205`                 |
| **9**     | Gimbal pitch ‑90° (down)        | 〃                                 |                                     |
| **o**     | Log **UAV** pose → `uav_wp.csv` | TF `map → x500_gimbal_0/base_link` | appends `x,y,z`                     |
| **p**     | Log **UGV** pose → `ugv_wp.csv` | TF `map → x1_asp/base_link`        | 〃                                  |
| **k**     | Kill motors                     | `/command/disarm`                  |                                     |

---

## Dependencies

| Package                            | Tested Version |
| ---------------------------------- | -------------- |
| ROS 2 Humble                       | ✅              |
| `px4_msgs`                         | PX4 v1.14 +    |
| `tf2_ros`, `tf2_geometry_msgs`     | ✅              |
| PX4 SITL (Gazebo Classic/Harmonic) | ✅              |

---

## How It Works

* Publishes **`geometry_msgs/Twist`** messages on `/command/twist` at **20 Hz**.
* After the first incoming set‑point, automatically sends
  `VEHICLE_CMD_DO_SET_MODE` (param1 = 1, param2 = 6) and `VEHICLE_CMD_COMPONENT_ARM_DISARM` to switch PX4 into OFFBOARD mode and ARM.
* Way‑points are extracted from TF transforms and appended to CSV files in the current working directory.
