# Keyboard Control Node ✈️🕹️

ROS 2 keyboard tele-op node for a **PX4 SITL + Gazebo** setup:

* **Body-frame velocity control** of the UAV (`/command/twist`)
* **Gimbal pitch presets** via PX4 `VEHICLE_CMD_DO_MOUNT_*`
* **On-the-fly waypoint logging** from TF transforms  
  └─ writes `uav_wp.csv` and `ugv_wp.csv`

> Works out-of-the-box with the X500 drone model (`x500_gimbal_0`)  
> and the X1 UGV (`x1_asp`) in the project’s Gazebo world.

---

## Features

| Key | Action | Topic / Command | Notes |
|-----|--------|-----------------|-------|
| **w / x** | Forward / Backward (Body X) | `/command/twist` | ± 0.5 m/s per tap, clipped ±3 m/s |
| **a / d** | Left / Right (Body Y) | 〃 | 〃 |
| **q / e** | Up / Down (Body Z) | 〃 | 〃 |
| **z / c** | Yaw CCW / CW | 〃 | ± 0.1 rad/s per tap |
| **s** | Hard stop (zero all twist) | 〃 | |
| **0** | Gimbal pitch 0° (level) | `VEHICLE_CMD_DO_MOUNT_CONTROL` | sends `204` + `205` combo |
| **1** | Gimbal pitch -90° (down) | 〃 | |
| **o** | Log current **UAV** pose → `uav_wp.csv` | TF `map → x500_gimbal_0/base_link` | appends `x,y,z` |
| **p** | Log current **UGV** pose → `ugv_wp.csv` | TF `map → x1_asp/base_link` | 〃 |

---

## Dependencies

| Package | Tested Version |
|---------|---------------|
| ROS 2 Humble | ✅ |
| `px4_msgs` | PX4 v1.14+ |
| `tf2_ros`, `tf2_geometry_msgs` | ✅ |
| PX4 SITL (Gazebo Classic/Harmonic) | ✅ |

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select utilities_pkg
source install/setup.bash
