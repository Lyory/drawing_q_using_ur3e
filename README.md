# UR3e Write Q - ROS 2 Humble

Project sử dụng **UR3e + MoveIt 2 + Gazebo** để thực hiện Cartesian path và trace chữ **Q**

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* MoveIt 2
* Gazebo
* Universal Robots ROS 2 packages
---

## 1. Cài đặt

```bash
sudo apt update

sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

---

## 3. Clone repositories

Clone repositories về máy

```bash
git clone https://github.com/Lyory/drawing_q_using_ur3e.git
```


## 5. Tiến hành chạy

```bash
cd drawing_q_using_ur3e/

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```
Sau khi build:

```bash
source install/setup.bash
```

Chạy launch file của package:

```bash
ros2 launch ur3e_write_q write_q.launch.py
```

