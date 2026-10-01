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


## Tự plan chữ Q, Execute trong RViz

Sau khi build và source, chạy `ros2 launch ur3e_write_q write_q.launch.py`.
Launch mở Gazebo và panel **MotionPlanning** trong RViz. Khi robot và RViz sẵn sàng,
hệ thống đợi cả hai controller active và joint state mới, rồi tự lập **một kế hoạch đầy đủ**: tới tư thế bắt đầu → nâng công cụ → tiếp cận
và vẽ chữ Q. Robot trong Gazebo vẫn đứng yên.

Đợi log `Full Q plan ready. Press Execute in RViz to draw.`, rồi bấm **Execute**
trong MotionPlanning để thực hiện toàn bộ kế hoạch. Đường xanh là các waypoint chữ Q.
Khi Plan, tay máy ảo chạy xem trước chữ Q. Khi bấm Execute, preview dừng và ẩn,
chỉ còn tay máy theo trạng thái thực tế của mô phỏng chạy theo kế hoạch đã lưu.
Bấm Plan lần nữa sẽ bật lại preview. Panel vẫn là MotionPlanning của MoveIt,
được mở rộng bằng plugin `ur3e_write_q/MotionPlanning` để đổi hiển thị đúng lúc.

Trong launch này, nút **Plan** tính lại toàn bộ chữ Q từ trạng thái hiện tại;
không dùng tay nắm Goal State để thay đổi bài vẽ. Giữ **External Comm.** bật và
**Use Cartesian Path** tắt như cấu hình mặc định để luồng tự plan hoạt động.
Dùng **Plan**, sau đó **Execute**; yêu cầu **Plan & Execute** bị từ chối để giữ
bước xem trước. Nếu robot đã đổi vị trí sau khi plan, bấm Plan lại trước khi Execute.
Nếu một chặng không lập kế hoạch đầy đủ, toàn bộ kế hoạch bị từ chối và không chạy.

`plan_q.py` trả kế hoạch gộp qua action `/move_action` cho RViz. Action lập kế
hoạch gốc của MoveIt được chuyển sang `/draw_q/backend_move_action`. Nút Execute vẫn dùng `/execute_trajectory` chuẩn của MoveIt. Các chặng
Cartesian dùng trạng thái cuối của chặng trước và FK, không dùng TF của robot
đang đứng yên. Script `draw_q.py` giữ các thông số/hình học chữ Q ban đầu.

Mỗi lần launch dùng một Gazebo transport partition riêng cho server, GUI, spawn và clock bridge, tránh kết nối nhầm vào Gazebo cũ còn chạy nền. Đóng phiên ROS cũ trước khi launch lại để tránh trùng node/topic trong cùng ROS domain.
