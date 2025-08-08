# scout_ws

---
```bash
conda deactivate
```
---

## 1. gazebo
---
```bash
cd ~/scout_ws
colcon build
source ~/scout_ws/install/setup.bash
source /usr/share/gazebo-11/setup.bash
```
---
---
```bash
ros2 launch scout_gazebo_sim office_scout_mini.launch.py
```
---

## 2. 조이스틱 제어
---
```bash
cd ~/Downloads/ETRI_7004_UI_
npm start
```
---

## 3. 키보드 제어
---
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
---

## 4. gzserver & gzclient 확인
---
```bash
ps faux | grep gzclient
ps faux | grep gzserver
```
---

| 시나리오                                      | 권장 파일                           |
| ----------------------------------------- | ------------------------------- |
| **“빨리 돌려서 네비·알고리즘만 시험”** (물리 세부치 중요 X)    | `mini.xacro` (가볍고 단순)           |
| **“실제 Scout Mini 와 동일한 운동·무게 중심을 보고 싶다”** | `scout_mini.xacro` (정확한 CAD·관성) |


# YOLO.V8
https://devshin.notion.site/Project-ROS2-with-YOLOv8-0e5f104cb8ca43ccb03b04c49100131d

## 1. yolov8_ros 패키지를 가져오고 빌드
---
```bash
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git -b 2.2.1
pip3 install -r yolov8_ros/requirements.txt
sudo apt install ros-humble-vision-msgs

# build
cd ~/ros2_ws
colcon build --symlink-install --packages-select yolov8_msgs
colcon build --symlink-install --packages-up-to yolov8_bringup
```
---

## 2. 시뮬레이션 환경을 열고 토픽을 확인합니다. 또한 토픽의 타입이 sensor_msgs/msg/Image인지 확인합니다.

# Terminal 1
은 그냥 scout_mini 실행시키면 됨

# Terminal 2
``` bash
ros2 topic list
```

/depth_camera/image_raw 등 이름을 확인하고 
```bash
ros2 topic info /depth_camera/image_raw
```
Type: sensor_msgs/msg/Image 등 확인
---

---
```bash
ros2 launch yolov8_bringup yolov8.launch.py device:=cpu
```
---