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


## 2. YOLO v5 실행

# Terminal 2
---
```bash
cd ~/scout_ws
colcon build
source ~/scout_ws/install/setup.bash
```

```bash
cd src/ugv_sim/scout_yolov5/scripts/
python yolo_fire_detection.py
```

## 3. 조이스틱 제어
---
```bash
cd ~/scout_ws/src/ETRI_7004_UI__FIX
npm start
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

go2 부분 urdf도 확인
-> urdf
-> 초기위치 => params.yaml