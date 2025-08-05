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

## 2. 키보드 제어
---
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
---

