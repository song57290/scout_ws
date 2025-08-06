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



| 시나리오                                      | 권장 파일                           |
| ----------------------------------------- | ------------------------------- |
| **“빨리 돌려서 네비·알고리즘만 시험”** (물리 세부치 중요 X)    | `mini.xacro` (가볍고 단순)           |
| **“실제 Scout Mini 와 동일한 운동·무게 중심을 보고 싶다”** | `scout_mini.xacro` (정확한 CAD·관성) |
