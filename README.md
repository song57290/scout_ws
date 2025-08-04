# scout_ws

## 1. gazebo
```bash
cd ~/scout_ws
colcon build
source install/setup.bash
# source /usr/share/gazebo-11/setup.bash 필요시
```
```bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
```

## 2. 키보드 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

