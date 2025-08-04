# scout_ws

## 1. rviz
```bash
cd ~/scout_ws
source install/setup.bash
```
```bash
ros2 launch scout_description display_mini.launch.py
```

## 2. gazebo
```bash
cd ~/scout_ws
source install/setup.bash 
source /usr/share/gazebo-11/setup.bash
```
```bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
```

## 3. 키보드 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

