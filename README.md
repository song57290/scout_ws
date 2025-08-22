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
ros2 launch scout_gazebo_sim office_scout_mini.launch.py use_dock:=true
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
cd ~/scout_ws/src/ETRI_7004_UI_FIX
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

## 1. yolov8_ros 패키지를 가져오고 빌드(사물 인식)
---
```bash
cd ~/scout_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git -b 2.2.1
pip3 install -r yolov8_ros/requirements.txt
sudo apt install ros-humble-vision-msgs

# build
cd ~/scout_ws
colcon build --symlink-install --packages-select yolov8_msgs
colcon build --symlink-install --packages-up-to yolov8_bringup
```
---

go2 부분 urdf도 확인
-> urdf
-> 초기위치 => params.yaml


PPT 참고
https://m.blog.naver.com/xiilab/223476760262
https://iambeginnerdeveloper.tistory.com/279





 cd src/ugv_sim/scout_yolov5/scripts/
python yolo_fire_detection.py

/home/etri/scout_ws/src/ugv_sim/scout_yolov5/scripts/utils/general.py:34: UserWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html. The pkg_resources package is slated for removal as early as 2025-11-30. Refrain from using this package or pin to Setuptools<81.
  import pkg_resources as pkg
YOLOv5 🚀 2025-8-11 Python-3.10.12 torch-2.5.1+cu121 CUDA:0 (NVIDIA GeForce RTX 4090, 24210MiB)

/home/etri/scout_ws/src/ugv_sim/scout_yolov5/scripts/models/experimental.py:79: FutureWarning: You are using `torch.load` with `weights_only=False` (the current default value), which uses the default pickle module implicitly. It is possible to construct malicious pickle data which will execute arbitrary code during unpickling (See https://github.com/pytorch/pytorch/blob/main/SECURITY.md#untrusted-models for more details). In a future release, the default value for `weights_only` will be flipped to `True`. This limits the functions that could be executed during unpickling. Arbitrary objects will no longer be allowed to be loaded via this mode unless they are explicitly allowlisted by the user via `torch.serialization.add_safe_globals`. We recommend you start setting `weights_only=True` for any use case where you don't have full control of the loaded file. Please open an issue on GitHub for any issues related to this experimental feature.
  ckpt = torch.load(attempt_download(w), map_location='cpu')  # load
Fusing layers... 
Model summary: 213 layers, 7012822 parameters, 0 gradients, 15.8 GFLOPs
[INFO]: Subscribed to image topic: /camera/image_raw
[INFO]: Publishing annotated to: /yolo/annotated_image/compressed
[INFO]: Using device: cuda:0


이렇게 출력 나오고 사진처럼 떠야함
