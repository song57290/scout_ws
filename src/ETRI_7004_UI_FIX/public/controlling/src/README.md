# Source Directory

이 디렉토리는 프로젝트의 주요 소스 코드를 포함하고 있습니다. 각 파일과 디렉토리는 특정 기능을 구현하며, 프로젝트의 핵심 로직을 구성합니다.

## 주요 디렉토리 및 파일 설명

- **🔵 cameras**: 카메라 관련 디스플레이 모듈을 포함합니다.

  - **🟢 cameraRosDisplayer.js**: ROS 카메라 피드를 표시하는 모듈입니다.
  - **🟢 cameraRtspDisplayer.js**: RTSP 카메라 피드를 표시하는 모듈입니다.

- **🔵 clientSet**: 클라이언트 설정을 관리하는 모듈을 포함합니다.

  - **🟢 clientLoad.js**: 클라이언트 설정을 로드하는 기능을 제공합니다.
  - **🟢 clientSet.js**: 로봇의 설정을 관리하고 ROS 연결을 처리하는 SetClient 클래스를 정의합니다.

- **🔵 joysticks**: 조이스틱 컨트롤러를 포함합니다.

  - **🟢 joystickController.js**: JoystickController 클래스를 정의하여 게임패드를 통한 로봇 및 카메라 제어를 구현합니다.

- **🔵 LiDAR**: LiDAR 관련 디스플레이 모듈을 포함합니다.

  - **🟢 lidar2dDisplayer.js**: 2D LiDAR 데이터를 시각화하는 모듈입니다.

- **🔵 map**: 지도 관련 모듈을 포함합니다.

  - **🟢 map.js**: 지도 데이터를 처리하고 시각화하는 모듈입니다.

- **🔵 robotStates**: 로봇 상태 디스플레이 모듈을 포함합니다.

  - **🟢 robotStatesDisplayer.js**: 로봇의 상태 정보를 표시하는 모듈입니다.

- **🔵 utills**: 유틸리티 함수와 모듈을 포함합니다.

  - **🟢 cameraToggle.js**: 카메라 뷰를 전환하는 기능을 제공합니다.
  - **🟢 config.js**: 설정 관련 유틸리티를 제공합니다.
  - **🟢 mapToggle.js**: 지도 뷰를 전환하는 기능을 제공합니다.
  - **🟢 robotButtons.js**: 로봇 버튼을 생성하고 관리하는 기능을 제공합니다.
  - **🟢 sidebar.js**: 사이드바의 표시 및 숨김 상태를 관리합니다.
  - **🟢 fullScreenToggle.js**: 전체화면 버튼을 관리합니다.
