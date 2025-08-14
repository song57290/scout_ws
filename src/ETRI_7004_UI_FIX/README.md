# 로봇 제어 및 모니터링 시스템

## 프로젝트 개요

이 프로젝트는 ROS(Robot Operating System)를 기반으로 한 로봇 제어 및 모니터링 시스템입니다. 웹 인터페이스를 통해 로봇의 이동, 카메라 제어, 센서 데이터 시각화 등을 제공합니다.

## 디렉토리 구조

### 📁 routes

서버 측 라우팅을 관리하는 디렉토리입니다.

- `controlling/`: 로봇 제어 관련 라우트
  - `controlling.js`: 제어 페이지 및 RTSP 스트림 관리
  - `modules/`: ONVIF 카메라 제어 등 부가 기능
- `monitoring/`: 모니터링 관련 라우트
- `setting/`: 설정 관련 라우트

### 📁 public

클라이언트 측 코드를 포함하는 디렉토리입니다.

#### controlling/

- `src/`

  - `cameras/`: ROS 및 RTSP 카메라 디스플레이 모듈
  - `clientSet/`: ROS 연결 및 클라이언트 설정
  - `joysticks/`: 게임패드 기반 로봇 제어
  - `LiDAR/`: 2D LiDAR 데이터 시각화
  - `map/`: 지도 시각화 및 로봇 위치 표시
  - `robotStates/`: 로봇 상태(배터리 등) 표시
  - `utills/`: 유틸리티 함수들

- `styles/`
  - `base/`: 기본 스타일
  - `components/`: UI 컴포넌트 스타일
  - `layout/`: 레이아웃 스타일

#### monitoring/

- 다중 RTSP 스트림 모니터링 인터페이스

#### setting/

- 로봇 및 시스템 설정 인터페이스

## 주요 기능

1. 로봇 제어

   - 게임패드를 통한 이동 제어
   - ONVIF 카메라 팬/틸트/줌 제어
   - 실시간 위치 추적

2. 센서 데이터 시각화

   - 2D LiDAR 스캔 데이터
   - 로봇 위치 및 방향
   - 배터리 상태

3. 카메라 스트림

   - ROS 카메라 피드
   - RTSP 스트림
   - 듀얼 뷰 지원

4. 사용자 인터페이스
   - 반응형 사이드바
   - 다중 로봇 선택
   - 실시간 상태 표시

## 기술 스택

- Frontend: HTML, CSS, JavaScript
- Backend: Node.js, Express
- Robot: ROS, ROSLIB
- 카메라: ONVIF, RTSP
- 통신: WebSocket, Socket.IO

## 설치 및 실행

1. 의존성 설치

npm install
