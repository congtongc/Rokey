# 터틀봇3 라인 트래킹 및 매니퓰레이터 제어 프로젝트

이 프로젝트는 터틀봇3를 이용한 자율주행 라인 트래킹과 아루코 마커 인식을 통한 매니퓰레이터 제어 시스템을 구현합니다.

## 프로젝트 개요

본 프로젝트는 ROS2 기반으로 개발되었으며, 다음과 같은 주요 기능을 포함합니다:

1. **라인 트래킹 자율주행**: 카메라를 통해 인식된 차선을 따라 자율주행
2. **아루코 마커 인식**: 카메라로 아루코 마커를 인식하여 위치 및 자세 추정
3. **매니퓰레이터 제어**: 인식된 아루코 마커를 기준으로 물체 집기/놓기 작업 수행

## 주요 기능

### 라인 트래킹 (Lane Tracking)

라인 트래킹 시스템은 다음과 같은 단계로 동작합니다:

1. **이미지 전처리**: 카메라 이미지 보정 및 노이즈 제거
2. **차선 검출**: 색상 필터링을 통한 흰색/노란색 차선 검출
3. **Bird's Eye View 변환**: 원근 변환을 통한 탑뷰 이미지 생성
4. **제어 신호 생성**: 차선 중앙과 로봇 위치를 기반으로 한 제어 명령 생성

주요 노드:
- `image_compensation.py`: 카메라 이미지 보정 및 색상 필터링
- `detect_bev.py`: Bird's Eye View 변환 및 차선 검출
- `control_lane.py`: 검출된 차선 정보를 바탕으로 주행 제어

### 아루코 마커 인식 (ArUco Marker Detection)

아루코 마커 인식 시스템은 다음과 같은 기능을 제공합니다:

1. **마커 검출**: OpenCV ArUco 라이브러리를 이용한 마커 검출
2. **위치 및 자세 추정**: 카메라 캘리브레이션 정보를 이용한 3D 위치 및 자세 추정
3. **마커 정보 발행**: 검출된 마커의 ID, 위치, 방향 정보를 ROS 토픽으로 발행

주요 노드:
- `turtlebot_aruco.py`: 아루코 마커 검출 및 위치 추정 및 터틀봇과 아루코 마커 연동

### 매니퓰레이터 제어 (Manipulator Control)

매니퓰레이터 제어 시스템은 다음과 같은 기능을 수행합니다:

1. **Pick & Place 작업**: 아루코 마커로 식별된 물체 집기/놓기
2. **경로 계획**: MoveIt 프레임워크를 이용한 충돌 없는 경로 계획
3. **그리퍼 제어**: 물체 파지를 위한 그리퍼 제어

주요 노드:
- `pick_and_place.py`: 아루코 마커 인식 시 자동화된 Pick & Place 작업 수행

## 시스템 요구사항

- ROS2 Humble
- Ubuntu 22.04
- Python 3.10 이상
- OpenCV 4.5 이상
- 터틀봇3 하드웨어 (Waffle Pi 모델 권장)
- OpenMANIPULATOR-X

## 설치 및 실행 방법

### 의존성 설치

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-msgs ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
sudo apt install ros-humble-moveit ros-humble-joint-state-publisher-gui
pip install opencv-contrib-python numpy
```

### 워크스페이스 설정

```bash
mkdir -p ~/rokey_project/Rokey/Lane_tracking_auto_driving/src
cd ~/rokey_project/Rokey/Lane_tracking_auto_driving
colcon build
source install/setup.bash
```

### 카메라 연결 실행
```bash
ros2 run turtlebot3_autorace_camera img_publish     # 카메라 이미지 
ros2 run turtlebot3_autorace_camera image_compensation  # 이미지 전처리
```

### 라인 트래킹 실행

```bash
ros2 run turtlebot3_autorace_detect detect_lane     # 라인 인식
```

### 로봇 제어 실행
```bash
ros2 run turtlebot3_autorace_detect detect_stop_line    # 신호등 앞 정지
ros2 run turtlebot3_autorace_driving control_lane   # 라인 인식 자율 주행
```

### 아루코 마커 인식 실행

```bash
ros2 run aruco_yolo turtlebot_aruco     # 아루코 마커 인식
```

### Pick & Place 실행

```bash
ros2 run aruco_yolo pick_and_place      # 아루코 마커 인식 기반 메니퓰레이터 제어
```

## 주요 토픽

### 라인 트래킹 관련 토픽
- `/image_compensated/compressed`: 보정된 카메라 이미지
- `/lane_center`: 차선 중앙 위치 정보
- `/control/cmd_vel`: 로봇 제어 명령

### 아루코 마커 관련 토픽
- `/detected_markers`: 검출된 아루코 마커 정보
- `/camera/image_raw/compressed`: 카메라 원본 이미지

## 시스템 아키텍처

본 프로젝트는 모듈화된 구조로 설계되어 있으며, 각 기능은 독립적인 ROS 노드로 구현되어 있습니다. 노드 간 통신은 ROS 토픽과 서비스를 통해 이루어집니다.
