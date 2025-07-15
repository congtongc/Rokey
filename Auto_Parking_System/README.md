# 로봇 자동화 주차 시스템 (Automated Robot Parking System)

## 프로젝트 소개

본 프로젝트는 ROS2 기반의 자율주행 로봇을 활용한 실내 주차 관리 시스템입니다. 두 대의 Turtlebot4와 OAK-D 카메라를 이용하여 주차 공간 인식, 차량 번호판 인식, 자율주행을 구현했습니다. 주차와 출차를 각각 독립된 로봇(robot2: 주차, robot3: 출차)이 담당하여 효율적인 주차장 운영이 가능합니다.

## 시스템 아키텍처

### ROS2 노드 구조

```
                                [하드웨어 계층]
                    Turtlebot4 ←→ OAK-D Camera ←→ Sensors
                                      ↕
                                [ROS2 노드 계층]
            ┌─────────────┬─────────────┬────────────┬──────────┐
     YOLO Detection   Nav2 Control   OCR Process   GUI Node   Database
            ↓             ↓             ↓            ↓          ↓
     Object Detection  Path Planning  Plate Reading  UI Logic   Data Store
```

### ROS2 토픽 구조

```
[주차 시스템 토픽 (robot2)]
/robot2/oakd/rgb/image_raw/compressed  # 압축 RGB 이미지
/robot2/oakd/stereo/image_raw          # 깊이 이미지
/robot2/oakd/stereo/camera_info        # 카메라 캘리브레이션
/robot2/detect/yolo_distance_image     # YOLO 인식 결과 이미지
/robot2/detect/object_info             # 인식된 객체 정보
/robot2/danger_state                   # 위험 상태 알림
/robot2/parking/location               # 주차 위치 정보
/robot2/carplate/ocr_result            # 번호판 인식 결과
/robot2/cmd_vel                        # 로봇 속도 제어
/robot2/cmd_audio                      # 오디오 피드백

[출차 시스템 토픽 (robot3)]
/robot3/oakd/rgb/image_raw/compressed  # 압축 RGB 이미지
/robot3/oakd/stereo/image_raw          # 깊이 이미지
/robot3/oakd/stereo/camera_info        # 카메라 캘리브레이션
/robot3/detect/yolo_distance_image     # YOLO 인식 결과 이미지
/robot3/detect/object_info             # 인식된 객체 정보
/robot3/danger_state                   # 위험 상태 알림
/robot3/parking/location               # 주차 위치 정보
/robot3/carplate/ocr_result            # 번호판 인식 결과
/robot3/cmd_vel                        # 로봇 속도 제어
/robot3/cmd_audio                      # 오디오 피드백
```

## 주요 기능 및 구현 방식

### 1. 컴퓨터 비전 시스템

- **주차 공간 인식 (`yolo_detect.py`)**

  - YOLOv8 커스텀 모델 사용 (학습 데이터: Roboflow)
  - OAK-D 스테레오 카메라의 깊이 정보 활용
  - 실시간 장애물 감지 및 위험 상황 알림 (`/danger_state` 토픽)
  - 10Hz 주기로 이미지 처리 및 결과 발행
  - 위험 감지 임계값(DANGER_THRESHOLD) 기반 안전 관리
- **차량 번호판 인식 (`detect_car_info2.py`)**

  - YOLOv8 + EasyOCR 조합
  - 실시간 이미지 처리 (5Hz)
  - 압축/비압축 이미지 모두 지원
  - 한글 번호판 특화 처리
  - 디버그용 이진화 이미지 발행

### 2. 자율주행 시스템

- **입차 시스템 (`sc_follow_waypoints2_1.py`)**

  - Nav2 기반 경로 계획 및 실행
  - 사전 정의된 주차 좌표 맵 활용

  ```python
  location_map = {
      "A-1": (-2.35, -5.01, -90.0),  # 일반 차량
      "A-2": (-1.32, -5.15, -90.0),
      "B-1": (1.03, -2.06, 0.0),     # 전기 차량
      "B-2": (0.94, -1.10, 0.0),
      "C-1": (-2.95, -3.54, 180.0),  # 장애인 차량
      "C-2": (-3.04, -4.59, 180.0),
  }
  ```

  - TF 기반 좌표 변환 및 위치 추적
  - 실시간 피드백 처리
- **출차 시스템 (`sc_follow_waypoints.py`)**

  - 웨이포인트 기반 내비게이션
  - 도킹/언도킹 자동화
  - 실시간 피드백 처리
  - 장애물 회피 및 안전 정지

### 3. GUI 시스템

- **기술 스택**
  - PyQt5 기반 사용자 인터페이스
  - ROS2 토픽 기반 실시간 데이터 통신
  - 멀티스레딩 구조 (GUI, ROS2, 데이터 처리)

#### 3.1 메인 주차 관리 GUI (`parking_gui.py`)

- **기술 스택**

  - PyQt5 기반 사용자 인터페이스
  - ROS2 토픽 기반 실시간 데이터 통신
  - 멀티스레딩 구조 (GUI, ROS2, 데이터 처리)
- **주요 기능**

  - 3단계 화면 구조
    - 시작 화면: 주차/출차 모드 선택
    - 주차 화면: 주차 프로세스 관리
    - 출차 화면: 출차 프로세스 관리
  - 실시간 데이터 관리
    - 주차장 상태 정보 관리
    - 차량별 주차 이력 추적
    - 실시간 위치 정보 업데이트
  - 차량 타입별 주차 공간 관리
    - 일반 차량: A-1, A-2
    - 전기 차량: B-1, B-2
    - 장애인 차량: C-1, C-2
  - ROS2 토픽 통신
    - 카메라 피드 구독
    - OCR 결과 처리
    - 주차 위치 명령 발행

#### 3.2 주차 관리자용 GUI (`parking_manager_gui.py`)

- **기술 스택**

  - PyQt5 기반 GUI 프레임워크
  - YOLO v8 객체 감지 모델 통합
  - OpenCV 기반 실시간 영상 처리
  - 멀티스레드 카메라 스트림 처리
- **주요 기능**

  - 실시간 CCTV 모니터링 (2채널)
    - CCTV1 (camera1=4): 주차장 전체 뷰
    - CCTV2 (camera2=2): 입구 뷰
  - YOLO 기반 실시간 차량 감지
    - 모델: car_detect.pt
    - 신뢰도 임계값: 0.85
    - 감지 클래스: 'cars'
  - 3단계 탭 기반 인터페이스
    - 메인 시작 화면
    - 주차장 관리 탭
    - 차량 감지 탭
  - 실시간 알림 및 상태 표시

### 4. 데이터 관리 시스템 (`db_manager.py`)

- **기술 스택**

  - InfluxDB Cloud 3.0
  - Python InfluxDB 클라이언트 3.0
  - Pandas 데이터 처리
  - 환경 변수 기반 설정 관리 (.env)
- **주요 기능 및 쿼리**

  - 실시간 주차 상태 관리
    ```sql
    WITH latest_status AS (
        SELECT license_plate, MAX(time) as latest_time
        FROM parking
        WHERE time >= now() - INTERVAL '30 days'
        GROUP BY license_plate
    )
    SELECT p.license_plate, p.car_type, p.location, p.status, p.time
    FROM parking p
    JOIN latest_status ls 
    ON p.license_plate = ls.license_plate AND p.time = ls.latest_time
    WHERE p.status = 'parked'
    ORDER BY p.time DESC
    ```
  - 차량 입/출차 기록
    - Line Protocol 형식 데이터 저장
    - RFC3339 타임스탬프 사용
    - 실시간 상태 업데이트
  - 주차 이력 추적
    - 최근 7일 출차 기록 조회 (기본값)
    - 실시간 위치 기반 조회
    - 부분 번호판 검색 지원
  - 통계 데이터 생성
    - 차량 타입별 집계
    - 위치별 점유율 분석
  - 차량 타입별 주차 공간 검증
    - 일반 차량: A-1, A-2
    - 전기 차량: B-1, B-2
    - 장애인 차량: C-1, C-2
  - 데이터 구조
    ```
    Measurement: parking
    Tags: 
      - license_plate (차량 번호)
      - car_type (차량 타입)
      - location (주차 위치)
    Fields:
      - status (parked/exit)
    Timestamp: Unix timestamp
    ```

### 5. 좌표 변환 시스템 (`detect_ps_front.py`)

- **카메라-맵 좌표 변환 (`detect_ps_front.py`)**
  - TF2 기반 좌표계 변환
  - 3D 포인트 클라우드 처리
  - 마커 기반 시각화
  - 실시간 위치 추적

### 6. 오디오 피드백 시스템 (`beep.py`)

- 위험 상황 감지 시 경고음 발생
- 상태 기반 사운드 패턴 재생
- 비동기 오디오 처리
- 주파수 기반 경고음 패턴
  ```python
  AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000))
  ```

## 시스템 요구사항

### 하드웨어

- Turtlebot4 2대 (주차용 1대, 출차용 1대)
- OAK-D 스테레오 카메라 2대
- 호스트 PC (CUDA 지원 필요)

### 소프트웨어

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- CUDA 지원 환경 (YOLOv8용)
- InfluxDB Cloud 계정

## 설치 방법

### 1. ROS2 설치

```bash
# ROS2 Humble 설치 가이드 참조
https://docs.ros.org/en/humble/Installation.html
```

### 2. 의존성 패키지 설치

```bash
# ROS2 패키지
sudo apt update
sudo apt install ros-humble-nav2* ros-humble-turtlebot4*

# Python 패키지
pip install ultralytics easyocr torch torchvision
pip install pyqt5 opencv-python-headless
pip install influxdb-client-3 pandas python-dotenv

# 한글 폰트 설치 (OCR용)
sudo apt install fonts-noto-cjk
```

### 3. 환경 변수 설정

```bash
# .env 파일 생성
cat << EOF > .env
INFLUXDB_ORG=project
INFLUXDB_TOKEN=your_token
INFLUXDB_HOST=your_host
INFLUXDB_DATABASE=parking
EOF
```

### 4. 프로젝트 설정

```bash
# 워크스페이스 생성
mkdir -p ~/rokey_ws/src
cd ~/rokey_ws/src
git clone <repository_url>

# 빌드
cd ~/rokey_ws
colcon build
source install/setup.bash
```

## 실행 방법

### 1. 기본 시스템 실행

```bash
# 터미널 1: 주차 시스템 (robot2) 카메라 및 기본 시스템
ros2 launch rokey_pjt robot_bringup.launch.py namespace:=/robot2

# 터미널 2: 출차 시스템 (robot3) 카메라 및 기본 시스템
ros2 launch rokey_pjt robot_bringup.launch.py namespace:=/robot3

# 터미널 3: 주차 YOLO 감지
ros2 run rokey_pjt yolo_detect --ros-args -r __ns:=/robot2

# 터미널 4: 출차 YOLO 감지
ros2 run rokey_pjt yolo_detect --ros-args -r __ns:=/robot3

# 터미널 5: 메인 GUI
ros2 run rokey_pjt parking_gui

# 터미널 6: 관리자 GUI (CCTV 모니터링)
ros2 run rokey_pjt parking_manager_gui
```

### 2. 자율주행 시스템 실행

```bash
# 터미널 7: 주차 Nav2 실행
ros2 launch nav2_bringup navigation_launch.py namespace:=/robot2

# 터미널 8: 출차 Nav2 실행
ros2 launch nav2_bringup navigation_launch.py namespace:=/robot3

# 터미널 9: 주차 SLAM
ros2 launch slam_toolbox online_async_launch.py namespace:=/robot2

# 터미널 10: 출차 SLAM
ros2 launch slam_toolbox online_async_launch.py namespace:=/robot3

# 터미널 11: 입차 시스템
ros2 run rokey_pjt sc_follow_waypoints2_1 --ros-args -r __ns:=/robot2

# 터미널 12: 출차 시스템
ros2 run rokey_pjt sc_follow_waypoints --ros-args -r __ns:=/robot3
```

### 3. 비전 시스템 실행

```bash
# 터미널 13: 주차 번호판 인식 (압축)
ros2 run rokey_pjt detect_car_info2 --ros-args -r __ns:=/robot2 --mode compressed

# 터미널 14: 출차 번호판 인식 (압축)
ros2 run rokey_pjt detect_car_info2 --ros-args -r __ns:=/robot3 --mode compressed

# 터미널 15: 주차 좌표 변환
ros2 run rokey_pjt detect_ps_front --ros-args -r __ns:=/robot2

# 터미널 16: 출차 좌표 변환
ros2 run rokey_pjt detect_ps_front --ros-args -r __ns:=/robot3
```

### 4. 유틸리티 실행

```bash
# 터미널 17: 주차 오디오 피드백
ros2 run rokey_pjt beep --ros-args -r __ns:=/robot2

# 터미널 18: 출차 오디오 피드백
ros2 run rokey_pjt beep --ros-args -r __ns:=/robot3

# 터미널 19: RViz2 시각화 (주차)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rokey_pjt)/share/rokey_pjt/rviz/nav2_config.rviz --ros-args -r __ns:=/robot2

# 터미널 20: RViz2 시각화 (출차)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rokey_pjt)/share/rokey_pjt/rviz/nav2_config.rviz --ros-args -r __ns:=/robot3
```

### 5. 시스템 모니터링

```bash
# 토픽 목록 확인
ros2 topic list

# 주차 시스템 토픽 모니터링
ros2 topic echo /robot2/parking/location
ros2 topic echo /robot2/carplate/ocr_result
ros2 topic echo /robot2/danger_state

# 출차 시스템 토픽 모니터링
ros2 topic echo /robot3/parking/location
ros2 topic echo /robot3/carplate/ocr_result
ros2 topic echo /robot3/danger_state

# 노드 상태 확인
ros2 node list

# TF 트리 확인
ros2 run tf2_tools view_frames
```

### 6. 전체 시스템 실행

```bash
# 주차 시스템 실행
ros2 launch rokey_pjt full_system.launch.py namespace:=/robot2

# 출차 시스템 실행
ros2 launch rokey_pjt full_system.launch.py namespace:=/robot3
```

### 주의사항

- 주차 시스템은 namespace:=/robot2, 출차 시스템은 namespace:=/robot3 사용
- YOLO 모델 파일 위치:
  - 주차 공간 감지: model/park_area.pt
  - 번호판 인식: model/car_plate2.pt
  - 차량 감지: model/car_detect.pt
- 실행 순서 준수:
  1. 기본 시스템 (카메라, YOLO)
  2. Nav2 및 SLAM
  3. 비전 시스템
  4. GUI 및 유틸리티
- 환경 설정:
  - config/.bashrc 파일에서 ROS2 및 시스템 환경 변수 설정
  - 실행 전 반드시 source config/.bashrc 실행

## 프로젝트 구조

```
Auto_Parking_System/
├── model/                # YOLO 모델 파일
│   ├── park_area.pt         # 주차 공간 감지 모델
│   ├── car_plate2.pt        # 번호판 인식 모델
│   └── car_detect.pt        # 차량 감지 모델
│
├── config/               # 설정 파일
│   └── .bashrc              # ROS2 및 시스템 환경 설정
│
├── docs/                # 문서
│   ├── README.md           # 문서 가이드
│   ├── bash.md             # 터미널 명령어 가이드
│   ├── shortcut/           # 단축키 가이드
│   └── 우분투 설치/         # 우분투 설치 가이드
│
├── rokey_ws/            # ROS2 워크스페이스
│   └── src/
│       └── rokey_pjt/      # 메인 프로젝트 패키지
│           ├── vision/     # 컴퓨터 비전 관련
│           ├── navigation/ # 자율주행 관련
│           ├── gui/        # GUI 관련
│           ├── database/   # 데이터 관리
│           └── utils/      # 유틸리티
│
├── .vscode/             # VS Code 설정
└── bash_command.md      # 자주 사용하는 명령어 모음
```

## 참고 자료

- [강의 자료](https://rainy-cream-352.notion.site/_-1fb450ef7c598037ba7fe0880210baa2)
- [Roboflow 데이터셋](https://app.roboflow.com/rokeyproject-4)
- [ROS2 Nav2 문서](https://navigation.ros.org/)
- [Turtlebot4 매뉴얼](https://turtlebot.github.io/turtlebot4-user-manual/)
- [InfluxDB 문서](https://docs.influxdata.com/)

## 시연 영상




## 설명 자료
