# Surgery assisted cooperative robot

- 음성 인식 기반 협동로봇 수술 보조 시스템

## 프로젝트 소개

- 이 프로젝트는 음성 명령을 통해 수술 도구 및 손, 수술 부위를 인식하고, 협동로봇이 인식된 수술 도구를 집어서 전달, 수술 부위 확대 등의 기능을 수행하는 시스템이다.  
- **ROS2 Humble(Ubuntu 22.04)** 기반으로 동작하며, **실시간 객체 인식(YOLO)**, **음성 인식(OpenAI, STT)**, **로봇 제어** 등 다양한 기술 요소들을 사용하였다.

## 주요 기능

- **음성 명령 인식**: 사용자의 자연어 명령에서 수술 도구 이름을 추출하여 로봇에 전달
- **객체 인식**: YOLO 기반으로 수술 도구, 손, 수술 부위를 실시간으로 인식
- **로봇 제어**: 인식된 객체의 3D 위치로 협동로봇을 이동 및 그리퍼로 집기/놓기 동작 등의 작업 수행
- **수술 부위 감지**: 수술 부위를 인식하여 특수 작업 가능
- **ROS2 서비스 구조**: 각 기능이 서비스로 분리되어 유연하게 연동 가능

## 폴더 구조
```
pick_and_place_voice/
├── robot_control/         # 로봇 제어 및 그리퍼 제어
│   ├── robot_control.py   # 수술 도구 전달 노드
│   └── onrobot.py         # 그리퍼(RG2) 제어
├── voice_processing/      # 음성 인식 및 명령 추출
│   ├── get_keyword.py     # 음성 명령에서 객체명 추출 (OpenAI 활용)
│   ├── stt.py             # STT(음성→텍스트)
│   ├── wakeup_word.py     # 웨이크업 워드 감지
│   └── MicController.py   # 마이크 제어
├── object_detection/      # 객체 인식(YOLO)
│   ├── detection.py       # 객체 인식 메인 노드
│   ├── yolo_surgical.py   # 수술 도구 인식
│   ├── yolo_hands.py      # 손 인식
│   ├── yolo_wound.py      # 수술 부위 인식
│   └── realsense.py       # 카메라 연동
├── detect_wound/          # 수술 부위 감지 서비스
│   └── detect_wound.py    # 수술 부위 감지 노드
├── resource/              # 모델, 파라미터, json 등 리소스
│   ├── *.pt, *.json, *.npy, *.tflite
├── setup.py, package.xml  # 패키지 설정 파일
└── README.md
```

## 설치 방법

1. 저장소 클론
```
  $ mkdir -p ~/ros2_ws/src
  $ cd ~/ros2_ws/src
  $ git clone https://github.com/congtongc/Rokey.git
```

2. 의존성 설치  
```
  $ cd ~/ros2_ws
  $ colcon build --symlink-install
  $ source install/setup.bash
```

3. 환경 변수 및 모델 파일 준비  
   - `resource/.env` 파일에 OpenAI API 키 입력 필요
   - `resource/` 폴더에 모델 파일(.pt, .tflite 등) 위치 확인

## 실행 방법

### 1. 필수 노드 (음성 인식 기반 동작에 반드시 필요)

**아래 두 노드는 음성 명령을 통한 객체 인식 및 명령 추출에 필수이므로 항시 실행**

#### (1) 객체 인식 노드 (카메라로 도구/손/수술 부위 인식)
```
$ ros2 run pick_and_place_voice object_detection
```

#### (2) 음성 명령 추출 노드 (명령어 인식 및 객체명 추출)
```
$ ros2 run pick_and_place_voice get_keyword
```

### 2. 선택 실행 노드 (상황에 따라 실행)

**아래 노드들은 수술 도구 전달 또는 수술 부위 감지 기능 중 필요에 따라 선택적으로 실행**

#### (1) 로봇 제어 노드 (수술 도구 전달 및 메인 작업)
```
$ ros2 run pick_and_place_voice robot_control
```

#### (2) 수술 부위 감지 노드 (수술 부위 인식 및 특수 작업)
```
$ ros2 run pick_and_place_voice detect_wound
```

## 전체적인 노드 설명 요약

| 노드명             | 필수/선택 | 주요 역할                        |
|--------------------|-----------|----------------------------------|
| object_detection   | 필수      | 객체(도구/손/수술 부위) 인식          |
| get_keyword        | 필수      | 음성 명령에서 객체명 추출        |
| robot_control      | 선택      | 수술 도구 전달 및 메인 작업             |
| detect_wound       | 선택      | 수술 부위 감지 및 특수 작업           |


### 마이크에 명령어를 말하면,  
**[필수 노드]** 가 **명령 추출 → 객체 인식 → 위치 인식**을 처리하고,  
**[선택 노드]** 를 실행하면 실제 로봇이 **수술 도구를 전달**하거나 **수술 부위 감지 기능**이 실행된다.

## 예시 명령어

1. **Hello Rokey** 로 웨이크업 워드 실행 후
2. **robot_control** 노드 실행 시
- "메스" → Scalpel 인식 후 집어서 전달
- "포셉" → Forceps 인식 후 집어서 전달
3. **detect_wound** 노드 실행 시
- "카메라" → wound 인식 및 특수 작업

## 학습 과정 및 세부 설명
- [B-2_수술 보조 로봇 프로젝트(협동2).pdf](https://github.com/user-attachments/files/20658700/B-2_.2.pdf)

## 전반적인 로봇의 동작 흐름
- https://github.com/user-attachments/assets/f4b465a0-58d0-449a-920e-7077362054ea
