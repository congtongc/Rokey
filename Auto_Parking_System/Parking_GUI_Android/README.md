# 로봇 자동 주차 시스템 GUI (Android)

## 프로젝트 개요

이 프로젝트는 로봇 자동 주차 시스템의 안드로이드 GUI 애플리케이션입니다. ROS2 기반의 로봇 시스템과 통신하여 주차 상태를 모니터링하고 제어하는 기능을 제공합니다.

## 시스템 아키텍처

### 1. 안드로이드 앱 (ParkingApp)

- **MVVM 아키텍처** 기반의 안드로이드 애플리케이션
- 주요 컴포넌트:
  - `MainActivity`: 앱의 메인 진입점, 네비게이션 및 권한 관리
  - `HomeFragment`: 주차장 현황 및 통계 표시
  - `ParkingFragment`: 실시간 카메라 스트림 및 주차 처리
  - `ExitFragment`: 출차 처리
- **주요 기능**:
  - 실시간 주차 현황 모니터링
  - 차량 번호판 인식 결과 표시
  - 주차/출차 처리
  - 실시간 카메라 스트림 표시
  - 서버 자동 검색 (mDNS)

### 2. 브릿지 서버 (bridge_server)

- **FastAPI** 기반의 Python 서버
- ROS2와 안드로이드 앱 간의 통신 중계
- 주요 컴포넌트:
  - `BridgeNode`: ROS2 노드 구현
  - `DBManager`: InfluxDB 데이터베이스 관리
- **주요 기능**:
  - ROS2 토픽 구독 및 발행
  - REST API 엔드포인트 제공
  - WebSocket 기반 실시간 데이터 전송
  - 주차 데이터 관리 및 저장

### 3. 객체 감지 앱 (DetectionApp)

- 주차 공간 및 차량 감지
- 번호판 인식 (OCR) 처리

## 기술 스택

### 안드로이드 앱

- Kotlin
- Android Jetpack
  - Navigation Component
  - ViewModel & LiveData
  - DataBinding
- Retrofit2 & OkHttp3
- WebSocket
- CameraX
- Coroutines
- Material Design Components

### 브릿지 서버

- Python
- FastAPI
- ROS2
- OpenCV
- WebSocket
- InfluxDB
- Uvicorn
- ZeroConf (mDNS)

## 주요 기능

### 1. 주차 관리

- 실시간 주차 현황 모니터링
- 차량 타입별 주차 구역 관리 (일반/전기/장애인)
- 주차 위치 자동 할당
- 주차/출차 이력 관리

### 2. 실시간 모니터링

- 실시간 카메라 스트림
- 번호판 인식 결과 실시간 표시
- 주차장 통계 정보 표시
- WebSocket 기반 실시간 데이터 업데이트

### 3. 네트워크 통신

- REST API를 통한 데이터 통신
- WebSocket을 통한 실시간 데이터 스트리밍
- mDNS를 통한 서버 자동 검색
- ROS2 토픽 기반 로봇 시스템 통신

## 설치 및 실행 방법

### 브릿지 서버 설정

1. Python 의존성 설치:

```bash
cd bridge_server
pip install -r requirements.txt
```

2. 환경 변수 설정:

- `bridge_server` 디렉토리에 `.env` 파일을 생성하고 다음 환경 변수들을 설정:

```
INFLUXDB_ORG=your_org
INFLUXDB_TOKEN=your_token
INFLUXDB_HOST=your_host
INFLUXDB_DATABASE=your_database
```

> **주의**: `.env` 파일은 보안을 위해 Git에 포함되지 않습니다. 프로젝트 설정 시 직접 생성해야 합니다.

3. 브릿지 서버 실행:

```bash
python bridge_server.py
```

### 안드로이드 앱 빌드

1. Android Studio에서 ParkingApp 프로젝트 열기
2. Gradle 동기화 실행
3. 앱 빌드 및 실행

## 주의사항

- **ROS2** 환경이 필요합니다
- **브릿지 서버와 안드로이드 앱 간의 통신 기능은 현재 개발 중입니다 (통신 불가)**

## 데이터베이스 구조

### InfluxDB 설정

- **데이터베이스**: InfluxDB Cloud 사용
- **보안**: 환경 변수를 통한 접속 정보 관리
- **필수 환경 변수**:
  - `INFLUXDB_ORG`: 조직 이름
  - `INFLUXDB_TOKEN`: 인증 토큰
  - `INFLUXDB_HOST`: 데이터베이스 호스트
  - `INFLUXDB_DATABASE`: 데이터베이스 이름

### 스키마

- **Measurement**: parking
- **Tags**:
  - license_plate: 차량 번호판
  - car_type: 차량 타입 (normal/ev/disabled)
  - location: 주차 위치
- **Fields**:
  - status: 주차 상태 (parked/exit)

## 주차 위치 체계

- **일반 차량**: A-1, A-2
- **전기 차량**: B-1, B-2
- **장애인 차량**: C-1, C-2

## 실행 화면

### 홈 화면

- 주차장 현황 및 통계 정보 표시
- 현재 주차된 차량 목록

### 주차 화면

- 실시간 카메라 스트림
- 번호판 인식 결과
- 주차 위치 선택 및 처리

### 출차 화면

- 차량 번호 입력/인식
- 출차 처리 및 결과
