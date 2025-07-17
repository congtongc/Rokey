# 로봇 자동 주차 시스템 GUI (Android)

## 프로젝트 개요

이 프로젝트는 로봇 자동 주차 시스템의 안드로이드 GUI 애플리케이션입니다. 주차장 관리를 위한 ParkingApp과 차량 감지를 위한 DetectionApp으로 구성되어 있습니다.

## 현재 구현 상태

### 1. ParkingApp (주차 관리 앱)

- **구현 완료**:
  - MVVM 아키텍처 기반 구현
  - 주차장 현황 모니터링
  - 주차/출차 처리
  - 실시간 데이터 업데이트 (WebSocket)
  - 서버 자동 검색 (mDNS)
  - 차량 번호 검색 및 관리
  - 차량 번호 입력 시스템:
    - 단일 입력 필드로 통합
    - 자동 형식 검증 (2-3자리 숫자 + 1자리 한글 + 4자리 숫자)
    - Snackbar를 통한 직관적인 오류 안내 (예시 포함)
  - 상태 알림 시스템:
    - TextView 기반 상태 표시
    - 카메라 스트림, 주차 완료 등 주요 상태 메시지 표시

### 2. DetectionApp (차량 관리자 앱)

- **현재 상태**: Prototype
- **구현된 기능**:
  - 기본 UI 구조
  - 서버 통신 인터페이스
- **미구현 기능**:
  - 실제 카메라 연동
  - 차량 감지 및 번호판 인식

### 3. 브릿지 서버

- **구현 완료**:
  - FastAPI 기반 REST API
  - WebSocket 실시간 통신
  - InfluxDB 데이터 관리
  - 주차 데이터 처리 및 저장
- **미구현/테스트 대기**:
  - ROS2 노드 연동
  - Turtlebot4 제어
  - Oak-D 카메라 연동

## 시스템 아키텍처

### 1. 안드로이드 앱 (ParkingApp)

- **주요 컴포넌트**:
  - `MainActivity`: 앱의 메인 진입점
  - `HomeFragment`: 주차장 현황 및 통계
  - `ParkingFragment`: 주차 처리
  - `ExitFragment`: 출차 처리

### 2. 브릿지 서버

- **주요 컴포넌트**:
  - FastAPI 서버
  - InfluxDB 데이터베이스
  - WebSocket 서버
  - ROS2 브릿지 (준비 중)

## 데이터베이스 구조

### InfluxDB

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

## 설치 및 실행 방법

### 브릿지 서버 설정

1. Python 의존성 설치:

```bash
cd bridge_server
pip install -r requirements.txt
```

2. 환경 변수 설정:

- `bridge_server` 디렉토리에 `.env` 파일 생성:

```
INFLUXDB_ORG=your_org
INFLUXDB_TOKEN=your_token
INFLUXDB_HOST=your_host
INFLUXDB_DATABASE=your_database
```

3. 서버 실행:

```bash
python bridge_server.py
```

### 안드로이드 앱 빌드

1. Android Studio에서 프로젝트 열기
2. Gradle 동기화 실행
3. 앱 빌드 및 실행

## 향후 개발 계획

1. **DetectionApp 개발**:
   - 카메라 연동
   - 실시간 차량 감지

2. **로봇 시스템 연동**:
   - Turtlebot4 제어 구현
   - 자율 주행 테스트
   - 안전 시스템 구축

3. **시스템 안정화**:
   - 에러 처리 강화
   - 성능 최적화
   - 보안 강화

## 실행 영상

#### Bridge Server
https://github.com/user-attachments/assets/643d061c-0cae-4a43-85e1-6bc0246f93ed

#### Parking App
https://github.com/user-attachments/assets/5c02c93a-cd36-40d3-b18b-0f76ffae7506
