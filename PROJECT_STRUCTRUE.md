# 📁 Enhanced Dobot Robot System - 프로젝트 구조

## 🏗️ 전체 구조 개요

```
enhanced_dobot_system/
├── 📄 main.py                    # 메인 애플리케이션 진입점
├── ⚙️ config.py                 # 시스템 설정 및 상수 정의
├── 📝 logger_setup.py           # 로깅 시스템 설정
├── 🔧 utils.py                  # 유틸리티 함수 및 에러 클래스
├── 🤖 robot_controller.py       # 로봇 제어 핵심 클래스 (버그 수정됨)
├── 👁️ yolo_detector.py          # YOLO 객체 인식 시스템
├── 🖼️ ui_components.py          # GUI 컴포넌트들
├── 📦 requirements.txt          # 필요한 패키지 목록
├── 🛠️ setup.py                  # 자동 설치 스크립트
├── 🧪 test_system.py            # 시스템 테스트 스크립트
├── 📖 README.md                 # 사용자 가이드
├── 📋 PROJECT_STRUCTURE.md      # 이 파일 (프로젝트 구조 설명)
└── 📂 자동 생성 폴더들/
    ├── logs/                    # 로그 파일들
    │   ├── robot_system.log     # 시스템 로그
    │   └── order_log.txt        # 주문 로그
    ├── fonts/                   # 폰트 파일들 (선택사항)
    │   └── NanumGothic.ttf      # 한글 폰트 (수동 추가)
    ├── models/                  # YOLO 모델 캐시
    └── data/                    # 데이터 파일들
```

## 📄 각 파일별 상세 설명

### 🎯 main.py - 메인 애플리케이션
**역할**: 전체 시스템의 진입점 및 GUI 메인 클래스
**주요 기능**:
- GUI 인터페이스 설정 및 관리
- 사용자 이벤트 처리
- 로봇 픽업 시퀀스 조정
- 카메라/YOLO 통합 관리

**핵심 클래스**:
```python
class FurnitureOrderSystem:
    # 메인 애플리케이션 클래스
    # GUI 설정, 이벤트 처리, 시스템 통합
```

### ⚙️ config.py - 설정 관리
**역할**: 시스템 전반의 설정값과 상수 정의
**주요 내용**:
- 로봇 연결 설정 (IP, 포트 등)
- 작업 공간 제한값
- 가구 정보 딕셔너리
- UI 색상 테마
- 의존성 패키지 확인

**핵심 설정**:
```python
@dataclass
class RobotConfig:
    ip_address: str = "192.168.1.6"
    movement_timeout: float = 30.0
    safety_height_offset: float = 50.0

FURNITURE_INFO = {
    "소파": {"position": [290, 218, -129, -59], ...},
    # 기타 가구들...
}
```

### 📝 logger_setup.py - 로깅 시스템
**역할**: 안전한 로깅 시스템 구현 (Windows 한글/이모지 지원)
**주요 기능**:
- UTF-8 안전 로깅
- 콘솔 출력 인코딩 처리
- 순환 로그 파일 관리
- 주문 로그 별도 관리

**핵심 클래스**:
```python
class SafeConsoleHandler(logging.StreamHandler):
    # Windows 콘솔 안전 출력

class OrderLogger:
    # 주문 처리 로그 전용 관리
```

### 🔧 utils.py - 유틸리티 함수들
**역할**: 공통 유틸리티 함수 및 에러 클래스 정의
**주요 기능**:
- 위치 좌표 유효성 검증
- 안전한 데이터 변환
- 폰트 관리 함수들
- 커스텀 예외 클래스들

**핵심 함수**:
```python
def validate_position(position: List[float]) -> bool:
    # 로봇 작업 공간 내 위치 검증

class RobotMovementError(RobotSystemError):
    # 로봇 이동 관련 예외
```

### 🤖 robot_controller.py - 로봇 제어 (버그 수정됨)
**역할**: Dobot 로봇 제어 핵심 로직
**주요 수정사항**:
- ✅ 타임아웃 데코레이터 오류 수정
- ✅ None 객체 접근 에러 해결
- ✅ 에러 모니터링 개선
- ✅ 스레드 안전성 향상

**핵심 클래스**:
```python
class RobotController:
    def move_to_position(self, position: List[float]) -> bool:
        # 수정된 타임아웃 처리로 안정성 향상
    
    def _wait_arrive(self, target: List[float]):
        # None 체크 추가로 에러 방지
```

### 👁️ yolo_detector.py - 객체 인식
**역할**: YOLOv8 기반 실시간 객체 인식
**주요 기능**:
- YOLOv8 모델 로드 및 관리
- 실시간 객체 탐지
- 커스텀 라벨 지원
- 인식 결과 시각화

**핵심 클래스**:
```python
class YOLODetector:
    def detect_objects(self, frame, confidence: float = 0.5):
        # 프레임별 객체 인식 수행
```

### 🖼️ ui_components.py - GUI 컴포넌트
**역할**: 재사용 가능한 GUI 컴포넌트들
**주요 컴포넌트**:
- `RobotArmVisualizer`: 3D 로봇팔 시각화
- `CameraDisplay`: 카메라 피드 표시
- `LogDisplay`: 로그 메시지 출력
- `DetectionDisplay`: 객체 인식 결과 표시

### 🛠️ setup.py - 자동 설치
**역할**: 시스템 자동 설치 및 환경 설정
**주요 기능**:
- Python 버전 확인
- 필수 패키지 자동 설치
- YOLO 모델 다운로드
- 디렉터리 구조 생성
- 기본 테스트 실행

### 🧪 test_system.py - 시스템 테스트
**역할**: 종합적인 시스템 테스트
**테스트 범위**:
- 각 모듈 단위 테스트
- 시스템 통합 테스트
- 성능 테스트
- 스트레스 테스트

## 🔄 데이터 흐름

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│   main.py   │───▶│ robot_       │───▶│   Dobot     │
│  (GUI 이벤트) │    │ controller.py│    │   Robot     │
└─────────────┘    └──────────────┘    └─────────────┘
       │                                      ▲
       ▼                                      │
┌─────────────┐    ┌──────────────┐          │
│  yolo_      │───▶│ ui_components│          │
│  detector.py│    │ .py (시각화)  │          │
└─────────────┘    └──────────────┘          │
       ▲                   │                 │
       │                   ▼                 │
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│   Camera    │    │ logger_setup │    │   config.py │
│   Feed      │    │ .py (로깅)    │───▶│  (설정값)    │
└─────────────┘    └──────────────┘    └─────────────┘
```

## 🚀 실행 순서

1. **시스템 초기화**
   ```python
   # config.py: 의존성 확인 및 설정 로드
   check_dependencies()
   
   # logger_setup.py: 로깅 시스템 초기화
   setup_logging()
   ```

2. **메인 앱 시작**
   ```python
   # main.py: GUI 및 로봇 컨트롤러 초기화
   app = FurnitureOrderSystem(root)
   ```

3. **사용자 상호작용**
   ```python
   # 가구 버튼 클릭 → 픽업 시퀀스 실행
   execute_pickup_sequence(furniture_name)
   
   # 카메라 버튼 클릭 → YOLO 객체 인식 시작
   toggle_camera()
   ```

## 🛡️ 에러 처리 계층

```
┌─────────────────────────────────┐
│        GUI 레벨 에러 처리         │ ← messagebox, 사용자 알림
├─────────────────────────────────┤
│       애플리케이션 레벨 에러       │ ← try/catch, 로깅
├─────────────────────────────────┤
│        로봇 컨트롤러 에러         │ ← 커스텀 예외, 안전 정지
├─────────────────────────────────┤
│         시스템 레벨 에러          │ ← 의존성 체크, 리소스 관리
└─────────────────────────────────┘
```

## 📊 성능 최적화 포인트

### 🏃‍♂️ 속도 최적화
- **로봇 이동**: 타임아웃 기반 비동기 처리
- **YOLO 인식**: 프레임 스킵 및 해상도 조절
- **GUI 업데이트**: 메인 스레드 분리

### 🧠 메모리 최적화
- **이미지 처리**: PIL/OpenCV 메모리 해제
- **로그 관리**: 순환 로그로 디스크 공간 절약
- **모델 캐시**: YOLO 모델 재사용

### 🔒 안정성 개선
- **에러 복구**: 자동 재연결 및 상태 복원
- **데이터 검증**: 입력값 사전 검증
- **리소스 정리**: 프로그램 종료 시 안전한 리소스 해제

## 🎯 확장 가능 포인트

### 🔧 새로운 기능 추가 시
1. **새로운 가구 타입**: `config.py`의 `FURNITURE_INFO`에 추가
2. **새로운 로봇 동작**: `robot_controller.py`에 메서드 추가
3. **새로운 UI 컴포넌트**: `ui_components.py`에 클래스 추가

### 🌐 시스템 통합 시
1. **데이터베이스 연동**: 주문 로그를 DB로 확장
2. **네트워크 통신**: 원격 제어 API 추가
3. **클라우드 연동**: YOLO 모델 클라우드 추론

---

**참고**: 각 파일은 독립적으로 테스트 가능하도록 설계되었으며, 의존성을 최소화하여 유지보수성을 높였습니다.
