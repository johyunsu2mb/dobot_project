# Dobot Project

## 프로젝트 개요

이 프로젝트는 Dobot Magician 로봇팔을 제어하고 다양한 자동화 작업을 수행하기 위한 통합 시스템입니다. 컴퓨터 비전, 머신러닝, 그리고 로봇 제어 기술을 결합하여 지능형 로봇 시스템을 구현합니다.

## 시스템 아키텍처

### 핵심 구성 요소

1. **하드웨어 플랫폼**
   - Dobot Magician 데스크톱 로봇팔
   - 다양한 엔드 이펙터 (그리퍼, 흡착기, 펜 홀더)
   - RGB-D 카메라 (Intel RealSense D435i)
   - 제어 컴퓨터 (Windows/Linux/macOS 호환)

2. **소프트웨어 스택**
   - **통신 계층**: 시리얼/USB 통신 프로토콜
   - **제어 계층**: 역기구학 및 궤적 계획
   - **비전 계층**: 객체 인식 및 위치 추정
   - **AI 계층**: 언어 모델 기반 작업 계획

## 작동 원리

### 1. 통신 프로토콜

Dobot Magician은 시리얼 통신을 통해 제어됩니다:

```
PC ←→ USB/Serial ←→ Dobot Controller ←→ Servo Motors
```

- **통신 속도**: 115200 baud
- **프로토콜**: Dobot Communication Protocol v1.1.5
- **명령 구조**: Header + ID + Payload + Checksum

### 2. 좌표계 및 운동학

**좌표계 변환 과정:**
1. **카메라 좌표계** → **로봇 베이스 좌표계**
2. **데카르트 좌표** → **관절 각도** (역기구학)
3. **궤적 계획** → **모터 제어 신호**

**주요 이동 모드:**
- **PTP (Point-to-Point)**: 관절 공간에서의 이동
- **Linear**: 직선 경로 이동
- **Arc**: 호 궤적 이동

### 3. 시각 인식 파이프라인

```
카메라 입력 → 전처리 → 객체 검출 → 포즈 추정 → 좌표 변환 → 동작 계획
```

- **객체 검출**: YOLO/GroundingDINO 기반
- **깊이 정보**: RGB-D 카메라 활용
- **좌표 보정**: 카메라-로봇 캘리브레이션

### 4. AI 통합 시스템

**멀티모달 AI 워크플로우:**
1. **음성 입력** → 텍스트 변환 (STT)
2. **자연어 처리** → 작업 명령 파싱
3. **시각 정보** → 환경 인식
4. **동작 생성** → 로봇 제어 명령
5. **피드백** → 음성 출력 (TTS)

## 설치 및 설정

### 시스템 요구사항

- **운영체제**: Windows 10/11, Ubuntu 20.04+, macOS 10.15+
- **Python**: 3.8 이상
- **메모리**: 8GB RAM 권장
- **GPU**: CUDA 지원 GPU (비전 처리용)

### 의존성 설치

```bash
# Python 패키지 설치
pip install -r requirements.txt

# ROS2 설치 (Linux 환경)
sudo apt install ros-humble-desktop

# Dobot 드라이버 설치
# Windows: DobotStudio에서 드라이버 다운로드
# Linux/macOS: libusb 기반 드라이버 컴파일
```

### 하드웨어 연결

1. **Dobot 연결**
   ```bash
   # USB 연결 확인
   ls /dev/ttyUSB*  # Linux
   # 또는 Device Manager 확인 (Windows)
   ```

2. **카메라 설정**
   ```bash
   # RealSense 카메라 테스트
   realsense-viewer
   ```

3. **캘리브레이션**
   ```bash
   # 카메라-로봇 캘리브레이션 실행
   python scripts/calibration.py
   ```

## 사용법

### 기본 제어

```python
from dobot_api import DobotApi

# 로봇 연결
dobot = DobotApi()
dobot.connect()

# 홈 위치로 이동
dobot.home()

# 특정 좌표로 이동
dobot.move_to(x=200, y=0, z=50, r=0)

# 그리퍼 제어
dobot.gripper_open()
dobot.gripper_close()
```

### 비전 기반 작업

```python
from vision_system import VisionSystem
from robot_controller import RobotController

# 시스템 초기화
vision = VisionSystem()
robot = RobotController()

# 객체 검출 및 파지
objects = vision.detect_objects()
for obj in objects:
    position = vision.get_3d_position(obj)
    robot.pick_and_place(position)
```

### AI 음성 제어

```python
from ai_controller import AIController

# AI 시스템 시작
ai = AIController()
ai.start_voice_control()

# 음성 명령 예시:
# "빨간 블록을 집어서 상자에 넣어줘"
# "테이블 위의 모든 물체를 정리해줘"
```

## 프로젝트 구조

```
dobot_project/
├── src/                    # 소스 코드
│   ├── dobot_api/         # Dobot 제어 API
│   ├── vision_system/     # 컴퓨터 비전 모듈
│   ├── ai_controller/     # AI 통합 시스템
│   └── utils/             # 유틸리티 함수
├── config/                # 설정 파일
│   ├── robot_config.yaml
│   ├── camera_params.yaml
│   └── ai_models.yaml
├── scripts/               # 실행 스크립트
│   ├── calibration.py
│   ├── test_robot.py
│   └── demo.py
├── backupfile/           # 백업 및 데이터
│   ├── calibration_data/
│   ├── trained_models/
│   └── logs/
├── docs/                 # 문서
├── tests/                # 테스트 코드
└── requirements.txt      # 의존성 목록
```

## 주요 기능

### 1. 정밀 제어
- 반복 정밀도: ±0.2mm
- 작업 반경: 320mm
- 축 개수: 4축 (3축 + 회전축)

### 2. 다양한 작업 모드
- **픽 앤 플레이스**: 객체 이동 및 정렬
- **그리기/쓰기**: 펜 홀더를 이용한 드로잉
- **3D 프린팅**: 압출기 장착시 3D 출력
- **레이저 조각**: 레이저 모듈 장착시 조각 작업

### 3. 시각 인식
- 실시간 객체 검출
- 색상 기반 분류
- 깊이 정보 활용한 3D 위치 추정
- 다중 객체 동시 추적

### 4. AI 통합
- 자연어 명령 처리
- 작업 계획 자동 생성
- 상황 인식 및 적응
- 음성 피드백

## 안전 주의사항

⚠️ **중요 안전 수칙**

1. **작업 공간 확보**: 로봇 주변 320mm 반경 내 장애물 제거
2. **비상 정지**: 응급시 전원 차단 준비
3. **충돌 방지**: 작업 전 경로 확인
4. **정기 점검**: 관절 및 케이블 상태 확인
5. **캘리브레이션**: 주간 단위 좌표계 보정

## 문제 해결

### 연결 문제
```bash
# 포트 권한 설정 (Linux)
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### 정확도 문제
- 캘리브레이션 재실행
- 작업 공간 내 조명 확인
- 카메라 렌즈 청소

### 성능 최적화
- GPU 메모리 할당 조정
- 이미지 해상도 최적화
- 추론 모델 경량화

## 참고 자료

- [Dobot Magician 공식 문서](https://www.dobot.cc/dobot-magician/specification)
- [Dobot Communication Protocol](https://github.com/Dobot-Arm/DobotLink)
- [ROS2 제어 가이드](https://github.com/jkaniuka/magician_ros2)
- [Python API 예제](https://github.com/AlexGustafsson/dobot-python)

## 연락처

- 프로젝트 관리자: [GitHub Profile](https://github.com/johyunsu2mb)
- 기술 지원: [Gmail](johyunsu61@gmail.com)

---

*이 README는 Dobot 로봇팔을 활용한 지능형 자동화 시스템 개발을 위한 종합 가이드입니다.*
