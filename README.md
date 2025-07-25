# Enhanced Dobot Robot & YOLO Object Detection System

## 🤖 프로젝트 개요

이 프로젝트는 Dobot 로봇팔과 YOLOv8 객체 인식을 결합한 향상된 가구 픽업 시스템입니다. 실제 로봇이 연결되지 않은 경우 시뮬레이션 모드로 동작합니다.

### ✨ 주요 기능

- **향상된 로봇 제어**: 실제 Dobot 로봇팔 또는 시뮬레이션 모드 지원
- **YOLOv8 객체 인식**: 실시간 카메라 피드를 통한 객체 탐지
- **가구 픽업 시퀀스**: 소파, 의자, 책상, 침대 자동 픽업 작업
- **3D 시각화**: 로봇팔과 작업 공간의 실시간 3D 시각화
- **안전한 로깅**: Windows 환경에서 한글/이모지 안전 처리
- **버그 수정**: 타임아웃, NoneType 에러 등 주요 버그 해결

## 📁 파일 구조

```
enhanced_dobot_system/
├── main.py                 # 메인 애플리케이션
├── config.py              # 설정 및 상수
├── logger_setup.py        # 로깅 시스템
├── utils.py               # 유틸리티 함수들
├── robot_controller.py    # 로봇 제어 클래스
├── yolo_detector.py       # YOLO 객체 인식
├── ui_components.py       # UI 컴포넌트들
├── requirements.txt       # 필요한 패키지 목록
├── setup.py              # 설치 스크립트
├── README.md             # 이 파일
└── logs/                 # 로그 파일들 (자동 생성)
    ├── robot_system.log
    └── order_log.txt
```

## 🚀 설치 및 실행

### 1. 필수 요구사항

- Python 3.8 이상
- Windows 10/11 (권장)
- 웹캠 (카메라 기능 사용 시)
- Dobot 로봇 (선택사항, 없으면 시뮬레이션 모드)

### 2. 패키지 설치

```bash
# 기본 패키지 설치
pip install -r requirements.txt

# 또는 개별 설치
pip install numpy matplotlib opencv-python ultralytics pillow pyyaml
```

### 3. 선택적 설치

```bash
# Dobot API (실제 로봇 사용 시)
# 별도로 Dobot 공식 API 설치 필요

# 폰트 개선 (권장)
# fonts 폴더에 NanumGothic.ttf 파일 추가
```

### 4. 실행

```bash
python main.py
```

## 🎮 사용법

### 기본 조작

1. **가구 픽업**: 좌측 패널의 가구 버튼 클릭
2. **수동 이동**: X, Y, Z, R 좌표 입력 후 이동 버튼 클릭
3. **그리퍼 제어**: ON/OFF 버튼으로 그리퍼 조작
4. **카메라 활성화**: YOLO 객체 인식과 함께 카메라 시작

### 향상된 픽업 로직

1. 안전 위치로 이동 (목표 위치 + 50mm 상승)
2. 그리퍼 열기
3. 목표 위치로 하강
4. 그리퍼 닫기 (물체 집기)
5. 안전 위치로 상승
6. 베이스 위치로 이동
7. **최종 위치로 이동**: [350, 0, 물건Z좌표, 회전값]

### 시스템 제어

- **재연결**: 로봇 연결 상태 재설정
- **전체 리셋**: 시스템 완전 초기화
- **로그 확인**: 우측 패널에서 실시간 로그 모니터링

## 🔧 설정

### config.py에서 수정 가능한 설정들

```python
# 로봇 연결 설정
ip_address = "192.168.1.6"
dashboard_port = 29999
move_port = 30003

# 움직임 설정
movement_timeout = 30.0
position_tolerance = 1.0
safety_height_offset = 50.0

# 작업 공간 제한
x_min, x_max = -400, 400
y_min, y_max = -400, 400
z_min, z_max = -200, 200
```

## 🚨 문제 해결

### 자주 발생하는 문제들

1. **"Movement timeout" 오류**
   - 로봇 연결 상태 확인
   - 목표 위치가 작업 공간 내에 있는지 확인
   - 로봇 전원 및 케이블 상태 점검

2. **"NoneType object" 오류**
   - 로봇 피드백 연결 상태 확인
   - 시스템 재시작 시도

3. **카메라 인식 안됨**
   - 웹캠 연결 상태 확인
   - 다른 프로그램에서 카메라 사용 중인지 확인

4. **YOLO 로딩 실패**
   - 인터넷 연결 확인 (모델 다운로드 필요)
   - ultralytics 패키지 재설치

### 로그 확인

```bash
# 시스템 로그
tail -f logs/robot_system.log

# 주문 로그
tail -f logs/order_log.txt
```

## 🛠️ 개발자 정보

### 디버깅 모드

환경변수 설정으로 디버깅 활성화:
```bash
export ROBOT_DEBUG=1
python main.py
```

### 코드 구조

- **모듈화**: 각 기능별로 별도 파일 분리
- **에러 처리**: 포괄적인 예외 처리 및 로깅
- **타입 힌팅**: 함수 매개변수 및 반환값 타입 명시
- **문서화**: 모든 클래스와 함수에 docstring 추가

## 📝 업데이트 내역

### v2.0.0 (2024-07-25)
- 주요 버그 수정 (타임아웃, NoneType 에러)
- 코드 모듈화 및 분할
- 안전한 로깅 시스템 구현
- 향상된 픽업 로직 적용

### v1.0.0 (2024-07-24)
- 초기 통합 버전 릴리스

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🤝 기여

버그 리포트나 기능 개선 제안은 이슈 트래커를 통해 제출해 주세요.

---

**참고**: 실제 Dobot 로봇 없이도 시뮬레이션 모드로 모든 기능을 테스트할 수 있습니다.
