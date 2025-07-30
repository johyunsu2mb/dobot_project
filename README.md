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

### 🔧 버그 수정 사항 (v2.0.1 - 통신 안정성 강화)

1. **통신 끊김 문제 해결**
   - 자동 연결 상태 모니터링 (5초 간격)
   - 연결 끊김 시 자동 재연결 기능
   - 스마트 에러 복구 시스템

2. **타임아웃 오류 개선**
   - 타임아웃 시간 증가 (30초 → 45초)
   - 재시도 로직 강화 (최대 3회)
   - 단계별 진행률 표시

3. **픽업 시퀀스 안정성 향상**
   - 각 단계별 독립적 오류 처리
   - 실패 시 안전 위치로 자동 복귀
   - 부분 성공 상태 추적

4. **그리퍼 제어 개선**
   - 그리퍼 응답 시간 증가 (1.0초 → 1.5초)
   - 재시도 메커니즘 추가
   - 안전한 상태 전환

## 📁 파일 구조 (최신 업데이트)

```
python_project_file/
├── 📄 main.py                      # 메인 애플리케이션
├── ⚙️ config.py                   # 시스템 설정 및 상수
├── 📝 logger_setup.py             # 로깅 시스템 설정
├── 🔧 utils.py                    # 유틸리티 함수 및 에러 클래스
├── 🤖 robot_controller.py         # 로봇 제어 클래스 (버그 수정됨)
├── 🛡️ dobot_api_handler.py        # **NEW!** Dobot API 핸들러
├── 👁️ yolo_detector.py            # YOLO 객체 인식 시스템
├── 🖼️ ui_components.py            # GUI 컴포넌트들
├── 📦 requirements.txt            # 필요한 패키지 목록
├── 🛠️ setup.py                    # 자동 설치 스크립트
├── 🧪 test_system.py              # 시스템 테스트 스크립트
├── 🔍 diagnose_dobot.py           # **NEW!** Dobot API 진단 도구
├── 🧪 integration_test.py         # **NEW!** 통합 테스트
├── ⚡ test_connection_stability.py # **NEW!** 연결 안정성 테스트
├── 📋 run.bat                     # **NEW!** Windows 실행 스크립트
├── 📋 run.sh                      # **NEW!** Linux/macOS 실행 스크립트
├── 📖 README.md                   # 프로젝트 가이드 (이 파일)
├── 📋 PROJECT_STRUCTURE.md        # 프로젝트 구조 상세 설명
├── 🚀 QUICK_START.md              # 빠른 시작 가이드
├── 🔧 TROUBLESHOOTING.md          # **NEW!** 트러블슈팅 가이드
├── ✅ INSTALLATION_CHECKLIST.md   # **NEW!** 설치 체크리스트
└── 📂 자동 생성 폴더들/
    ├── logs/                      # 로그 파일들
    ├── fonts/                     # 폰트 파일들 (선택사항)
    ├── models/                    # YOLO 모델 캐시
    └── data/                      # 데이터 파일들
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

### 🤖 Dobot API 인식 문제 (NEW!)

**문제**: "Dobot API를 찾을 수 없습니다" 메시지가 나타남

**해결방법**:

1. **진단 도구 실행**:
   ```bash
   python diagnose_dobot.py
   ```

2. **PyDobot 설치 (권장)**:
   ```bash
   pip install pydobot
   ```

3. **공식 Dobot API 설치**:
   - [Dobot 공식 사이트](https://www.dobot.cc/downloadcenter.html)에서 Dobot Studio 다운로드
   - Python API 라이브러리 추가 설치

4. **대체 라이브러리**:
   ```bash
   pip install DobotDllType
   # 또는
   pip install pyserial
   ```

5. **하드웨어 확인**:
   - USB 케이블로 Dobot 연결
   - 전원 어댑터 연결 확인
   - 장치 관리자에서 드라이버 설치 확인

**💡 중요**: API가 없어도 시뮬레이션 모드로 모든 기능을 테스트할 수 있습니다!

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

### v2.1.0 (2025-07-27)
-Yolo v8 연동
-자동모드 추가(미완성)

### v2.0.1 (2025-07-25)
- 그립퍼 기능 개선

### v2.0.0 (2025-07-25)
- 주요 버그 수정 (타임아웃, NoneType 에러)
- 코드 모듈화 및 분할
- 안전한 로깅 시스템 구현
- 향상된 픽업 로직 적용

### v1.0.0 (2025-07-24)
- 초기 통합 버전 릴리스

## 🤝 기여

버그 리포트나 기능 개선 제안 : johyunsu61@gmail.com

---

**참고**: 실제 Dobot 로봇 없이도 시뮬레이션 모드로 모든 기능을 테스트할 수 있습니다.
