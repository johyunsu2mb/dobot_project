# 🚀 빠른 시작 가이드

## ⚡ 5분 만에 시작하기

### 1️⃣ 파일 준비 (1분)
모든 파일을 `python_project_file` 폴더에 저장하세요:
```
python_project_file/
├── main.py                      # 메인 애플리케이션  
├── config.py                   # 설정 파일
├── dobot_api_handler.py        # **NEW!** API 핸들러 (중요!)
├── robot_controller.py         # 로봇 제어
├── yolo_detector.py           # YOLO 검출
├── ui_components.py           # UI 컴포넌트
├── logger_setup.py            # 로깅 설정
├── utils.py                   # 유틸리티
├── diagnose_dobot.py          # **NEW!** 진단 도구
├── integration_test.py        # **NEW!** 통합 테스트
├── run.bat / run.sh           # **NEW!** 실행 스크립트
├── requirements.txt           # 패키지 목록
└── 기타 문서들...
```

### 2️⃣ 자동 설치 실행 (3분)
```bash
# 자동 설치 (권장)
python setup.py

# 또는 수동 설치
pip install numpy matplotlib opencv-python ultralytics pillow pyyaml pydobot
```

### 3️⃣ 즉시 실행 (1분)
```bash
# 방법 1: 직접 실행
python main.py

# 방법 2: 실행 스크립트 사용 (권장)
# Windows
run.bat
# 또는 Linux/macOS  
chmod +x run.sh && ./run.sh

# 방법 3: 진단 후 실행
python diagnose_dobot.py  # 문제가 있는 경우
python integration_test.py  # 전체 테스트
```

## 🎯 첫 번째 테스트

### ✅ 시뮬레이션 모드에서 가구 픽업 테스트
1. 프로그램 실행 후 좌측 패널에서 **"[CHAIR] 의자"** 버튼 클릭
2. 로그에서 다음과 같은 메시지 확인:
   ```
   [TARGET] 의자 향상된 픽업 시퀀스 시작
   [PIN] 목표 위치: [292, -188, -115, -59]
   1. 안전 위치로 이동: [292, -188, -65.0, -59]
   2. 그리퍼 열기
   3. 의자 위치로 하강: [292, -188, -115, -59]
   4. 의자 집기 - 그리퍼 활성화
   5. 물체를 들고 안전 위치로 상승
   6. 베이스 위치로 이동: [300, -30, 5, 0]
   7. 최종 배치 위치로 이동: [350, 0, -115, -59]
   [SUCCESS] 의자 픽업 및 배치 시퀀스 완료!
   ```

### ✅ 수동 좌표 이동 테스트
1. 좌측 패널의 **"Manual Coordinate Control"** 섹션에서:
   - X: `100`
   - Y: `100` 
   - Z: `-100`
   - R: `0`
2. **"[PIN] Move to Coordinates"** 버튼 클릭
3. 성공 메시지 확인

### ✅ 카메라 & YOLO 테스트 (웹캠 있는 경우)
1. **"[CAMERA] Start Camera & YOLO"** 버튼 클릭
2. 카메라 피드가 우측 상단에 표시되는지 확인
3. 객체가 인식되면 우측 중앙에 결과 표시

## 🔧 문제 해결

### ❌ "Dobot API를 찾을 수 없습니다" 오류 (NEW!)
```bash
# 1단계: 진단 도구 실행
python diagnose_dobot.py

# 2단계: 권장 API 설치
pip install pydobot

# 3단계: 대체 방법
pip install pyserial
```

**💡 참고**: API 없이도 시뮬레이션 모드로 모든 기능 테스트 가능!

### ❌ "ModuleNotFoundError" 오류
```bash
# 패키지 재설치
pip install --upgrade numpy matplotlib opencv-python ultralytics pillow pyyaml
```

### ❌ "Movement timeout" 오류 (정상)
시뮬레이션 모드에서는 정상적인 동작입니다. 실제 로봇 연결 시에만 해결됩니다.

### ❌ 카메라 인식 안됨
```bash
# 카메라 테스트
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed')"
```

### ❌ YOLO 모델 다운로드 실패
인터넷 연결 확인 후 다시 실행하세요. 첫 실행 시 자동으로 모델을 다운로드합니다.

## 🧪 시스템 진단 및 테스트

### 🔍 문제 진단 (Dobot API 문제 해결)
```bash
python diagnose_dobot.py
```

예상 출력:
```
🤖======================================================🤖
     Enhanced Dobot Robot System - API 진단 도구
🤖======================================================🤖

🔍 Dobot API 설정 진단 중...
✅ Python 버전: 3.9.0
⚠️ Dobot API 가용성: 사용불가
💡 권장사항:
  1. 가장 쉬운 방법: pip install pydobot
  2. 공식 Dobot Studio와 함께 설치
  ...
✨ 시뮬레이션 모드는 API 없이도 정상 작동합니다!
```

### ⚡ 연결 안정성 테스트 (NEW!)
**통신 끊김 문제가 있는 경우:**
```bash
python test_connection_stability.py
```

**테스트 옵션:**
- **빠른 테스트 (30초)**: 기본 연결 확인
- **표준 테스트 (5분)**: 일반적인 안정성 검사  
- **스트레스 테스트 (10분)**: 장시간 연결 안정성

**예상 결과:**
```
📊 연결 안정성 테스트 결과
✅ 성공한 이동: 45회 (90.0%)
🔄 연결 복구: 2회
🎉 우수: 연결이 매우 안정적입니다!
```
```bash
python integration_test.py
```

예상 출력:
```
🧪======================================================================🧪
    Enhanced Dobot Robot System - 통합 테스트
🧪======================================================================🧪

✅ PASS       Python 버전
✅ PASS       파일 main.py
✅ PASS       Dobot API 핸들러 로드
⚠️ WARNING    YOLO 모델 로드 (모델 없음, 정상)
✅ PASS       가구 픽업 워크플로우

📊 테스트 결과 요약
전체 테스트: 25개
✅ 성공: 22개
❌ 실패: 0개
⚠️ 경고: 3개

🎉 모든 테스트가 성공했습니다! 시스템이 완벽히 준비되었습니다.
```

## 🎮 주요 기능 사용법

### 🤖 로봇 제어
| 기능 | 위치 | 설명 |
|------|------|------|
| 가구 픽업 | 좌측 패널 가구 버튼 | 자동 픽업 시퀀스 실행 |
| 수동 이동 | 좌측 패널 좌표 입력 | 직접 좌표 지정 이동 |
| 그리퍼 제어 | 좌측 패널 ON/OFF | 그리퍼 열기/닫기 |

### 📷 비전 시스템  
| 기능 | 위치 | 설명 |
|------|------|------|
| 카메라 시작 | 좌측 패널 카메라 버튼 | 실시간 피드 활성화 |
| YOLO 인식 | 자동 실행 | 객체 자동 탐지 |
| 커스텀 라벨 | 좌측 패널 라벨 버튼 | 사용자 정의 라벨 로드 |

### 🔧 시스템 관리
| 기능 | 위치 | 설명 |
|------|------|------|
| 로봇 재연결 | 좌측 패널 재연결 버튼 | 연결 상태 리셋 |
| 전체 리셋 | 좌측 패널 리셋 버튼 | 시스템 완전 초기화 |
| 로그 확인 | 우측 패널 하단 | 실시간 로그 모니터링 |

### 🔧 새로운 진단 도구 (NEW!)
| 도구 | 명령어 | 용도 |
|------|--------|------|
| API 진단 | `python diagnose_dobot.py` | Dobot API 설치/연결 문제 해결 |
| 통합 테스트 | `python integration_test.py` | 전체 시스템 동작 확인 |
| 연결 안정성 테스트 | `python test_connection_stability.py` | **NEW!** 통신 끊김 문제 진단 |
| 실행 스크립트 | `run.bat` / `./run.sh` | 안전한 시스템 시작 |

## 🎯 향상된 픽업 로직 (NEW!)

기존 문제점을 해결한 새로운 로직:
```
1. 안전 위치 이동 → 2. 그리퍼 열기 → 3. 물체 위치 하강 
→ 4. 그리퍼 닫기 → 5. 안전 위치 상승 → 6. 베이스 이동 
→ 7. 최종 위치 [350, 0, 물건Z좌표, 회전값] 이동 완료!
```

## 📞 도움말

### 📖 상세 문서
- `README.md`: 전체 프로젝트 가이드
- `PROJECT_STRUCTURE.md`: 코드 구조 상세 설명

### 🐛 버그 리포트
로그 파일 확인:
- `logs/robot_system.log`: 전체 시스템 로그
- `logs/order_log.txt`: 주문 처리 로그

### 💡 팁 및 해결책

### 🤖 Dobot API 관련
- **API 없음**: 정상입니다! 시뮬레이션 모드로 모든 기능 테스트 가능
- **연결 실패**: `python diagnose_dobot.py` 실행하여 문제 진단
- **실제 로봇 연결**: `pip install pydobot` 후 USB/전원 확인

### 📷 카메라 관련  
- **카메라 인식 안됨**: 다른 프로그램이 사용 중인지 확인
- **YOLO 모델 다운로드 실패**: 인터넷 연결 확인 후 재시도
- **객체 인식 정확도 낮음**: 조명 개선 및 카메라 각도 조정

### 🖥️ 시스템 관련
- **폰트 깨짐**: `fonts/` 폴더에 `NanumGothic.ttf` 파일 추가
- **GUI 크기 문제**: 디스플레이 배율 설정 확인
- **느린 실행**: 가상환경 사용 및 백그라운드 프로그램 종료

### 🔧 문제 해결 순서
1. **진단 도구**: `python diagnose_dobot.py`
2. **통합 테스트**: `python integration_test.py`  
3. **연결 안정성**: `python test_connection_stability.py` (통신 문제 시)
4. **로그 확인**: `logs/robot_system.log` 파일 검토
5. **재설치**: `python setup.py` 재실행
6. **문서 참조**: `TROUBLESHOOTING.md` 확인

---

## 🎉 설치 완료!

**✨ Enhanced Dobot Robot System 준비 완료! ✨**

### 🚀 다음 단계:
1. **첫 실행**: `python main.py` 또는 `run.bat` 실행  
2. **시뮬레이션 테스트**: 가구 버튼 클릭하여 픽업 시퀀스 확인
3. **실제 로봇 연결**: `pip install pydobot` 후 하드웨어 연결
4. **카메라 테스트**: 웹캠 연결 후 YOLO 객체 인식 시도

### 📞 도움이 필요하면:
- **📖 상세 가이드**: `README.md`
- **🔧 문제 해결**: `TROUBLESHOOTING.md`  
- **✅ 설치 체크**: `INSTALLATION_CHECKLIST.md`
- **🏗️ 구조 이해**: `PROJECT_STRUCTURE.md`

**💡 기억하세요**: Dobot API가 없어도 완벽한 시뮬레이션으로 모든 기능을 체험할 수 있습니다!
