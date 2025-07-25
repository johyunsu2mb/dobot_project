# 🚀 빠른 시작 가이드

## ⚡ 5분 만에 시작하기

### 1️⃣ 파일 준비 (1분)
모든 파일을 동일한 폴더에 저장하세요:
```
enhanced_dobot_system/
├── main.py
├── config.py
├── logger_setup.py
├── utils.py
├── robot_controller.py
├── yolo_detector.py
├── ui_components.py
├── requirements.txt
├── setup.py
└── test_system.py
```

### 2️⃣ 자동 설치 실행 (3분)
```bash
# Windows
python setup.py

# 또는 수동 설치
pip install numpy matplotlib opencv-python ultralytics pillow pyyaml
```

### 3️⃣ 즉시 실행 (1분)
```bash
python main.py
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

## 🧪 시스템 테스트

전체 시스템이 정상 동작하는지 확인:
```bash
python test_system.py
```

예상 출력:
```
🧪 Enhanced Dobot Robot System 테스트 시작
✅ config.py 임포트 성공
✅ logger_setup.py 동작 확인
✅ utils.py 함수들 동작 확인
✅ robot_controller.py 시뮬레이션 모드 확인
✅ yolo_detector.py YOLO 모델 로드 확인
✅ ui_components.py 컴포넌트 생성 확인
🎉 모든 테스트가 성공적으로 완료되었습니다!
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

### 💡 팁
- 실제 로봇 없이도 모든 기능 테스트 가능
- 시뮬레이션 모드에서 로직 검증 후 실제 로봇 연결 권장
- 첫 실행 시 YOLO 모델 다운로드로 시간이 걸릴 수 있음

---

**🎉 설치 완료! 이제 Enhanced Dobot Robot System을 사용할 준비가 되었습니다!**