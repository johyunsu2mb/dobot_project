# 🔧 Enhanced Dobot Robot System - 트러블슈팅 가이드

## 🚨 주요 문제별 해결책

### 🤖 Dobot API 관련 문제

#### ❌ "Dobot API를 찾을 수 없습니다" 오류

**증상**: 
```
⚠️ Dobot API가 사용 불가능합니다. 시뮬레이션 모드로 실행됩니다.
```

**해결 단계**:

1. **진단 도구 실행**:
   ```bash
   python diagnose_dobot.py
   ```

2. **권장 API 설치**:
   ```bash
   # 가장 쉬운 방법
   pip install pydobot
   
   # 또는 시리얼 통신 기반
   pip install pyserial
   ```

3. **공식 API 설치** (고급 사용자):
   - [Dobot 공식 사이트](https://www.dobot.cc/downloadcenter.html) 방문
   - Dobot Studio 다운로드 및 설치
   - Python API 라이브러리 추가 설치

4. **설치 확인**:
   ```bash
   python -c "import pydobot; print('설치 성공')"
   ```

#### ❌ API는 있지만 로봇 연결 실패

**증상**:
```
[YELLOW] API Available (Not Connected)
```

**해결 체크리스트**:

- [ ] **전원 확인**: Dobot 로봇 전원 어댑터 연결
- [ ] **USB 연결**: USB 케이블이 컴퓨터와 로봇에 제대로 연결
- [ ] **드라이버 설치**: 장치 관리자에서 Dobot 인식 확인
- [ ] **포트 확인**: 
  ```bash
  python -c "import serial.tools.list_ports; [print(p) for p in serial.tools.list_ports.comports()]"
  ```
- [ ] **방화벽**: Windows 방화벽에서 Python/Dobot Studio 허용
- [ ] **다른 프로그램**: Dobot Studio나 다른 프로그램이 로봇을 사용 중인지 확인

#### ❌ "Movement timeout" 오류

**증상**:
```
[ERROR] 의자 픽업 실패: Movement timeout: [292, -188, -65.0, -59]
```

**원인별 해결**:

1. **실제 로봇 연결 문제**:
   - 로봇 전원 상태 확인
   - USB 케이블 재연결
   - 로봇 재시작 (전원 껐다 켜기)

2. **로봇 위치 문제**:
   - 로봇이 홈 위치에 있는지 확인
   - 수동으로 로봇 위치 조정
   - 작업 공간 제한 확인

3. **네트워크 설정 문제**:
   ```python
   # config.py에서 IP 주소 확인
   ip_address = "192.168.1.6"  # 실제 로봇 IP로 변경
   ```

### 📦 패키지 설치 문제

#### ❌ "ModuleNotFoundError" 오류

**해결 순서**:

1. **기본 패키지 재설치**:
   ```bash
   pip install --upgrade pip
   pip install numpy matplotlib opencv-python ultralytics pillow pyyaml
   ```

2. **가상환경 사용** (권장):
   ```bash
   python -m venv robot_env
   # Windows
   robot_env\Scripts\activate
   # Linux/macOS  
   source robot_env/bin/activate
   
   pip install -r requirements.txt
   ```

3. **시스템별 패키지 설치**:
   
   **Windows**:
   ```bash
   # Microsoft Visual C++ Build Tools 필요할 수 있음
   # https://visualstudio.microsoft.com/visual-cpp-build-tools/
   ```
   
   **Ubuntu/Debian**:
   ```bash
   sudo apt update
   sudo apt install python3-dev python3-pip
   sudo apt install libopencv-dev python3-opencv
   ```
   
   **macOS**:
   ```bash
   brew install python3
   brew install opencv
   ```

#### ❌ YOLO 모델 다운로드 실패

**증상**:
```
❌ YOLO model loading failed: HTTP Error 403
```

**해결 방법**:

1. **인터넷 연결 확인**
2. **방화벽/프록시 설정 확인**
3. **수동 모델 다운로드**:
   ```bash
   # 작은 모델부터 시도
   python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
   ```
4. **오프라인 모델 사용**:
   - 미리 다운로드된 모델 파일 사용
   - 네트워크 환경에서 모델 다운로드 후 복사

### 📷 카메라 관련 문제

#### ❌ "No camera found" 오류

**해결 체크리스트**:

- [ ] **카메라 연결**: 웹캠이 제대로 연결되어 있는지 확인
- [ ] **드라이버 설치**: 카메라 드라이버 설치 상태 확인
- [ ] **다른 프로그램**: 다른 앱이 카메라를 사용 중인지 확인
- [ ] **권한 설정**: 카메라 접근 권한 허용

**테스트 방법**:
```bash
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed'); cap.release()"
```

**다른 카메라 인덱스 시도**:
```python
# main.py에서 카메라 인덱스 변경
self.cap = cv2.VideoCapture(1)  # 0 대신 1, 2, 3 시도
```

### 🖼️ GUI 관련 문제

#### ❌ 한글 폰트가 깨져 보임

**해결 방법**:

1. **나눔고딕 폰트 설치**:
   - [나눔고딕 다운로드](https://hangeul.naver.com/2017/nanum)
   - `fonts/` 폴더에 `NanumGothic.ttf` 파일 복사

2. **시스템 폰트 사용**:
   ```python
   # utils.py에서 폰트 설정 확인
   ```

#### ❌ GUI가 너무 작거나 큼

**해결 방법**:

1. **DPI 설정 조정** (Windows):
   - 우클릭 → 디스플레이 설정 → 배율 조정

2. **GUI 크기 수동 조정**:
   ```python
   # main.py에서 창 크기 변경
   self.root.geometry("1800x1200")  # 크기 조정
   ```

### 🔗 네트워크 관련 문제

#### ❌ 로봇과 네트워크 연결 실패

**확인 사항**:

1. **IP 주소 확인**:
   ```bash
   ping 192.168.1.6  # 로봇 IP 핑 테스트
   ```

2. **포트 확인**:
   ```bash
   telnet 192.168.1.6 29999  # 대시보드 포트
   telnet 192.168.1.6 30003  # 이동 포트
   ```

3. **방화벽 설정**:
   - Windows Defender 방화벽에서 Python 허용
   - 포트 29999, 30003, 30004 허용

### 🪟 운영체제별 문제

#### Windows 특화 문제

1. **PowerShell 실행 정책**:
   ```powershell
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
   ```

2. **경로에 공백/한글 있음**:
   - 영문 경로로 이동하여 실행

3. **Visual C++ 런타임 오류**:
   - Microsoft Visual C++ Redistributable 설치

#### Linux 특화 문제

1. **권한 문제**:
   ```bash
   sudo usermod -a -G dialout $USER  # 시리얼 포트 접근
   sudo usermod -a -G video $USER    # 카메라 접근
   # 로그아웃 후 재로그인 필요
   ```

2. **X11 디스플레이 문제**:
   ```bash
   export DISPLAY=:0
   ```

#### macOS 특화 문제

1. **보안 설정**:
   - 시스템 환경설정 → 보안 및 개인정보보호
   - 카메라 접근 허용

2. **Homebrew 패키지**:
   ```bash
   brew install python-tk  # tkinter 설치
   ```

## 🛠️ 고급 진단 도구

### 시스템 전체 진단
```bash
python diagnose_dobot.py     # Dobot API 전체 진단
python test_system.py        # 시스템 통합 테스트
python setup.py              # 재설치 및 환경 설정
```

### 개별 컴포넌트 테스트
```bash
# 각 모듈별 개별 테스트
python -c "import config; print('Config OK')"
python -c "import robot_controller; print('Robot Controller OK')"
python -c "import yolo_detector; print('YOLO Detector OK')"
```

### 로그 분석
```bash
# 실시간 로그 모니터링
tail -f logs/robot_system.log     # Linux/macOS
Get-Content logs\robot_system.log -Wait  # PowerShell

# 에러 로그만 필터링
grep "ERROR" logs/robot_system.log
```

## 📞 추가 도움

### 커뮤니티 지원
- GitHub Issues 등록
- 로그 파일과 함께 문제 상황 설명

### 개발자 모드
```bash
# 디버그 모드로 실행
export ROBOT_DEBUG=1  # Linux/macOS
set ROBOT_DEBUG=1     # Windows
python main.py
```

### 완전 초기화
```bash
# 모든 설정 및 로그 삭제
rm -rf logs/ models/ __pycache__/  # Linux/macOS
rmdir /s logs models __pycache__   # Windows
python setup.py  # 재설치
```

---

**💡 중요**: 문제가 해결되지 않아도 시뮬레이션 모드로 모든 기능을 테스트할 수 있습니다!
