#!/bin/bash

# Enhanced Dobot Robot System 실행 스크립트 (Linux/macOS)

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 헤더 출력
echo -e "${CYAN}"
echo "🤖======================================================🤖"
echo "     Enhanced Dobot Robot & YOLO Object Detection System"
echo "🤖======================================================🤖"
echo -e "${NC}"

# Python 설치 확인
echo -e "${BLUE}🐍 Python 확인 중...${NC}"
if ! command -v python3 &> /dev/null; then
    if ! command -v python &> /dev/null; then
        echo -e "${RED}❌ Python이 설치되지 않았습니다.${NC}"
        echo -e "${YELLOW}💡 Python 3.8 이상을 설치하세요.${NC}"
        echo "   Ubuntu/Debian: sudo apt install python3 python3-pip"
        echo "   CentOS/RHEL: sudo yum install python3 python3-pip"
        echo "   macOS: brew install python3"
        exit 1
    else
        PYTHON_CMD="python"
    fi
else
    PYTHON_CMD="python3"
fi

echo -e "${GREEN}✅ Python 확인됨${NC}"
$PYTHON_CMD --version

# 필수 파일 확인
echo -e "${BLUE}📁 필수 파일 확인 중...${NC}"
required_files=("main.py" "dobot_api_handler.py" "config.py" "robot_controller.py")

for file in "${required_files[@]}"; do
    if [[ ! -f "$file" ]]; then
        echo -e "${RED}❌ $file 파일을 찾을 수 없습니다.${NC}"
        echo -e "${YELLOW}💡 모든 파일이 같은 폴더에 있는지 확인하세요.${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✅ 필수 파일 확인됨${NC}"

# 기본 패키지 확인
echo -e "${BLUE}📦 필수 패키지 확인 중...${NC}"
if ! $PYTHON_CMD -c "import numpy, matplotlib, cv2, PIL" 2>/dev/null; then
    echo -e "${YELLOW}⚠️ 일부 필수 패키지가 없습니다.${NC}"
    echo -n "💡 자동 설치를 시도하시겠습니까? (y/N): "
    read -r install_choice
    
    if [[ $install_choice =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}🔄 패키지 설치 중...${NC}"
        
        # pip 명령어 확인
        if command -v pip3 &> /dev/null; then
            PIP_CMD="pip3"
        elif command -v pip &> /dev/null; then
            PIP_CMD="pip"
        else
            echo -e "${RED}❌ pip이 설치되지 않았습니다.${NC}"
            echo "설치: sudo apt install python3-pip (Ubuntu/Debian)"
            exit 1
        fi
        
        # 패키지 설치
        $PIP_CMD install numpy matplotlib opencv-python pillow ultralytics pyyaml
        
        if [[ $? -eq 0 ]]; then
            echo -e "${GREEN}✅ 패키지 설치 완료${NC}"
        else
            echo -e "${RED}❌ 패키지 설치 실패${NC}"
            echo -e "${YELLOW}💡 수동으로 설치하세요: $PIP_CMD install -r requirements.txt${NC}"
            exit 1
        fi
    else
        echo -e "${YELLOW}⚠️ 패키지 없이 실행 시도 중...${NC}"
    fi
else
    echo -e "${GREEN}✅ 필수 패키지 확인됨${NC}"
fi

# Dobot API 확인
echo -e "${BLUE}🤖 Dobot API 확인 중...${NC}"
if $PYTHON_CMD -c "from dobot_api_handler import DOBOT_API_AVAILABLE; print('API Available:', DOBOT_API_AVAILABLE)" 2>/dev/null; then
    echo -e "${GREEN}✅ Dobot API 핸들러 로드 성공${NC}"
else
    echo -e "${YELLOW}⚠️ Dobot API 핸들러 로드 실패${NC}"
    echo -e "${YELLOW}💡 diagnose_dobot.py를 실행하여 문제를 확인하세요.${NC}"
fi

# 로그 디렉터리 생성
if [[ ! -d "logs" ]]; then
    mkdir -p logs
fi
echo -e "${GREEN}✅ 로그 디렉터리 준비됨${NC}"

# 권한 확인 (Linux/macOS)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # 카메라 권한 확인 (Linux)
    if [[ -e /dev/video0 ]]; then
        if [[ ! -r /dev/video0 ]]; then
            echo -e "${YELLOW}⚠️ 카메라 접근 권한이 없을 수 있습니다.${NC}"
            echo "💡 사용자를 video 그룹에 추가하세요: sudo usermod -a -G video $USER"
        fi
    fi
fi

echo ""
echo -e "${CYAN}🚀 시스템 시작 중...${NC}"
echo "======================================================"
echo -e "${YELLOW}💡 팁:${NC}"
echo "   - Dobot API가 없어도 시뮬레이션 모드로 실행됩니다"
echo "   - 실제 로봇 연결을 위해서는 'pip3 install pydobot' 실행"
echo "   - 문제 발생 시 diagnose_dobot.py 실행하여 진단"
echo "======================================================"
echo ""

# 메인 프로그램 실행
$PYTHON_CMD main.py
exit_code=$?

# 종료 처리
echo ""
if [[ $exit_code -eq 0 ]]; then
    echo -e "${GREEN}✅ 프로그램이 정상적으로 종료되었습니다.${NC}"
else
    echo -e "${RED}❌ 프로그램이 오류와 함께 종료되었습니다.${NC}"
    echo -e "${YELLOW}💡 logs/robot_system.log 파일을 확인하세요.${NC}"
fi

echo ""
echo -e "${BLUE}🔧 문제 해결 도구:${NC}"
echo "   - $PYTHON_CMD diagnose_dobot.py  (Dobot API 진단)"
echo "   - $PYTHON_CMD test_system.py     (시스템 테스트)"
echo "   - $PYTHON_CMD setup.py           (재설치)"
echo ""

# 스크립트가 직접 실행된 경우에만 대기
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo -n "Press any key to continue..."
    read -n 1 -s
    echo ""
fi
