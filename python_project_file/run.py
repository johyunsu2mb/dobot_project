#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot 로봇팔 제어 시스템 실행 스크립트
간단한 명령어로 애플리케이션을 시작할 수 있습니다.

사용법:
    python run.py              # 기본 실행
    python run.py --debug      # 디버그 모드
    python run.py --simulation # 시뮬레이션 모드 강제
    python run.py --real       # 실제 로봇 모드 시도
"""

import sys
import os
import argparse
import logging

# 프로젝트 루트 디렉토리를 Python 경로에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 설정 및 로깅 모듈 import
from config import AppConfig
from logger_setup import initialize_logging


def check_dependencies():
    """필수 의존성 패키지 확인"""
    required_packages = {
        'tkinter': 'tkinter (GUI)',
        'numpy': 'numpy (수치 계산)',
        'matplotlib': 'matplotlib (그래프)',
    }
    
    missing_packages = []
    
    for package, description in required_packages.items():
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(f"  - {package} ({description})")
    
    if missing_packages:
        print("❌ 다음 필수 패키지가 설치되지 않았습니다:")
        for package in missing_packages:
            print(package)
        print("\n설치 방법:")
        print("  pip install -r requirements.txt")
        return False
    
    return True


def check_optional_dependencies():
    """선택적 의존성 패키지 확인 및 알림"""
    optional_packages = {
        'pydobot': '실제 Dobot 로봇 제어',
        'pyserial': '시리얼 통신',
        'cv2': '카메라/이미지 처리 (opencv-python)',
        'PIL': '이미지 처리 (Pillow)'
    }
    
    missing_optional = []
    
    for package, description in optional_packages.items():
        try:
            __import__(package)
        except ImportError:
            missing_optional.append(f"  - {package} ({description})")
    
    if missing_optional:
        print("ℹ️  다음 선택적 패키지가 없습니다 (기본 기능은 정상 동작):")
        for package in missing_optional:
            print(package)
        print()


def setup_environment(args):
    """환경 설정"""
    # 디버그 모드 설정
    if args.debug:
        AppConfig.DEBUG_MODE = True
        AppConfig.VERBOSE_LOGGING = True
        AppConfig.LOG_LEVEL = "DEBUG"
        print("🔧 디버그 모드 활성화")
    
    # 시뮬레이션/실제 모드 설정
    if args.simulation:
        AppConfig.SIMULATION_MODE = True
        print("🎮 시뮬레이션 모드로 시작")
    elif args.real:
        AppConfig.SIMULATION_MODE = False
        print("🤖 실제 로봇 모드로 시작 시도")
    
    # 로깅 초기화
    logger = initialize_logging()
    
    if args.debug:
        logger.info("디버그 모드로 실행")
    
    return logger


def print_welcome_message():
    """환영 메시지 출력"""
    print("=" * 60)
    print("🦾 Dobot 로봇팔 제어 시스템 v2.0")
    print("=" * 60)
    print()
    print("주요 개선사항:")
    print("  ✅ YOLOv8 기능 제거 (경량화)")
    print("  ✅ 좌표 동기화 문제 해결")
    print("  ✅ 연결 상태 관리 개선")
    print("  ✅ 현대적인 GUI 인터페이스")
    print("  ✅ 모듈화된 코드 구조")
    print()


def print_system_info():
    """시스템 정보 출력"""
    print("📋 시스템 정보:")
    print(f"  - Python 버전: {sys.version.split()[0]}")
    print(f"  - 플랫폼: {sys.platform}")
    print(f"  - 작업 디렉토리: {current_dir}")
    print(f"  - 로봇 IP: {AppConfig.ROBOT_IP}")
    print(f"  - 시뮬레이션 모드: {'예' if AppConfig.SIMULATION_MODE else '아니오'}")
    print()


def main():
    """메인 함수"""
    # 명령행 인수 파싱
    parser = argparse.ArgumentParser(
        description="Dobot 로봇팔 제어 시스템",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  python run.py                 # 기본 실행
  python run.py --debug         # 디버그 모드
  python run.py --simulation    # 시뮬레이션 모드
  python run.py --real          # 실제 로봇 연결 시도

주의사항:
  - 실제 로봇 연결을 위해서는 Dobot이 네트워크에 연결되어 있어야 합니다.
  - 시뮬레이션 모드에서는 모든 기능을 테스트할 수 있습니다.
        """
    )
    
    parser.add_argument('--debug', action='store_true',
                       help='디버그 모드로 실행 (상세 로그 출력)')
    parser.add_argument('--simulation', action='store_true',
                       help='시뮬레이션 모드 강제 실행')
    parser.add_argument('--real', action='store_true',
                       help='실제 로봇 연결 시도')
    parser.add_argument('--no-gui', action='store_true',
                       help='GUI 없이 실행 (테스트용)')
    parser.add_argument('--version', action='version',
                       version=f'Dobot Controller v{AppConfig.VERSION}')
    
    args = parser.parse_args()
    
    # 상충하는 옵션 체크
    if args.simulation and args.real:
        print("❌ --simulation과 --real 옵션은 동시에 사용할 수 없습니다.")
        sys.exit(1)
    
    # 환영 메시지
    print_welcome_message()
    
    # 의존성 확인
    if not check_dependencies():
        sys.exit(1)
    
    # 선택적 의존성 확인
    check_optional_dependencies()
    
    # 시스템 정보 출력
    if args.debug:
        print_system_info()
    
    # 환경 설정
    logger = setup_environment(args)
    
    try:
        if args.no_gui:
            # GUI 없이 테스트 실행
            print("🧪 GUI 없이 테스트 모드로 실행...")
            from robot_controller import RobotController
            
            robot = RobotController(simulation_mode=True)
            print("✅ 로봇 컨트롤러 테스트 성공")
            
            # 간단한 동작 테스트
            print("🔄 기본 동작 테스트...")
            robot.move_to(250, 0, 50, 0)
            print("✅ 이동 테스트 완료")
            
        else:
            # GUI 모드로 실행
            print("🚀 애플리케이션을 시작합니다...")
            print()
            
            # 메인 애플리케이션 import 및 실행
            from main import DobotApp
            
            app = DobotApp()
            app.run()
    
    except KeyboardInterrupt:
        print("\n\n⏹️  사용자에 의해 중단되었습니다.")
        logger.info("사용자에 의한 애플리케이션 중단")
        
    except ImportError as e:
        print(f"\n❌ 모듈 import 오류: {e}")
        print("필요한 패키지가 설치되어 있는지 확인하세요.")
        logger.error(f"Import 오류: {e}")
        sys.exit(1)
        
    except Exception as e:
        print(f"\n💥 예상치 못한 오류가 발생했습니다: {e}")
        logger.error(f"예상치 못한 오류: {e}", exc_info=True)
        
        if args.debug:
            import traceback
            print("\n📋 상세 오류 정보:")
            traceback.print_exc()
        
        sys.exit(1)
    
    finally:
        print("👋 애플리케이션을 종료합니다.")
        logger.info("애플리케이션 정상 종료")


if __name__ == "__main__":
    main()
