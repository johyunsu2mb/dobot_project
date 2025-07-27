#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot 로봇팔 제어 시스템 테스트 스크립트
시스템 구성 요소들을 개별적으로 테스트하여 정상 동작을 확인합니다.
"""

import sys
import os
import time
import traceback
from typing import List, Tuple, Dict, Any

# 프로젝트 루트 디렉토리를 Python 경로에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)


class SystemTester:
    """시스템 테스트 클래스"""
    
    def __init__(self):
        self.test_results = []
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        
    def run_test(self, test_name: str, test_func, *args, **kwargs) -> bool:
        """개별 테스트 실행"""
        self.total_tests += 1
        print(f"🧪 테스트 실행: {test_name}")
        
        try:
            # 테스트 시작 시간 기록
            start_time = time.time()
            
            # 테스트 함수 실행
            result = test_func(*args, **kwargs)
            
            # 실행 시간 계산
            execution_time = time.time() - start_time
            
            if result:
                print(f"  ✅ 통과 ({execution_time:.2f}초)")
                self.passed_tests += 1
                self.test_results.append({
                    'name': test_name,
                    'status': 'PASS',
                    'time': execution_time,
                    'error': None
                })
            else:
                print(f"  ❌ 실패 ({execution_time:.2f}초)")
                self.failed_tests += 1
                self.test_results.append({
                    'name': test_name,
                    'status': 'FAIL',
                    'time': execution_time,
                    'error': 'Test returned False'
                })
            
            return result
            
        except Exception as e:
            execution_time = time.time() - start_time
            print(f"  💥 예외 발생: {str(e)} ({execution_time:.2f}초)")
            self.failed_tests += 1
            self.test_results.append({
                'name': test_name,
                'status': 'ERROR',
                'time': execution_time,
                'error': str(e)
            })
            return False
    
    def print_summary(self):
        """테스트 결과 요약 출력"""
        print("\n" + "="*60)
        print("📊 테스트 결과 요약")
        print("="*60)
        print(f"총 테스트: {self.total_tests}")
        print(f"통과: {self.passed_tests} ✅")
        print(f"실패: {self.failed_tests} ❌")
        print(f"성공률: {(self.passed_tests/self.total_tests*100):.1f}%")
        
        if self.failed_tests > 0:
            print("\n🔍 실패한 테스트:")
            for result in self.test_results:
                if result['status'] != 'PASS':
                    print(f"  - {result['name']}: {result['error']}")
        
        print("\n⏱️  실행 시간:")
        total_time = sum(r['time'] for r in self.test_results)
        print(f"  전체: {total_time:.2f}초")
        
        if self.test_results:
            avg_time = total_time / len(self.test_results)
            print(f"  평균: {avg_time:.2f}초")


def test_imports():
    """필수 모듈 import 테스트"""
    try:
        import tkinter
        import numpy
        import matplotlib
        import yaml
        
        # YOLOv8 관련 패키지 (선택적)
        try:
            import cv2
            import PIL
            from ultralytics import YOLO
            print("    ✅ YOLOv8 관련 패키지 모두 설치됨")
        except ImportError as e:
            print(f"    ⚠️  YOLOv8 패키지 일부 누락: {e}")
            print("    💡 'pip install opencv-python pillow ultralytics' 로 설치하세요")
        
        return True
    except ImportError as e:
        print(f"    누락된 패키지: {e}")
        return False


def test_config_module():
    """설정 모듈 테스트"""
    try:
        from config import AppConfig, DobotCommands, ErrorCodes
        
        # 기본 설정값 확인
        assert hasattr(AppConfig, 'ROBOT_IP')
        assert hasattr(AppConfig, 'HOME_POSITION')
        assert hasattr(AppConfig, 'FURNITURE_POSITIONS')
        
        # 좌표 검증 함수 테스트
        assert AppConfig.validate_coordinates(250, 0, 50, 0) == True
        assert AppConfig.validate_coordinates(1000, 0, 50, 0) == False  # 범위 초과
        
        # 명령어 생성 테스트
        move_cmd = DobotCommands.move_j(250, 0, 50, 0)
        assert "MovJ" in move_cmd
        
        # 에러 메시지 테스트
        error_msg = ErrorCodes.get_message(ErrorCodes.CONNECTION_FAILED)
        assert len(error_msg) > 0
        
        return True
    except Exception as e:
        print(f"    설정 모듈 오류: {e}")
        return False


def test_utils_module():
    """유틸리티 모듈 테스트"""
    try:
        from utils import CoordinateManager, ErrorHandler, PerformanceMonitor
        
        # 좌표 관리자 테스트
        coord_mgr = CoordinateManager()
        assert coord_mgr.validate_coordinates(250, 0, 50, 0) == True
        
        distance = coord_mgr.calculate_distance([0, 0, 0], [100, 0, 0])
        assert distance == 100.0
        
        coord_mgr.update_position([250, 0, 50, 0])
        assert coord_mgr.current_position == [250, 0, 50, 0]
        
        # 에러 핸들러 테스트
        error_handler = ErrorHandler()
        assert hasattr(error_handler, 'handle_error')
        
        # 성능 모니터 테스트
        perf_monitor = PerformanceMonitor()
        perf_monitor.start_timer('test')
        time.sleep(0.1)
        exec_time = perf_monitor.end_timer('test')
        assert exec_time >= 0.1
        
        return True
    except Exception as e:
        print(f"    유틸리티 모듈 오류: {e}")
        return False


def test_logger_module():
    """로깅 모듈 테스트"""
    try:
        from logger_setup import setup_logger, MemoryHandler
        
        # 로거 생성 테스트
        logger = setup_logger('test_logger')
        assert logger is not None
        
        # 메모리 핸들러 테스트
        memory_handler = MemoryHandler(max_records=10)
        logger.addHandler(memory_handler)
        
        # 로그 메시지 테스트
        logger.info("테스트 메시지")
        logger.warning("경고 메시지")
        logger.error("에러 메시지")
        
        records = memory_handler.get_records()
        assert len(records) >= 3
        
        return True
    except Exception as e:
        print(f"    로깅 모듈 오류: {e}")
        return False


def test_robot_controller():
    """로봇 컨트롤러 테스트 (시뮬레이션 모드)"""
    try:
        from robot_controller import RobotController, RobotState
        
        # 시뮬레이션 모드로 로봇 생성
        robot = RobotController(simulation_mode=True)
        assert robot.simulation_mode == True
        
        # 연결 테스트
        connected = robot.connect()
        assert connected == True
        
        # 상태 확인
        assert robot.get_state() == RobotState.CONNECTED
        
        # 이동 테스트
        success = robot.move_to(250, 0, 50, 0)
        assert success == True
        
        # 현재 위치 확인
        position = robot.get_current_position()
        assert position is not None
        assert len(position) == 4
        
        # 그리퍼 테스트
        success = robot.set_gripper(True)
        assert success == True
        
        success = robot.set_gripper(False)
        assert success == True
        
        # 연결 해제
        robot.disconnect()
        
        return True
    except Exception as e:
        print(f"    로봇 컨트롤러 오류: {e}")
        return False


def test_ui_components():
    """UI 컴포넌트 테스트 (GUI 없이)"""
    try:
        # GUI 환경이 없을 수 있으므로 import만 테스트
        from ui_components import ModernUI, CoordinateFrame, FurnitureFrame
        
        # 클래스가 정의되어 있는지 확인
        assert ModernUI is not None
        assert CoordinateFrame is not None
        assert FurnitureFrame is not None
        
        return True
    except Exception as e:
        print(f"    UI 컴포넌트 오류: {e}")
        return False


def test_pickup_sequence():
    """픽업 시퀀스 테스트"""
    try:
        from robot_controller import RobotController
        from config import AppConfig
        
        # 시뮬레이션 로봇 생성
        robot = RobotController(simulation_mode=True)
        robot.connect()
        
        # 가구 위치 정보 가져오기
        furniture_pos = AppConfig.FURNITURE_POSITIONS['sofa']
        pickup_pos = furniture_pos['pickup']
        place_pos = furniture_pos['place']
        
        # 픽업 시퀀스 실행
        success = robot.pickup_sequence(pickup_pos, place_pos)
        assert success == True
        
        robot.disconnect()
        return True
        
    except Exception as e:
        print(f"    픽업 시퀀스 오류: {e}")
        return False


def test_coordinate_validation():
    """좌표 검증 테스트"""
    try:
        from config import AppConfig
        
        # 유효한 좌표들
        valid_coords = [
            (250, 0, 50, 0),      # 홈 위치
            (300, 100, 20, 45),   # 일반적인 위치
            (-200, -200, 100, -90), # 경계 근처
        ]
        
        for x, y, z, r in valid_coords:
            assert AppConfig.validate_coordinates(x, y, z, r) == True
        
        # 무효한 좌표들
        invalid_coords = [
            (500, 0, 50, 0),      # X 범위 초과
            (0, 500, 50, 0),      # Y 범위 초과
            (0, 0, 300, 0),       # Z 범위 초과
            (0, 0, 50, 200),      # R 범위 초과
        ]
        
        for x, y, z, r in invalid_coords:
            assert AppConfig.validate_coordinates(x, y, z, r) == False
        
        return True
        
    except Exception as e:
        print(f"    좌표 검증 오류: {e}")
        return False


def test_error_handling():
    """에러 처리 테스트"""
    try:
        from utils import ErrorHandler
        from config import ErrorCodes
        
        error_handler = ErrorHandler()
        
        # 가상의 에러 생성 및 처리
        test_error = Exception("테스트 에러")
        result = error_handler.handle_error(test_error, "테스트 컨텍스트", show_dialog=False)
        
        # 에러 카운트 확인
        assert error_handler.error_count > 0
        
        # 에러 카운트 리셋
        error_handler.reset_error_count()
        assert error_handler.error_count == 0
        
        return True
        
    except Exception as e:
        print(f"    에러 처리 오류: {e}")
        return False


def test_performance_monitoring():
    """성능 모니터링 테스트"""
    try:
        from utils import PerformanceMonitor
        
        monitor = PerformanceMonitor()
        
        # 여러 작업의 실행 시간 측정
        operations = ['operation1', 'operation2', 'operation3']
        
        for op in operations:
            monitor.start_timer(op)
            time.sleep(0.01)  # 짧은 대기
            exec_time = monitor.end_timer(op)
            assert exec_time >= 0.01
        
        # 평균 시간 계산
        avg_time = monitor.get_average_time('operation1')
        assert avg_time >= 0.01
        
        # 성능 리포트 생성
        report = monitor.get_performance_report()
        assert len(report) == len(operations)
        
        return True
        
    except Exception as e:
        print(f"    성능 모니터링 오류: {e}")
        return False


def main():
    """메인 테스트 함수"""
    print("🚀 Dobot 로봇팔 제어 시스템 테스트 시작")
    print("=" * 60)
    
    tester = SystemTester()
    
    # 기본 테스트들
    tester.run_test("필수 패키지 import", test_imports)
    tester.run_test("설정 모듈", test_config_module)
    tester.run_test("유틸리티 모듈", test_utils_module)
    tester.run_test("로깅 모듈", test_logger_module)
    
    # 핵심 기능 테스트들
    tester.run_test("로봇 컨트롤러 (시뮬레이션)", test_robot_controller)
    tester.run_test("UI 컴포넌트", test_ui_components)
    tester.run_test("YOLOv8 시스템", test_yolo_system)
    tester.run_test("카메라 감지", test_camera_detection)
    tester.run_test("픽업 시퀀스", test_pickup_sequence)
    
    # 검증 및 에러 처리 테스트들
    tester.run_test("좌표 검증", test_coordinate_validation)
    tester.run_test("에러 처리", test_error_handling)
    tester.run_test("성능 모니터링", test_performance_monitoring)
    
    # 결과 요약
    tester.print_summary()
    
    # 전체적인 시스템 상태 평가
    print("\n🎯 시스템 상태 평가:")
    
    if tester.failed_tests == 0:
        print("  🟢 모든 테스트 통과 - 시스템이 정상적으로 동작합니다!")
        print("  ✨ 'python run.py'로 애플리케이션을 시작할 수 있습니다.")
        return 0
        
    elif tester.failed_tests <= 2:
        print("  🟡 일부 테스트 실패 - 기본 기능은 동작할 수 있습니다.")
        print("  ⚠️  실패한 테스트를 확인하고 문제를 해결하세요.")
        return 1
        
    else:
        print("  🔴 다수 테스트 실패 - 시스템에 문제가 있습니다.")
        print("  🛠️  requirements.txt 설치 및 코드 확인이 필요합니다.")
        return 2


if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
        
    except KeyboardInterrupt:
        print("\n\n⏹️  테스트가 사용자에 의해 중단되었습니다.")
        sys.exit(1)
        
    except Exception as e:
        print(f"\n💥 테스트 실행 중 예상치 못한 오류: {e}")
        print("\n📋 상세 오류 정보:")
        traceback.print_exc()
        sys.exit(3)
