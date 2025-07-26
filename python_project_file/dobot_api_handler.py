"""
robot_controller.py 수정사항

기존 robot_controller.py 파일에서 다음 부분들을 수정하거나 추가하세요.
전체 파일을 교체하지 말고, 기존 코드에서 해당 부분만 수정하면 됩니다.

주요 수정사항:
- 자동 리소스 정리 등록
- 안전한 연결/해제 로직
- 에러 처리 강화
- 시뮬레이션 모드 지원
"""

# ========== 파일 상단에 추가할 import들 ==========
import atexit
import signal
import sys
import time
import logging
from typing import Optional
from dobot_api_handler import DobotAPIHandler  # 개선된 API 핸들러

logger = logging.getLogger(__name__)

# ========== RobotController 클래스 수정사항 ==========

class RobotController:
    def __init__(self, ip_address: str = "192.168.1.6"):
        """
        기존 __init__ 함수에 다음 줄들을 추가하세요
        """
        self.ip_address = ip_address
        self.dobot_api: Optional[DobotAPIHandler] = None
        self.is_simulation_mode = False
        
        # 🔥 자동 정리 등록 (중요!) - 이 줄을 추가
        atexit.register(self.emergency_cleanup)
        
        logger.info(f"RobotController 초기화: {ip_address}")
    
    def emergency_cleanup(self):
        """
        새로 추가할 함수 - 비상시 리소스 정리
        """
        try:
            if self.dobot_api:
                logger.info("비상 정리 실행...")
                self.dobot_api.cleanup_all_connections()
        except Exception as e:
            logger.error(f"비상 정리 중 오류: {e}")
    
    def connect(self) -> bool:
        """
        기존 connect 함수를 이 코드로 교체하세요
        """
        try:
            logger.info("로봇 연결 시작...")
            
            # 🔥 기존 연결이 있으면 완전히 정리 (중요!)
            if self.dobot_api:
                logger.info("기존 연결 정리 중...")
                self.dobot_api.cleanup_all_connections()
                time.sleep(1.0)
            
            # 새 연결 생성
            self.dobot_api = DobotAPIHandler(self.ip_address)
            
            if self.dobot_api.connect_with_retry():
                self.is_simulation_mode = False
                logger.info("✅ 실제 로봇 연결 성공")
                return True
            else:
                logger.warning("⚠️ 실제 로봇 연결 실패, 시뮬레이션 모드로 전환")
                self.is_simulation_mode = True
                return True
                
        except Exception as e:
            logger.error(f"로봇 연결 중 오류: {e}")
            self.is_simulation_mode = True
            return True  # 시뮬레이션 모드로라도 계속 진행
    
    def disconnect(self):
        """
        기존 disconnect 함수를 이 코드로 교체하세요
        """
        try:
            if self.dobot_api:
                logger.info("로봇 연결 해제 시작...")
                self.dobot_api.cleanup_all_connections()
                self.dobot_api = None
            
            self.is_simulation_mode = False
            logger.info("✅ 로봇 연결 해제 완료")
            
        except Exception as e:
            logger.error(f"연결 해제 중 오류: {e}")
    
    def move_to(self, x: float, y: float, z: float, r: float = 0.0, 
                wait_for_completion: bool = True) -> bool:
        """
        기존 move_to 함수를 이 코드로 교체하거나 수정하세요
        """
        # 시뮬레이션 모드 처리
        if self.is_simulation_mode:
            logger.info(f"[시뮬레이션] 이동: ({x}, {y}, {z}, {r})")
            time.sleep(0.5)  # 시뮬레이션 딜레이
            return True
        
        # 연결 상태 확인
        if not self.dobot_api or not self.dobot_api.is_connected:
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            # 좌표 검증 (선택적)
            if not self._validate_coordinates(x, y, z, r):
                logger.error(f"유효하지 않은 좌표: ({x}, {y}, {z}, {r})")
                return False
            
            # 이동 명령 전송
            command = f"MovJ({x},{y},{z},{r})"
            response = self.dobot_api.send_command(command, "move")
            
            if response and "OK" in response.upper():
                logger.info(f"✅ 이동 완료: ({x}, {y}, {z}, {r})")
                return True
            else:
                logger.error(f"❌ 이동 실패: {response}")
                return False
                
        except Exception as e:
            logger.error(f"이동 명령 실패: {e}")
            return False
    
    def _validate_coordinates(self, x: float, y: float, z: float, r: float) -> bool:
        """
        새로 추가할 함수 - 좌표 유효성 검사
        """
        # 작업 공간 제한 (config.py에서 가져오거나 여기서 정의)
        x_min, x_max = -400, 400
        y_min, y_max = -400, 400
        z_min, z_max = -200, 200
        
        if not (x_min <= x <= x_max):
            return False
        if not (y_min <= y <= y_max):
            return False
        if not (z_min <= z <= z_max):
            return False
        
        return True
    
    def gripper_control(self, enable: bool, wait_time: float = 1.5) -> bool:
        """
        그리퍼 제어 함수 - 기존 함수가 있다면 수정, 없다면 추가
        """
        if self.is_simulation_mode:
            logger.info(f"[시뮬레이션] 그리퍼: {'ON' if enable else 'OFF'}")
            time.sleep(wait_time)
            return True
        
        if not self.dobot_api or not self.dobot_api.is_connected:
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            # 그리퍼 명령 전송
            command = f"DO(1,{1 if enable else 0})"
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                time.sleep(wait_time)  # 그리퍼 동작 대기
                logger.info(f"✅ 그리퍼 {'ON' if enable else 'OFF'} 완료")
                return True
            else:
                logger.error(f"❌ 그리퍼 제어 실패: {response}")
                return False
                
        except Exception as e:
            logger.error(f"그리퍼 제어 실패: {e}")
            return False
    
    def get_current_pose(self) -> Optional[Tuple[float, float, float, float]]:
        """
        현재 위치 조회 함수 - 새로 추가하거나 기존 함수 수정
        """
        if self.is_simulation_mode:
            # 시뮬레이션에서는 기본 위치 반환
            return (200.0, 0.0, 100.0, 0.0)
        
        if not self.dobot_api or not self.dobot_api.is_connected:
            logger.error("로봇이 연결되지 않음")
            return None
        
        try:
            command = "GetPose()"
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                # 응답 파싱 (실제 응답 형식에 맞게 수정 필요)
                # 예: "OK,{200.0,0.0,100.0,0.0}"
                pose_data = response.split(',')[1:]  # OK 부분 제거
                if len(pose_data) >= 4:
                    x = float(pose_data[0].strip('{}'))
                    y = float(pose_data[1])
                    z = float(pose_data[2])
                    r = float(pose_data[3].strip('{}'))
                    return (x, y, z, r)
            
            return None
            
        except Exception as e:
            logger.error(f"위치 조회 실패: {e}")
            return None
    
    def home_robot(self) -> bool:
        """
        로봇 홈 위치로 이동 - 새로 추가하거나 기존 함수 수정
        """
        logger.info("로봇을 홈 위치로 이동 중...")
        
        if self.is_simulation_mode:
            logger.info("[시뮬레이션] 홈 위치로 이동 완료")
            time.sleep(2.0)
            return True
        
        if not self.dobot_api or not self.dobot_api.is_connected:
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            # 홈 명령 전송
            command = "EnableRobot()"
            response1 = self.dobot_api.send_command(command, "dashboard")
            
            command = "Home()"
            response2 = self.dobot_api.send_command(command, "dashboard")
            
            if response1 and response2:
                logger.info("✅ 홈 위치로 이동 완료")
                return True
            else:
                logger.error("❌ 홈 이동 실패")
                return False
                
        except Exception as e:
            logger.error(f"홈 이동 실패: {e}")
            return False
    
    def is_robot_connected(self) -> bool:
        """
        연결 상태 확인 함수 - 새로 추가
        """
        if self.is_simulation_mode:
            return True
        
        if not self.dobot_api:
            return False
        
        return self.dobot_api.is_connected and self.dobot_api.check_connection_health()
    
    # ========== Context Manager 지원 (권장) ==========
    
    def __enter__(self):
        """Context manager 진입"""
        if self.connect():
            return self
        else:
            raise ConnectionError("로봇 연결 실패")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 종료"""
        self.disconnect()


# ========== 사용 예제 ==========

def example_usage():
    """
    개선된 로봇 컨트롤러 사용 예제
    """
    
    # 방법 1: Context Manager 사용 (권장)
    try:
        with RobotController() as robot:
            # 로봇 작업 수행
            robot.home_robot()
            robot.move_to(200, 0, 100)
            robot.gripper_control(True)
            robot.move_to(300, 100, 150)
            robot.gripper_control(False)
            # with 블록을 벗어나면 자동으로 연결 해제
            
    except Exception as e:
        logger.error(f"로봇 작업 중 오류: {e}")
    
    # 방법 2: 수동 관리
    robot = RobotController()
    try:
        if robot.connect():
            robot.move_to(200, 0, 100)
            robot.move_to(300, 100, 150)
    finally:
        robot.disconnect()  # 반드시 호출!


# ========== 기존 코드에 추가할 헬퍼 함수들 ==========

def safe_robot_operation(robot: RobotController, operation_func, *args, **kwargs):
    """
    안전한 로봇 동작 실행 헬퍼
    """
    max_retries = 3
    for attempt in range(max_retries):
        try:
            if not robot.is_robot_connected():
                logger.warning("로봇 연결 끊어짐. 재연결 시도...")
                if not robot.connect():
                    continue
            
            result = operation_func(*args, **kwargs)
            if result:
                return True
                
        except Exception as e:
            logger.error(f"동작 실행 실패 (시도 {attempt + 1}/{max_retries}): {e}")
            
        if attempt < max_retries - 1:
            time.sleep(1.0)
    
    return False
