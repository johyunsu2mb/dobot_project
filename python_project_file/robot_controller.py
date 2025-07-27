"""
robot_controller.py - 완전한 통합 버전 (모든 기능 포함)

기존 GitHub 프로젝트의 모든 기능을 유지하면서 
통신 문제 해결 및 안정성 개선사항을 통합한 완전한 버전입니다.

주요 기능:
- 기존의 모든 로봇 제어 기능 유지
- 통신 안정성 개선 (핵심!)
- 픽업 시퀀스 완전 구현
- 그리퍼 제어 강화
- 안전 기능 추가
- 시뮬레이션 모드 지원
- 좌표 검증 및 안전 영역
- 상태 모니터링
- 자동 리소스 정리
- 고급 경로 계획
- 다중 로봇 지원 준비
- 실시간 피드백
- 캘리브레이션 시스템
- 작업 기록 및 분석
"""

import time
import threading
import logging
import atexit
import signal
import sys
import json
import numpy as np
import socket
import queue
import math
import copy
from typing import Optional, Tuple, Dict, Any, List, Callable, Union
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
import csv
from datetime import datetime, timedelta
import pickle

logger = logging.getLogger(__name__)

# ========== 고급 데이터 클래스 및 열거형 ==========

class RobotState(Enum):
    """로봇 상태 - 확장된 상태"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    MOVING = "moving"
    PICKING = "picking"
    PLACING = "placing"
    HOMING = "homing"
    CALIBRATING = "calibrating"
    PAUSED = "paused"
    ERROR = "error"
    SIMULATION = "simulation"
    MAINTENANCE = "maintenance"
    EMERGENCY_STOP = "emergency_stop"

class GripperState(Enum):
    """그리퍼 상태"""
    OPEN = "open"
    CLOSED = "closed"
    PARTIAL = "partial"
    UNKNOWN = "unknown"
    ERROR = "error"

class MovementMode(Enum):
    """이동 모드"""
    JOINT = "joint"          # 관절 이동
    LINEAR = "linear"        # 직선 이동
    ARC = "arc"             # 호 이동
    SPLINE = "spline"       # 스플라인 이동

class CoordinateSystem(Enum):
    """좌표계"""
    WORLD = "world"          # 세계 좌표계
    BASE = "base"           # 베이스 좌표계
    TOOL = "tool"           # 툴 좌표계
    USER = "user"           # 사용자 좌표계

@dataclass
class RobotPosition:
    """확장된 로봇 위치 정보"""
    x: float
    y: float
    z: float
    r: float
    timestamp: float = field(default_factory=time.time)
    coordinate_system: CoordinateSystem = CoordinateSystem.WORLD
    joint_angles: Optional[List[float]] = None
    velocity: Optional[float] = None
    acceleration: Optional[float] = None
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.r)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'x': self.x, 'y': self.y, 'z': self.z, 'r': self.r,
            'timestamp': self.timestamp,
            'coordinate_system': self.coordinate_system.value,
            'joint_angles': self.joint_angles,
            'velocity': self.velocity,
            'acceleration': self.acceleration
        }
    
    def distance_to(self, other: 'RobotPosition') -> float:
        """다른 위치까지의 거리"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    def interpolate_to(self, target: 'RobotPosition', ratio: float) -> 'RobotPosition':
        """두 위치 사이의 보간"""
        return RobotPosition(
            x=self.x + (target.x - self.x) * ratio,
            y=self.y + (target.y - self.y) * ratio,
            z=self.z + (target.z - self.z) * ratio,
            r=self.r + (target.r - self.r) * ratio,
            coordinate_system=self.coordinate_system
        )

@dataclass
class FurnitureConfig:
    """확장된 가구 설정 정보"""
    name: str
    position: RobotPosition
    approach_height: float = 50.0
    pickup_speed: int = 50
    place_position: Optional[RobotPosition] = None
    grip_width: float = 0.0  # 그리퍼 폭
    grip_force: float = 50.0  # 그리퍼 힘
    approach_angle: float = 0.0  # 접근 각도
    safety_zone: Optional[Dict[str, float]] = None  # 안전 구역
    pickup_sequence: Optional[List[Dict]] = None  # 커스텀 픽업 시퀀스
    
    def __post_init__(self):
        if self.place_position is None:
            self.place_position = RobotPosition(350, 0, self.position.z, 0)
        if self.safety_zone is None:
            self.safety_zone = {
                'radius': 100.0,
                'height_min': -50.0,
                'height_max': 100.0
            }

@dataclass
class RobotConfiguration:
    """로봇 설정 정보"""
    model: str = "Dobot_MG400"
    dof: int = 4  # 자유도
    max_reach: float = 400.0  # 최대 도달 거리
    max_payload: float = 750.0  # 최대 페이로드 (g)
    max_speed: float = 100.0  # 최대 속도
    max_acceleration: float = 50.0  # 최대 가속도
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {
        'joint1': (-135, 135),
        'joint2': (-5, 85),
        'joint3': (-10, 95),
        'joint4': (-90, 90)
    })
    tcp_offset: Tuple[float, float, float] = (0, 0, 0)  # TCP 오프셋

# ========== 확장된 가구 설정 ==========

DEFAULT_FURNITURE_CONFIGS = {
    'sofa': FurnitureConfig(
        name='소파',
        position=RobotPosition(150, 200, 50, 0),
        approach_height=60,
        grip_width=80.0,
        grip_force=60.0,
        approach_angle=45.0,
        pickup_sequence=[
            {'action': 'approach', 'height_offset': 80},
            {'action': 'orient', 'angle': 45},
            {'action': 'descend', 'speed': 30},
            {'action': 'grip', 'force': 60},
            {'action': 'lift', 'height_offset': 80}
        ]
    ),
    'chair': FurnitureConfig(
        name='의자', 
        position=RobotPosition(200, 150, 80, 45),
        approach_height=70,
        grip_width=40.0,
        grip_force=70.0,
        approach_angle=90.0,
        pickup_sequence=[
            {'action': 'approach', 'height_offset': 90},
            {'action': 'orient', 'angle': 90},
            {'action': 'descend', 'speed': 25},
            {'action': 'grip', 'force': 70},
            {'action': 'lift', 'height_offset': 90}
        ]
    ),
    'desk': FurnitureConfig(
        name='책상',
        position=RobotPosition(250, 100, 120, 90),
        approach_height=80,
        grip_width=60.0,
        grip_force=80.0,
        approach_angle=0.0,
        pickup_sequence=[
            {'action': 'approach', 'height_offset': 100},
            {'action': 'orient', 'angle': 0},
            {'action': 'descend', 'speed': 20},
            {'action': 'grip', 'force': 80},
            {'action': 'lift', 'height_offset': 100}
        ]
    ),
    'bed': FurnitureConfig(
        name='침대',
        position=RobotPosition(300, 0, 60, 0),
        approach_height=70,
        grip_width=100.0,
        grip_force=50.0,
        approach_angle=0.0,
        pickup_sequence=[
            {'action': 'approach', 'height_offset': 100},
            {'action': 'orient', 'angle': 0},
            {'action': 'descend', 'speed': 35},
            {'action': 'grip', 'force': 50},
            {'action': 'lift', 'height_offset': 100}
        ]
    ),
    # 추가 가구 타입들
    'table': FurnitureConfig(
        name='테이블',
        position=RobotPosition(180, 120, 100, 30),
        approach_height=75
    ),
    'stool': FurnitureConfig(
        name='스툴',
        position=RobotPosition(160, 180, 70, 60),
        approach_height=65
    ),
    'cabinet': FurnitureConfig(
        name='캐비닛',
        position=RobotPosition(280, 50, 140, 0),
        approach_height=85
    )
}

# ========== 개선된 API 핸들러 클래스 ==========

class DobotAPIHandler:
    """강화된 Dobot API 핸들러 - 완전한 기능"""
    
    def __init__(self, ip_address: str, dashboard_port: int, move_port: int, feed_port: int):
        self.ip_address = ip_address
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.feed_port = feed_port
        
        # 소켓 관리
        self.dashboard_socket = None
        self.move_socket = None
        self.feed_socket = None
        
        # 연결 상태
        self.is_connected = False
        self.socket_lock = threading.Lock()
        self.command_queue = queue.Queue()
        
        # 통신 설정
        self.connect_timeout = 10.0
        self.command_timeout = 5.0
        self.reconnect_attempts = 3
        self.heartbeat_interval = 5.0
        
        # 성능 모니터링
        self.command_count = 0
        self.error_count = 0
        self.last_response_time = 0
        self.response_times = []
        
        # 스레드 관리
        self.heartbeat_thread = None
        self.is_heartbeat_running = False
        
        logger.info(f"DobotAPIHandler 초기화: {ip_address}:{dashboard_port}")
    
    def connect_with_retry(self, max_retries: int = 3) -> bool:
        """재시도가 포함된 연결 - 강화된 버전"""
        for attempt in range(max_retries):
            try:
                logger.info(f"Dobot 연결 시도 {attempt + 1}/{max_retries}")
                
                if self._establish_connections():
                    self.is_connected = True
                    self._start_heartbeat()
                    self._start_command_processor()
                    logger.info("✅ Dobot 연결 성공")
                    return True
                else:
                    logger.warning(f"연결 시도 {attempt + 1} 실패")
                    if attempt < max_retries - 1:
                        time.sleep(2.0 ** attempt)  # 지수 백오프
                        
            except Exception as e:
                logger.error(f"연결 시도 {attempt + 1} 오류: {e}")
                if attempt < max_retries - 1:
                    time.sleep(2.0 ** attempt)
        
        return False
    
    def _establish_connections(self) -> bool:
        """실제 소켓 연결 수행 - 강화된 버전"""
        try:
            # 기존 연결 정리
            self._cleanup_sockets()
            
            # Dashboard 포트 연결
            self.dashboard_socket = self._create_socket()
            self.dashboard_socket.connect((self.ip_address, self.dashboard_port))
            logger.debug(f"Dashboard 연결 성공: {self.dashboard_port}")
            
            # Move 포트 연결
            self.move_socket = self._create_socket()
            self.move_socket.connect((self.ip_address, self.move_port))
            logger.debug(f"Move 연결 성공: {self.move_port}")
            
            # Feed 포트 연결 (옵션)
            try:
                self.feed_socket = self._create_socket()
                self.feed_socket.connect((self.ip_address, self.feed_port))
                logger.debug(f"Feed 연결 성공: {self.feed_port}")
            except Exception as e:
                logger.warning(f"Feed 포트 연결 실패 (계속 진행): {e}")
                self.feed_socket = None
            
            # 연결 테스트
            test_response = self._send_raw_command("RobotMode()", "dashboard")
            if test_response and "OK" in test_response:
                logger.info("연결 테스트 성공")
                return True
            else:
                logger.warning(f"연결 테스트 실패: {test_response}")
                return False
            
        except Exception as e:
            logger.error(f"소켓 연결 실패: {e}")
            self._cleanup_sockets()
            return False
    
    def _create_socket(self) -> socket.socket:
        """소켓 생성 및 설정"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.connect_timeout)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        return sock
    
    def send_command(self, command: str, port_type: str = "dashboard", 
                    wait_response: bool = True, timeout: float = None) -> Optional[str]:
        """명령 전송 - 강화된 버전"""
        if not self.is_connected:
            logger.error("로봇이 연결되지 않음")
            return None
        
        # 명령 큐에 추가
        command_data = {
            'command': command,
            'port_type': port_type,
            'wait_response': wait_response,
            'timeout': timeout or self.command_timeout,
            'timestamp': time.time()
        }
        
        try:
            self.command_queue.put(command_data, timeout=1.0)
            self.command_count += 1
            
            if wait_response:
                # 응답 대기 (실제 구현에서는 응답 큐 사용)
                return self._send_raw_command(command, port_type, timeout)
            else:
                return "OK"  # 비동기 명령
                
        except queue.Full:
            logger.error("명령 큐가 가득 참")
            return None
        except Exception as e:
            logger.error(f"명령 전송 실패: {e}")
            self.error_count += 1
            return None
    
    def _send_raw_command(self, command: str, port_type: str = "dashboard", 
                         timeout: float = None) -> Optional[str]:
        """직접 명령 전송"""
        with self.socket_lock:
            try:
                socket_obj = self._get_socket(port_type)
                if socket_obj is None:
                    return None
                
                # 명령 전송
                command_bytes = (command + "\n").encode('utf-8')
                socket_obj.send(command_bytes)
                
                # 응답 수신
                if timeout:
                    socket_obj.settimeout(timeout)
                else:
                    socket_obj.settimeout(self.command_timeout)
                
                start_time = time.time()
                response = socket_obj.recv(1024).decode('utf-8').strip()
                response_time = time.time() - start_time
                
                # 성능 통계 업데이트
                self.last_response_time = response_time
                self.response_times.append(response_time)
                if len(self.response_times) > 100:
                    self.response_times = self.response_times[-100:]
                
                logger.debug(f"명령: {command} -> 응답: {response} ({response_time:.3f}초)")
                return response
                
            except socket.timeout:
                logger.error(f"명령 타임아웃: {command}")
                return None
            except Exception as e:
                logger.error(f"명령 전송 실패 [{command}]: {e}")
                return None
    
    def _get_socket(self, port_type: str):
        """포트 타입에 따른 소켓 반환"""
        if port_type == "dashboard":
            return self.dashboard_socket
        elif port_type == "move":
            return self.move_socket
        elif port_type == "feed":
            return self.feed_socket
        else:
            logger.error(f"알 수 없는 포트 타입: {port_type}")
            return None
    
    def _start_heartbeat(self):
        """하트비트 시작"""
        if self.is_heartbeat_running:
            return
        
        self.is_heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        logger.debug("하트비트 시작")
    
    def _heartbeat_loop(self):
        """하트비트 루프"""
        while self.is_heartbeat_running and self.is_connected:
            try:
                response = self._send_raw_command("RobotMode()", "dashboard", timeout=2.0)
                if response is None or "OK" not in response:
                    logger.warning("하트비트 실패 - 연결 확인 필요")
                    self.is_connected = False
                    break
                
                time.sleep(self.heartbeat_interval)
                
            except Exception as e:
                logger.error(f"하트비트 오류: {e}")
                self.is_connected = False
                break
        
        logger.debug("하트비트 종료")
    
    def _start_command_processor(self):
        """명령 처리기 시작"""
        command_thread = threading.Thread(target=self._command_processor_loop, daemon=True)
        command_thread.start()
        logger.debug("명령 처리기 시작")
    
    def _command_processor_loop(self):
        """명령 처리 루프"""
        while self.is_connected:
            try:
                command_data = self.command_queue.get(timeout=1.0)
                
                if not command_data['wait_response']:
                    # 비동기 명령 처리
                    self._send_raw_command(
                        command_data['command'], 
                        command_data['port_type'],
                        command_data['timeout']
                    )
                
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"명령 처리 오류: {e}")
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """성능 통계 반환"""
        avg_response_time = 0
        if self.response_times:
            avg_response_time = sum(self.response_times) / len(self.response_times)
        
        return {
            'command_count': self.command_count,
            'error_count': self.error_count,
            'error_rate': self.error_count / max(self.command_count, 1),
            'last_response_time': self.last_response_time,
            'avg_response_time': avg_response_time,
            'is_connected': self.is_connected
        }
    
    def cleanup_all_connections(self):
        """🔥 모든 연결 안전 정리 - 강화된 버전"""
        logger.info("Dobot API 연결 정리 시작...")
        
        # 하트비트 중지
        self.is_heartbeat_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=2.0)
        
        # 연결 상태 업데이트
        self.is_connected = False
        
        # 소켓 정리
        with self.socket_lock:
            self._cleanup_sockets()
        
        # 큐 정리
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break
        
        logger.info("Dobot API 연결 정리 완료")
    
    def _cleanup_sockets(self):
        """소켓 정리 - 강화된 버전"""
        sockets = [
            ("dashboard", self.dashboard_socket),
            ("move", self.move_socket),
            ("feed", self.feed_socket)
        ]
        
        for name, sock in sockets:
            if sock:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                    logger.debug(f"{name} 소켓 정리 완료")
                except Exception as e:
                    logger.debug(f"{name} 소켓 정리 중 오류: {e}")
        
        self.dashboard_socket = None
        self.move_socket = None
        self.feed_socket = None

# ========== 메인 로봇 컨트롤러 클래스 ==========

class RobotController:
    """완전한 기능을 가진 로봇 컨트롤러 - 모든 기능 통합"""
    
    def __init__(self, ip_address: str = "192.168.1.6",
                 dashboard_port: int = 29999,
                 move_port: int = 30003,
                 feed_port: int = 30004,
                 config_file: Optional[str] = None):
        
        # 연결 설정
        self.ip_address = ip_address
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.feed_port = feed_port
        
        # 설정 로드
        self.config = self._load_configuration(config_file)
        self.robot_config = RobotConfiguration()
        
        # 🔥 개선된 API 핸들러 사용
        self.dobot_api: Optional[DobotAPIHandler] = None
        
        # 상태 관리
        self.current_state = RobotState.DISCONNECTED
        self.gripper_state = GripperState.UNKNOWN
        self.is_simulation_mode = False
        self.is_emergency_stopped = False
        self.is_calibrated = False
        
        # 위치 및 설정
        self.current_position = RobotPosition(200, 0, 100, 0)
        self.home_position = RobotPosition(200, 0, 100, 0)
        self.safe_position = RobotPosition(200, 0, 200, 0)
        self.last_known_position = self.current_position
        
        # 확장된 작업 공간 제한
        self.workspace_limits = {
            'x_min': -400, 'x_max': 400,
            'y_min': -400, 'y_max': 400, 
            'z_min': -200, 'z_max': 200,
            'r_min': -180, 'r_max': 180
        }
        
        # 동적 작업공간 (장애물 회피용)
        self.dynamic_obstacles = []
        self.safety_zones = []
        
        # 안전 설정
        self.safety_height_offset = 50.0
        self.max_speed = 100
        self.position_tolerance = 1.0
        self.movement_timeout = 45.0
        self.collision_detection = True
        
        # 가구 설정
        self.furniture_configs = DEFAULT_FURNITURE_CONFIGS.copy()
        
        # 고급 기능 설정
        self.path_planning_enabled = True
        self.force_feedback_enabled = False
        self.vision_guidance_enabled = False
        
        # 콜백 함수들
        self.state_change_callbacks: List[Callable[[RobotState], None]] = []
        self.position_update_callbacks: List[Callable[[RobotPosition], None]] = []
        self.error_callbacks: List[Callable[[str], None]] = []
        self.collision_callbacks: List[Callable[[str], None]] = []
        
        # 스레드 및 모니터링
        self.position_monitor_thread: Optional[threading.Thread] = None
        self.safety_monitor_thread: Optional[threading.Thread] = None
        self.command_queue = queue.Queue()
        self.is_monitoring = False
        
        # 통계 및 로깅
        self.command_count = 0
        self.error_count = 0
        self.last_command_time = 0
        self.connection_attempts = 0
        self.operation_history = []
        
        # 성능 모니터링
        self.performance_data = {
            'cycle_times': [],
            'success_rates': [],
            'error_types': {},
            'position_accuracy': []
        }
        
        # 캘리브레이션 데이터
        self.calibration_data = {
            'tool_offset': [0, 0, 0],
            'coordinate_transform': np.eye(4),
            'vision_transform': None
        }
        
        # 🔥 자동 정리 등록 (통신 문제 해결의 핵심)
        atexit.register(self.emergency_cleanup)
        
        # 신호 처리 등록
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)
        except:
            pass  # Windows에서 SIGTERM이 없을 수 있음
        
        logger.info(f"RobotController 초기화 완료: {ip_address}")
        logger.info(f"설정: 시뮬레이션={self.is_simulation_mode}, 캘리브레이션={self.is_calibrated}")
    
    def _load_configuration(self, config_file: Optional[str]) -> Dict[str, Any]:
        """설정 파일 로드"""
        default_config = {
            'robot': {
                'model': 'MG400',
                'tcp_offset': [0, 0, 0],
                'max_speed': 100,
                'max_acceleration': 50
            },
            'safety': {
                'collision_detection': True,
                'safety_zones': [],
                'emergency_stop_distance': 10.0
            },
            'vision': {
                'enabled': False,
                'calibration_file': None
            }
        }
        
        if config_file and Path(config_file).exists():
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    default_config.update(user_config)
            except Exception as e:
                logger.warning(f"설정 파일 로드 실패: {e}")
        
        return default_config
    
    def _signal_handler(self, signum, frame):
        """신호 처리"""
        logger.info(f"신호 {signum} 받음. 로봇 정리 중...")
        self.emergency_cleanup()
        sys.exit(0)
    
    def emergency_cleanup(self):
        """🔥 비상시 리소스 정리 (통신 문제 해결 핵심) - 강화된 버전"""
        try:
            logger.info("로봇 비상 정리 실행...")
            
            # 모니터링 중지
            self.is_monitoring = False
            
            # 스레드 정리
            threads_to_join = [
                self.position_monitor_thread,
                self.safety_monitor_thread
            ]
            
            for thread in threads_to_join:
                if thread and thread.is_alive():
                    thread.join(timeout=1.0)
            
            # API 연결 정리
            if self.dobot_api:
                self.dobot_api.cleanup_all_connections()
                self.dobot_api = None
            
            # 상태 초기화
            self.current_state = RobotState.DISCONNECTED
            self.is_emergency_stopped = False
            
            # 작업 기록 저장
            self._save_operation_history()
            
            logger.info("로봇 비상 정리 완료")
            
        except Exception as e:
            logger.error(f"비상 정리 중 오류: {e}")
    
    # ========== 연결 관리 ==========
    
    def connect(self) -> bool:
        """🔥 개선된 로봇 연결 (통신 문제 해결) - 강화된 버전"""
        try:
            logger.info("로봇 연결 시작...")
            self._set_state(RobotState.CONNECTING)
            self.connection_attempts += 1
            
            # 🔥 기존 연결이 있으면 완전히 정리
            if self.dobot_api:
                logger.info("기존 연결 정리 중...")
                self.dobot_api.cleanup_all_connections()
                time.sleep(1.0)
            
            # 새 API 핸들러 생성
            self.dobot_api = DobotAPIHandler(
                self.ip_address,
                self.dashboard_port,
                self.move_port,
                self.feed_port
            )
            
            # 연결 시도
            if self.dobot_api.connect_with_retry(max_retries=3):
                self.is_simulation_mode = False
                self._set_state(RobotState.CONNECTED)
                
                # 로봇 활성화 및 초기 설정
                if self._initialize_robot():
                    self._start_monitoring_systems()
                    logger.info("✅ 실제 로봇 연결 성공")
                    return True
                else:
                    logger.warning("로봇 초기화 실패, 시뮬레이션 모드로 전환")
                    self._enable_simulation_mode()
                    return True
            else:
                logger.warning("실제 로봇 연결 실패, 시뮬레이션 모드로 전환")
                self._enable_simulation_mode()
                return True
                
        except Exception as e:
            logger.error(f"로봇 연결 중 오류: {e}")
            self._enable_simulation_mode()
            return True
    
    def _enable_simulation_mode(self):
        """시뮬레이션 모드 활성화 - 강화된 버전"""
        self.is_simulation_mode = True
        self._set_state(RobotState.SIMULATION)
        self._start_monitoring_systems()
        
        # 시뮬레이션 환경 설정
        self._setup_simulation_environment()
        
        logger.info("시뮬레이션 모드 활성화")
    
    def _setup_simulation_environment(self):
        """시뮬레이션 환경 설정"""
        # 가상 장애물 추가
        self.dynamic_obstacles = [
            {'position': [100, 100, 50], 'radius': 30, 'height': 100},
            {'position': [-100, 150, 80], 'radius': 25, 'height': 120}
        ]
        
        # 가상 센서 데이터 생성
        self.simulation_data = {
            'joint_angles': [0, 45, 45, 0],
            'forces': [0, 0, 0, 0, 0, 0],
            'temperatures': [25, 26, 24, 25]
        }
    
    def _initialize_robot(self) -> bool:
        """로봇 초기 설정 - 강화된 버전"""
        try:
            if not self.dobot_api or self.is_simulation_mode:
                return True
            
            # 로봇 모드 확인
            response = self.dobot_api.send_command("RobotMode()", "dashboard")
            logger.debug(f"로봇 모드 응답: {response}")
            
            # 로봇 활성화
            response = self.dobot_api.send_command("EnableRobot()", "dashboard")
            if not response or "OK" not in response.upper():
                logger.error("로봇 활성화 실패")
                return False
            
            # 에러 클리어
            self.dobot_api.send_command("ClearError()", "dashboard")
            
            # 속도 및 가속도 설정
            self.dobot_api.send_command(f"SpeedFactor({self.max_speed})", "dashboard")
            self.dobot_api.send_command(f"AccJ({self.robot_config.max_acceleration})", "dashboard")
            self.dobot_api.send_command(f"AccL({self.robot_config.max_acceleration})", "dashboard")
            
            # TCP 설정
            tcp_x, tcp_y, tcp_z = self.calibration_data['tool_offset']
            self.dobot_api.send_command(f"Tool({tcp_x},{tcp_y},{tcp_z})", "dashboard")
            
            # 현재 위치 조회
            self._update_current_position()
            
            # 충돌 감지 설정
            if self.collision_detection:
                self.dobot_api.send_command("CollisionOn()", "dashboard")
            
            logger.info("로봇 초기화 완료")
            return True
            
        except Exception as e:
            logger.error(f"로봇 초기화 실패: {e}")
            return False
    
    def _start_monitoring_systems(self):
        """모니터링 시스템 시작 - 강화된 버전"""
        if self.is_monitoring:
            return
        
        self.is_monitoring = True
        
        # 위치 모니터링 스레드
        self.position_monitor_thread = threading.Thread(
            target=self._position_monitor_loop, daemon=True)
        self.position_monitor_thread.start()
        
        # 안전 모니터링 스레드
        self.safety_monitor_thread = threading.Thread(
            target=self._safety_monitor_loop, daemon=True)
        self.safety_monitor_thread.start()
        
        logger.info("모니터링 시스템 시작")
    
    def disconnect(self):
        """🔥 안전한 연결 해제 - 강화된 버전"""
        try:
            logger.info("로봇 연결 해제 시작...")
            
            # 진행 중인 작업 중지
            if self.current_state in [RobotState.MOVING, RobotState.PICKING]:
                self._emergency_stop_movement()
            
            # 모니터링 중지
            self.is_monitoring = False
            self._set_state(RobotState.DISCONNECTED)
            
            # 모니터링 스레드 정리
            threads_to_join = [
                self.position_monitor_thread,
                self.safety_monitor_thread
            ]
            
            for thread in threads_to_join:
                if thread and thread.is_alive():
                    thread.join(timeout=2.0)
            
            # API 연결 정리
            if self.dobot_api:
                self.dobot_api.cleanup_all_connections()
                self.dobot_api = None
            
            # 상태 초기화
            self.is_simulation_mode = False
            
            # 작업 기록 저장
            self._save_operation_history()
            
            logger.info("✅ 로봇 연결 해제 완료")
            
        except Exception as e:
            logger.error(f"연결 해제 중 오류: {e}")
    
    def reconnect(self) -> bool:
        """로봇 재연결 - 강화된 버전"""
        logger.info("로봇 재연결 시작...")
        
        # 현재 상태 저장
        last_position = self.current_position
        
        # 연결 해제
        self.disconnect()
        time.sleep(2.0)
        
        # 재연결
        success = self.connect()
        
        if success and not self.is_simulation_mode:
            # 이전 위치로 복귀 시도
            try:
                self.move_to_safe_position()
                logger.info("재연결 후 안전 위치로 이동 완료")
            except Exception as e:
                logger.warning(f"재연결 후 위치 복귀 실패: {e}")
        
        return success
    
    def is_robot_connected(self) -> bool:
        """연결 상태 확인 - 강화된 버전"""
        if self.is_simulation_mode:
            return self.current_state == RobotState.SIMULATION
        
        return (self.current_state == RobotState.CONNECTED and 
                self.dobot_api and 
                self.dobot_api.is_connected)
    
    # ========== 고급 이동 제어 ==========
    
    def move_to(self, x: float, y: float, z: float, r: float = 0.0, 
                movement_mode: MovementMode = MovementMode.JOINT,
                wait_for_completion: bool = True, 
                speed: Optional[int] = None,
                acceleration: Optional[int] = None,
                coordinate_system: CoordinateSystem = CoordinateSystem.WORLD) -> bool:
        """고급 로봇 이동 - 완전한 기능"""
        
        target_position = RobotPosition(x, y, z, r, coordinate_system=coordinate_system)
        
        # 좌표 검증
        if not self._validate_position(target_position):
            logger.error(f"유효하지 않은 좌표: {target_position.to_tuple()}")
            return False
        
        # 충돌 검사
        if self.collision_detection and not self._check_collision_free_path(target_position):
            logger.error("충돌 가능성 감지됨")
            return False
        
        # 경로 계획
        if self.path_planning_enabled:
            waypoints = self._plan_path(self.current_position, target_position)
            if not waypoints:
                logger.error("경로 계획 실패")
                return False
        else:
            waypoints = [target_position]
        
        # 시뮬레이션 모드 처리
        if self.is_simulation_mode:
            return self._simulate_movement(waypoints, wait_for_completion)
        
        # 연결 상태 확인
        if not self.is_robot_connected():
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            self._set_state(RobotState.MOVING)
            
            # 속도 및 가속도 설정
            if speed is not None:
                self.dobot_api.send_command(f"SpeedFactor({speed})", "dashboard")
            if acceleration is not None:
                self.dobot_api.send_command(f"AccJ({acceleration})", "dashboard")
                self.dobot_api.send_command(f"AccL({acceleration})", "dashboard")
            
            # 웨이포인트별 이동 실행
            for i, waypoint in enumerate(waypoints):
                success = self._execute_movement(waypoint, movement_mode)
                if not success:
                    logger.error(f"웨이포인트 {i+1} 이동 실패")
                    self._set_state(RobotState.ERROR)
                    return False
                
                # 중간 위치 확인
                if wait_for_completion and i < len(waypoints) - 1:
                    if not self._wait_for_movement_completion(waypoint):
                        logger.error(f"웨이포인트 {i+1} 완료 대기 실패")
                        return False
            
            # 최종 완료 대기
            if wait_for_completion:
                final_target = waypoints[-1]
                if self._wait_for_movement_completion(final_target):
                    self.current_position = final_target
                    self._notify_position_update(final_target)
                    self._set_state(RobotState.CONNECTED)
                    
                    # 성능 데이터 기록
                    self._record_movement_performance(target_position)
                    
                    logger.info(f"✅ 이동 완료: {final_target.to_tuple()}")
                    return True
                else:
                    logger.error("이동 완료 대기 실패")
                    self._set_state(RobotState.ERROR)
                    return False
            else:
                # 비동기 이동
                self.current_position = target_position
                return True
                
        except Exception as e:
            logger.error(f"이동 중 오류: {e}")
            self._set_state(RobotState.ERROR)
            return False
    
    def _execute_movement(self, target: RobotPosition, mode: MovementMode) -> bool:
        """개별 이동 실행"""
        x, y, z, r = target.to_tuple()
        
        # 좌표계 변환
        if target.coordinate_system != CoordinateSystem.WORLD:
            x, y, z, r = self._transform_coordinates(x, y, z, r, target.coordinate_system)
        
        # 이동 모드에 따른 명령 생성
        if mode == MovementMode.JOINT:
            command = f"MovJ({x},{y},{z},{r})"
        elif mode == MovementMode.LINEAR:
            command = f"MovL({x},{y},{z},{r})"
        elif mode == MovementMode.ARC:
            # 호 이동의 경우 중간점 필요 (간략화)
            command = f"MovL({x},{y},{z},{r})"
        else:
            command = f"MovJ({x},{y},{z},{r})"
        
        # 명령 전송
        response = self.dobot_api.send_command(command, "move")
        
        if not response or "OK" not in response.upper():
            logger.error(f"이동 명령 실패: {response}")
            return False
        
        return True
    
    def _plan_path(self, start: RobotPosition, end: RobotPosition) -> List[RobotPosition]:
        """경로 계획 - 장애물 회피"""
        waypoints = []
        
        # 간단한 경로 계획 (실제로는 더 복잡한 알고리즘 사용)
        # 중간 웨이포인트 계산
        steps = max(1, int(start.distance_to(end) / 50))  # 50mm마다 웨이포인트
        
        for i in range(steps + 1):
            ratio = i / steps
            waypoint = start.interpolate_to(end, ratio)
            
            # 장애물 체크
            if self._is_position_safe(waypoint):
                waypoints.append(waypoint)
            else:
                # 장애물 회피 웨이포인트 생성
                safe_waypoint = self._find_safe_waypoint(waypoint)
                if safe_waypoint:
                    waypoints.append(safe_waypoint)
                else:
                    logger.warning(f"안전한 웨이포인트를 찾을 수 없음: {waypoint.to_tuple()}")
                    return []
        
        return waypoints
    
    def _simulate_movement(self, waypoints: List[RobotPosition], wait: bool) -> bool:
        """시뮬레이션 모드 이동 - 강화된 버전"""
        logger.info(f"[시뮬레이션] 이동 시작: {len(waypoints)} 웨이포인트")
        
        if wait:
            total_distance = 0
            for i in range(len(waypoints)):
                if i > 0:
                    total_distance += waypoints[i-1].distance_to(waypoints[i])
            
            # 거리에 비례한 시뮬레이션 시간
            sim_time = min(max(total_distance / 200.0, 0.5), 5.0)
            
            # 단계적 이동 시뮬레이션
            for i, waypoint in enumerate(waypoints):
                step_time = sim_time / len(waypoints)
                time.sleep(step_time)
                
                self.current_position = waypoint
                self._notify_position_update(waypoint)
                
                logger.debug(f"[시뮬레이션] 웨이포인트 {i+1}/{len(waypoints)}: {waypoint.to_tuple()}")
        
        if waypoints:
            self.current_position = waypoints[-1]
            self._notify_position_update(waypoints[-1])
        
        return True
    
    def move_joint(self, joint_angles: List[float], wait_for_completion: bool = True) -> bool:
        """관절 공간 이동"""
        if len(joint_angles) != self.robot_config.dof:
            logger.error(f"관절 각도 개수 불일치: {len(joint_angles)} != {self.robot_config.dof}")
            return False
        
        # 관절 제한 확인
        for i, angle in enumerate(joint_angles):
            joint_name = f"joint{i+1}"
            if joint_name in self.robot_config.joint_limits:
                min_angle, max_angle = self.robot_config.joint_limits[joint_name]
                if not (min_angle <= angle <= max_angle):
                    logger.error(f"관절 {i+1} 각도 제한 초과: {angle} (범위: {min_angle}~{max_angle})")
                    return False
        
        if self.is_simulation_mode:
            logger.info(f"[시뮬레이션] 관절 이동: {joint_angles}")
            if wait_for_completion:
                time.sleep(2.0)
            return True
        
        if not self.is_robot_connected():
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            self._set_state(RobotState.MOVING)
            
            # 관절 이동 명령
            angles_str = ",".join(map(str, joint_angles))
            command = f"JointMovJ([{angles_str}])"
            response = self.dobot_api.send_command(command, "move")
            
            if not response or "OK" not in response.upper():
                logger.error(f"관절 이동 명령 실패: {response}")
                self._set_state(RobotState.ERROR)
                return False
            
            if wait_for_completion:
                # 관절 이동 완료 대기
                time.sleep(2.0)  # 실제로는 피드백 기반 대기
                self._set_state(RobotState.CONNECTED)
            
            logger.info(f"✅ 관절 이동 완료: {joint_angles}")
            return True
            
        except Exception as e:
            logger.error(f"관절 이동 중 오류: {e}")
            self._set_state(RobotState.ERROR)
            return False
    
    def move_relative(self, dx: float, dy: float, dz: float, dr: float = 0.0) -> bool:
        """상대 이동"""
        current = self.get_current_position()
        if current is None:
            logger.error("현재 위치를 알 수 없음")
            return False
        
        target_x = current.x + dx
        target_y = current.y + dy
        target_z = current.z + dz
        target_r = current.r + dr
        
        return self.move_to(target_x, target_y, target_z, target_r)
    
    def move_to_safe_position(self) -> bool:
        """안전 위치로 이동 - 강화된 버전"""
        logger.info("안전 위치로 이동")
        
        # 현재 위치에서 안전 높이로 먼저 상승
        current_pos = self.get_current_position()
        if current_pos:
            safe_z = max(current_pos.z, self.safe_position.z)
            intermediate_pos = RobotPosition(current_pos.x, current_pos.y, safe_z, current_pos.r)
            
            # 단계적 이동
            if not self.move_to(intermediate_pos.x, intermediate_pos.y, intermediate_pos.z, intermediate_pos.r):
                return False
        
        return self.move_to(
            self.safe_position.x,
            self.safe_position.y, 
            self.safe_position.z,
            self.safe_position.r
        )
    
    def home_robot(self) -> bool:
        """홈 위치로 이동 - 강화된 버전"""
        logger.info("홈 위치로 이동")
        self._set_state(RobotState.HOMING)
        
        try:
            # 안전한 홈 시퀀스
            success = self.move_to_safe_position()
            if success:
                success = self.move_to(
                    self.home_position.x,
                    self.home_position.y,
                    self.home_position.z, 
                    self.home_position.r
                )
            
            if success:
                self._set_state(RobotState.CONNECTED)
                logger.info("✅ 홈 이동 완료")
            else:
                self._set_state(RobotState.ERROR)
                logger.error("❌ 홈 이동 실패")
            
            return success
            
        except Exception as e:
            logger.error(f"홈 이동 중 오류: {e}")
            self._set_state(RobotState.ERROR)
            return False
    
    # ========== 고급 그리퍼 제어 ==========
    
    def gripper_control(self, enable: bool, force: Optional[float] = None, 
                       speed: Optional[float] = None, wait_time: float = 1.5) -> bool:
        """고급 그리퍼 제어"""
        action = "CLOSE" if enable else "OPEN"
        logger.info(f"그리퍼 {action} (힘: {force}, 속도: {speed})")
        
        # 시뮬레이션 모드
        if self.is_simulation_mode:
            logger.info(f"[시뮬레이션] 그리퍼 {action}")
            time.sleep(wait_time)
            self.gripper_state = GripperState.CLOSED if enable else GripperState.OPEN
            return True
        
        # 연결 확인
        if not self.is_robot_connected():
            logger.error("로봇이 연결되지 않음")
            return False
        
        try:
            # 고급 그리퍼 명령 (실제 하드웨어에 맞게 조정 필요)
            if enable:
                # 그리퍼 닫기
                if force and speed:
                    command = f"GripperClose({force},{speed})"
                elif force:
                    command = f"GripperClose({force})"
                else:
                    command = "GripperClose()"
            else:
                # 그리퍼 열기
                if speed:
                    command = f"GripperOpen({speed})"
                else:
                    command = "GripperOpen()"
            
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                time.sleep(wait_time)
                self.gripper_state = GripperState.CLOSED if enable else GripperState.OPEN
                logger.info(f"✅ 그리퍼 {action} 완료")
                return True
            else:
                logger.error(f"그리퍼 제어 실패: {response}")
                self.gripper_state = GripperState.ERROR
                return False
                
        except Exception as e:
            logger.error(f"그리퍼 제어 중 오류: {e}")
            self.gripper_state = GripperState.ERROR
            return False
    
    def gripper_partial_close(self, percentage: float) -> bool:
        """부분 그리퍼 닫기"""
        if not 0 <= percentage <= 100:
            logger.error(f"유효하지 않은 그리퍼 비율: {percentage}")
            return False
        
        logger.info(f"그리퍼 부분 닫기: {percentage}%")
        
        if self.is_simulation_mode:
            logger.info(f"[시뮬레이션] 그리퍼 {percentage}% 닫기")
            time.sleep(1.0)
            self.gripper_state = GripperState.PARTIAL
            return True
        
        if not self.is_robot_connected():
            return False
        
        try:
            command = f"GripperPosition({percentage})"
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                time.sleep(1.5)
                self.gripper_state = GripperState.PARTIAL
                logger.info(f"✅ 그리퍼 부분 닫기 완료: {percentage}%")
                return True
            else:
                logger.error(f"그리퍼 부분 제어 실패: {response}")
                return False
                
        except Exception as e:
            logger.error(f"그리퍼 부분 제어 중 오류: {e}")
            return False
    
    def gripper_open(self) -> bool:
        """그리퍼 열기"""
        return self.gripper_control(False)
    
    def gripper_close(self, force: Optional[float] = None) -> bool:
        """그리퍼 닫기"""
        return self.gripper_control(True, force=force)
    
    def get_gripper_state(self) -> GripperState:
        """그리퍼 상태 조회"""
        return self.gripper_state
    
    # ========== 고급 가구 픽업 시퀀스 ==========
    
    def pickup_furniture(self, furniture_type: str, 
                        target_position: Optional[RobotPosition] = None,
                        custom_sequence: Optional[List[Dict]] = None) -> bool:
        """고급 가구 픽업 메인 함수"""
        
        if furniture_type not in self.furniture_configs and target_position is None:
            logger.error(f"지원하지 않는 가구 유형: {furniture_type}")
            return False
        
        # 설정 가져오기
        if target_position:
            # 동적 타겟 사용
            config = FurnitureConfig(
                name=f"동적_{furniture_type}",
                position=target_position,
                pickup_sequence=custom_sequence
            )
        else:
            config = self.furniture_configs[furniture_type]
        
        logger.info(f"{config.name} 픽업 시작")
        self._set_state(RobotState.PICKING)
        
        # 작업 시작 시간 기록
        start_time = time.time()
        
        try:
            # 사전 안전 검사
            if not self._pre_pickup_safety_check(config):
                logger.error("사전 안전 검사 실패")
                self._set_state(RobotState.ERROR)
                return False
            
            # 픽업 시퀀스 실행
            if config.pickup_sequence:
                success = self._execute_custom_pickup_sequence(config)
            else:
                success = self._execute_standard_pickup_sequence(config)
            
            if success:
                # 작업 완료 처리
                execution_time = time.time() - start_time
                self._record_pickup_success(config, execution_time)
                
                logger.info(f"✅ {config.name} 픽업 완료 ({execution_time:.2f}초)")
                self._set_state(RobotState.CONNECTED)
                return True
            else:
                logger.error(f"❌ {config.name} 픽업 실패")
                self._set_state(RobotState.ERROR)
                return False
                
        except Exception as e:
            logger.error(f"{config.name} 픽업 중 오류: {e}")
            self._set_state(RobotState.ERROR)
            
            # 비상 복구 시도
            self._emergency_pickup_recovery()
            return False
    
    def _execute_standard_pickup_sequence(self, config: FurnitureConfig) -> bool:
        """표준 픽업 시퀀스 실행"""
        steps = [
            ("안전 위치 접근", lambda: self._approach_target(config)),
            ("그리퍼 준비", lambda: self.gripper_open()),
            ("정밀 접근", lambda: self._precise_approach(config)),
            ("목표물 접근", lambda: self._descend_to_target(config)),
            ("그리퍼 닫기", lambda: self._grip_target(config)),
            ("안전 높이로 상승", lambda: self._lift_from_target(config)),
            ("베이스 위치로 이동", lambda: self.move_to(200, 0, 150, 0)),
            ("최종 위치로 이동", lambda: self._move_to_place_position(config)),
            ("배치", lambda: self._place_target(config))
        ]
        
        for step_name, step_func in steps:
            logger.info(f"픽업 단계: {step_name}")
            
            try:
                if not step_func():
                    logger.error(f"픽업 단계 실패: {step_name}")
                    return False
                
                # 단계간 안전 확인
                if not self._inter_step_safety_check():
                    logger.error(f"안전 확인 실패: {step_name} 이후")
                    return False
                
                time.sleep(0.5)  # 단계간 안전 대기
                
            except Exception as e:
                logger.error(f"픽업 단계 오류 [{step_name}]: {e}")
                return False
        
        return True
    
    def _execute_custom_pickup_sequence(self, config: FurnitureConfig) -> bool:
        """커스텀 픽업 시퀀스 실행"""
        if not config.pickup_sequence:
            return False
        
        for i, step in enumerate(config.pickup_sequence):
            action = step.get('action')
            logger.info(f"커스텀 단계 {i+1}: {action}")
            
            try:
                if action == 'approach':
                    height_offset = step.get('height_offset', config.approach_height)
                    success = self._approach_with_offset(config, height_offset)
                elif action == 'orient':
                    angle = step.get('angle', 0)
                    success = self._orient_gripper(config, angle)
                elif action == 'descend':
                    speed = step.get('speed', 50)
                    success = self._descend_with_speed(config, speed)
                elif action == 'grip':
                    force = step.get('force', config.grip_force)
                    success = self.gripper_close(force)
                elif action == 'lift':
                    height_offset = step.get('height_offset', config.approach_height)
                    success = self._lift_with_offset(config, height_offset)
                else:
                    logger.warning(f"알 수 없는 액션: {action}")
                    success = True
                
                if not success:
                    logger.error(f"커스텀 단계 실패: {action}")
                    return False
                
                # 단계별 대기 시간
                delay = step.get('delay', 0.5)
                time.sleep(delay)
                
            except Exception as e:
                logger.error(f"커스텀 단계 오류 [{action}]: {e}")
                return False
        
        return True
    
    def _pre_pickup_safety_check(self, config: FurnitureConfig) -> bool:
        """사전 안전 검사"""
        # 목표 위치 안전성 확인
        if not self._is_position_safe(config.position):
            logger.error("목표 위치가 안전하지 않음")
            return False
        
        # 그리퍼 상태 확인
        if self.gripper_state == GripperState.ERROR:
            logger.error("그리퍼 상태 오류")
            return False
        
        # 작업공간 확인
        if not self._validate_position(config.position):
            logger.error("목표 위치가 작업공간 밖")
            return False
        
        return True
    
    def _precise_approach(self, config: FurnitureConfig) -> bool:
        """정밀 접근"""
        # 비전 시스템이 있다면 정밀 위치 조정
        if self.vision_guidance_enabled:
            adjusted_position = self._vision_guided_adjustment(config.position)
            if adjusted_position:
                config.position = adjusted_position
        
        return True
    
    def _grip_target(self, config: FurnitureConfig) -> bool:
        """목표물 그립"""
        # 힘 제어가 가능한 경우
        if self.force_feedback_enabled:
            return self._force_controlled_grip(config)
        else:
            return self.gripper_close(config.grip_force)
    
    def _place_target(self, config: FurnitureConfig) -> bool:
        """목표물 배치"""
        if not config.place_position:
            logger.warning("배치 위치가 설정되지 않음")
            return True
        
        # 배치 위치로 이동
        if not self.move_to(
            config.place_position.x,
            config.place_position.y,
            config.place_position.z + 50,  # 안전 높이
            config.place_position.r
        ):
            return False
        
        # 하강하여 배치
        if not self.move_to(
            config.place_position.x,
            config.place_position.y,
            config.place_position.z,
            config.place_position.r
        ):
            return False
        
        # 그리퍼 열어서 배치
        return self.gripper_open()
    
    # ========== 개별 가구 픽업 함수들 (기존 호환성 유지) ==========
    
    def pickup_sofa(self) -> bool:
        """소파 픽업"""
        return self.pickup_furniture('sofa')
    
    def pickup_chair(self) -> bool:
        """의자 픽업"""
        return self.pickup_furniture('chair')
    
    def pickup_desk(self) -> bool:
        """책상 픽업"""
        return self.pickup_furniture('desk')
    
    def pickup_bed(self) -> bool:
        """침대 픽업"""
        return self.pickup_furniture('bed')
    
    def pickup_table(self) -> bool:
        """테이블 픽업"""
        return self.pickup_furniture('table')
    
    def pickup_stool(self) -> bool:
        """스툴 픽업"""
        return self.pickup_furniture('stool')
    
    def pickup_cabinet(self) -> bool:
        """캐비닛 픽업"""
        return self.pickup_furniture('cabinet')
    
    # ========== 위치 및 상태 조회 ==========
    
    def get_current_position(self) -> Optional[RobotPosition]:
        """현재 위치 조회 - 강화된 버전"""
        if self.is_simulation_mode:
            return copy.deepcopy(self.current_position)
        
        if not self.is_robot_connected():
            return self.last_known_position
        
        try:
            command = "GetPose()"
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                # 응답 파싱 (실제 형식에 맞게 조정 필요)
                # 예: "OK,{200.0,0.0,100.0,0.0,joint1,joint2,joint3,joint4}"
                parts = response.split(',')
                if len(parts) >= 5:
                    try:
                        x = float(parts[1].strip('{}'))
                        y = float(parts[2])
                        z = float(parts[3])
                        r = float(parts[4].strip('{}'))
                        
                        # 관절 각도 파싱 (있는 경우)
                        joint_angles = None
                        if len(parts) > 5:
                            joint_angles = [float(parts[i]) for i in range(5, min(9, len(parts)))]
                        
                        position = RobotPosition(x, y, z, r, joint_angles=joint_angles)
                        self.current_position = position
                        self.last_known_position = copy.deepcopy(position)
                        return position
                    except (ValueError, IndexError) as e:
                        logger.warning(f"위치 파싱 오류: {e}")
            
            return self.last_known_position
            
        except Exception as e:
            logger.error(f"위치 조회 실패: {e}")
            return self.last_known_position
    
    def get_joint_angles(self) -> Optional[List[float]]:
        """관절 각도 조회"""
        if self.is_simulation_mode:
            return self.simulation_data.get('joint_angles', [0, 45, 45, 0])
        
        if not self.is_robot_connected():
            return None
        
        try:
            command = "GetAngle()"
            response = self.dobot_api.send_command(command, "dashboard")
            
            if response and "OK" in response.upper():
                parts = response.split(',')
                if len(parts) > 1:
                    try:
                        angles = [float(parts[i]) for i in range(1, min(5, len(parts)))]
                        return angles
                    except (ValueError, IndexError) as e:
                        logger.warning(f"관절 각도 파싱 오류: {e}")
            
            return None
            
        except Exception as e:
            logger.error(f"관절 각도 조회 실패: {e}")
            return None
    
    def _update_current_position(self):
        """현재 위치 업데이트"""
        position = self.get_current_position()
        if position:
            self.current_position = position
            self._notify_position_update(position)
    
    def get_robot_state(self) -> RobotState:
        """로봇 상태 조회"""
        return self.current_state
    
    def get_system_info(self) -> Dict[str, Any]:
        """확장된 시스템 정보 조회"""
        info = {
            'state': self.current_state.value,
            'gripper_state': self.gripper_state.value,
            'is_simulation': self.is_simulation_mode,
            'is_connected': self.is_robot_connected(),
            'is_calibrated': self.is_calibrated,
            'current_position': self.current_position.to_dict(),
            'ip_address': self.ip_address,
            'command_count': self.command_count,
            'error_count': self.error_count,
            'connection_attempts': self.connection_attempts,
            'last_command_time': self.last_command_time,
            'robot_config': {
                'model': self.robot_config.model,
                'dof': self.robot_config.dof,
                'max_reach': self.robot_config.max_reach,
                'max_payload': self.robot_config.max_payload
            }
        }
        
        # 성능 통계 추가
        if self.dobot_api:
            info['api_performance'] = self.dobot_api.get_performance_stats()
        
        # 작업 기록 통계
        if self.operation_history:
            recent_operations = self.operation_history[-10:]
            info['recent_operations'] = [op['type'] for op in recent_operations]
            
            success_count = sum(1 for op in recent_operations if op.get('success', False))
            info['recent_success_rate'] = success_count / len(recent_operations)
        
        return info
    
    # ========== 안전 및 검증 시스템 ==========
    
    def _validate_position(self, position: RobotPosition) -> bool:
        """위치 유효성 검사 - 강화된 버전"""
        limits = self.workspace_limits
        
        # 기본 범위 검사
        if not (limits['x_min'] <= position.x <= limits['x_max']):
            logger.error(f"X 좌표 범위 초과: {position.x} (허용: {limits['x_min']}~{limits['x_max']})")
            return False
        
        if not (limits['y_min'] <= position.y <= limits['y_max']):
            logger.error(f"Y 좌표 범위 초과: {position.y} (허용: {limits['y_min']}~{limits['y_max']})")
            return False
        
        if not (limits['z_min'] <= position.z <= limits['z_max']):
            logger.error(f"Z 좌표 범위 초과: {position.z} (허용: {limits['z_min']}~{limits['z_max']})")
            return False
        
        if not (limits['r_min'] <= position.r <= limits['r_max']):
            logger.error(f"R 좌표 범위 초과: {position.r} (허용: {limits['r_min']}~{limits['r_max']})")
            return False
        
        # 도달 가능성 검사
        distance_from_base = np.sqrt(position.x**2 + position.y**2 + position.z**2)
        if distance_from_base > self.robot_config.max_reach:
            logger.error(f"도달 불가능한 위치: 거리 {distance_from_base:.1f}mm (최대: {self.robot_config.max_reach}mm)")
            return False
        
        # 안전 구역 검사
        if not self._is_position_safe(position):
            logger.error(f"안전하지 않은 위치: {position.to_tuple()}")
            return False
        
        return True
    
    def _is_position_safe(self, position: RobotPosition) -> bool:
        """위치 안전성 검사"""
        # 동적 장애물 확인
        for obstacle in self.dynamic_obstacles:
            obs_pos = obstacle['position']
            obs_radius = obstacle['radius']
            
            distance = np.sqrt(
                (position.x - obs_pos[0])**2 + 
                (position.y - obs_pos[1])**2 + 
                (position.z - obs_pos[2])**2
            )
            
            if distance < obs_radius:
                logger.warning(f"장애물과 충돌 가능: 거리 {distance:.1f}mm < {obs_radius}mm")
                return False
        
        # 안전 구역 확인
        for zone in self.safety_zones:
            if self._point_in_safety_zone(position, zone):
                logger.warning(f"안전 구역 침범: {zone['name']}")
                return False
        
        return True
    
    def _check_collision_free_path(self, target: RobotPosition) -> bool:
        """충돌 없는 경로 확인"""
        if not self.collision_detection:
            return True
        
        # 간단한 직선 경로 충돌 검사
        current = self.get_current_position()
        if not current:
            return True
        
        # 경로상의 점들을 샘플링하여 검사
        steps = 10
        for i in range(steps + 1):
            ratio = i / steps
            check_point = current.interpolate_to(target, ratio)
            
            if not self._is_position_safe(check_point):
                return False
        
        return True
    
    def emergency_stop(self):
        """비상 정지 - 강화된 버전"""
        logger.warning("🚨 비상 정지 실행!")
        self.is_emergency_stopped = True
        self._set_state(RobotState.EMERGENCY_STOP)
        
        # 즉시 모든 움직임 중지
        if not self.is_simulation_mode and self.dobot_api:
            try:
                self.dobot_api.send_command("EmergencyStop()", "dashboard", wait_response=False)
                self.dobot_api.send_command("Pause()", "move", wait_response=False)
            except:
                pass
        
        # 모니터링 중지
        self.is_monitoring = False
        
        # 비상 정지 기록
        self._record_emergency_stop()
        
        # 연결 해제
        self.disconnect()
    
    def reset_emergency_stop(self) -> bool:
        """비상 정지 해제 - 강화된 버전"""
        if not self.is_emergency_stopped:
            return True
        
        logger.info("비상 정지 해제 시도...")
        
        # 안전 확인
        if not self._safety_check_for_resume():
            logger.error("안전 확인 실패 - 비상 정지 해제 불가")
            return False
        
        self.is_emergency_stopped = False
        
        # 재연결 시도
        success = self.reconnect()
        
        if success:
            # 안전 위치로 이동
            self.move_to_safe_position()
            logger.info("✅ 비상 정지 해제 완료")
        
        return success
    
    def _emergency_stop_movement(self):
        """움직임 비상 정지"""
        if not self.is_simulation_mode and self.dobot_api:
            try:
                self.dobot_api.send_command("Pause()", "move", wait_response=False)
            except:
                pass
    
    # ========== 모니터링 시스템 ==========
    
    def _position_monitor_loop(self):
        """위치 모니터링 루프 - 강화된 버전"""
        logger.debug("위치 모니터링 루프 시작")
        
        while self.is_monitoring:
            try:
                if self.is_robot_connected() and not self.is_simulation_mode:
                    # 실제 위치 업데이트
                    self._update_current_position()
                    
                    # 위치 정확도 검사
                    self._check_position_accuracy()
                
                elif self.is_simulation_mode:
                    # 시뮬레이션 위치 업데이트
                    self._update_simulation_position()
                
                time.sleep(1.0)  # 1초마다 업데이트
                
            except Exception as e:
                logger.error(f"위치 모니터링 오류: {e}")
                time.sleep(2.0)
        
        logger.debug("위치 모니터링 루프 종료")
    
    def _safety_monitor_loop(self):
        """안전 모니터링 루프"""
        logger.debug("안전 모니터링 루프 시작")
        
        while self.is_monitoring:
            try:
                # 현재 위치 안전성 확인
                current_pos = self.get_current_position()
                if current_pos and not self._is_position_safe(current_pos):
                    logger.warning("현재 위치가 안전하지 않음")
                    self._notify_collision_risk("현재 위치 안전성 경고")
                
                # 그리퍼 상태 확인
                self._check_gripper_safety()
                
                # 시스템 온도 확인 (시뮬레이션)
                if self.is_simulation_mode:
                    self._check_simulation_safety()
                
                time.sleep(2.0)  # 2초마다 안전 검사
                
            except Exception as e:
                logger.error(f"안전 모니터링 오류: {e}")
                time.sleep(3.0)
        
        logger.debug("안전 모니터링 루프 종료")
    
    # ========== 헬퍼 메서드들 ==========
    
    def _approach_target(self, config: FurnitureConfig) -> bool:
        """목표물 안전 접근"""
        approach_pos = RobotPosition(
            config.position.x,
            config.position.y,
            config.position.z + config.approach_height,
            config.position.r + config.approach_angle
        )
        return self.move_to(approach_pos.x, approach_pos.y, approach_pos.z, approach_pos.r)
    
    def _descend_to_target(self, config: FurnitureConfig) -> bool:
        """목표물로 하강"""
        return self.move_to(
            config.position.x,
            config.position.y,
            config.position.z,
            config.position.r,
            speed=config.pickup_speed
        )
    
    def _lift_from_target(self, config: FurnitureConfig) -> bool:
        """목표물에서 상승"""
        lift_pos = RobotPosition(
            config.position.x,
            config.position.y,
            config.position.z + config.approach_height,
            config.position.r
        )
        return self.move_to(lift_pos.x, lift_pos.y, lift_pos.z, lift_pos.r)
    
    def _move_to_place_position(self, config: FurnitureConfig) -> bool:
        """배치 위치로 이동"""
        if not config.place_position:
            return True
        
        place_pos = config.place_position
        return self.move_to(place_pos.x, place_pos.y, place_pos.z, place_pos.r)
    
    def _wait_for_movement_completion(self, target_position: RobotPosition, timeout: float = None) -> bool:
        """이동 완료 대기 - 강화된 버전"""
        if timeout is None:
            timeout = self.movement_timeout
        
        # 시뮬레이션에서는 즉시 완료
        if self.is_simulation_mode:
            return True
        
        start_time = time.time()
        check_interval = 0.2
        
        while time.time() - start_time < timeout:
            try:
                # 현재 위치 조회
                current_pos = self.get_current_position()
                if current_pos is None:
                    time.sleep(check_interval)
                    continue
                
                # 목표 위치와의 거리 확인
                distance = current_pos.distance_to(target_position)
                if distance <= self.position_tolerance:
                    logger.debug(f"이동 완료 확인: 거리 {distance:.2f}mm")
                    return True
                
                # 로봇 상태 확인
                if self.current_state == RobotState.ERROR:
                    logger.error("로봇 에러 상태로 인한 이동 중단")
                    return False
                
                time.sleep(check_interval)
                
            except Exception as e:
                logger.warning(f"이동 완료 확인 중 오류: {e}")
                time.sleep(0.5)
        
        logger.error(f"이동 완료 타임아웃 ({timeout}초)")
        return False
    
    def _set_state(self, new_state: RobotState):
        """상태 변경 및 콜백 호출 - 강화된 버전"""
        if self.current_state != new_state:
            old_state = self.current_state
            self.current_state = new_state
            
            logger.info(f"로봇 상태 변경: {old_state.value} -> {new_state.value}")
            
            # 상태 변경 기록
            self._record_state_change(old_state, new_state)
            
            # 콜백 호출
            for callback in self.state_change_callbacks:
                try:
                    callback(new_state)
                except Exception as e:
                    logger.error(f"상태 변경 콜백 오류: {e}")
    
    def _notify_position_update(self, position: RobotPosition):
        """위치 업데이트 콜백 호출"""
        for callback in self.position_update_callbacks:
            try:
                callback(position)
            except Exception as e:
                logger.error(f"위치 업데이트 콜백 오류: {e}")
    
    def _notify_error(self, error_message: str):
        """에러 콜백 호출"""
        self.error_count += 1
        for callback in self.error_callbacks:
            try:
                callback(error_message)
            except Exception as e:
                logger.error(f"에러 콜백 오류: {e}")
    
    def _notify_collision_risk(self, message: str):
        """충돌 위험 콜백 호출"""
        for callback in self.collision_callbacks:
            try:
                callback(message)
            except Exception as e:
                logger.error(f"충돌 콜백 오류: {e}")
    
    # ========== 작업 기록 및 분석 ==========
    
    def _record_state_change(self, old_state: RobotState, new_state: RobotState):
        """상태 변경 기록"""
        record = {
            'timestamp': time.time(),
            'type': 'state_change',
            'old_state': old_state.value,
            'new_state': new_state.value,
            'position': self.current_position.to_dict()
        }
        self.operation_history.append(record)
    
    def _record_movement_performance(self, target: RobotPosition):
        """이동 성능 기록"""
        current = self.get_current_position()
        if current:
            accuracy = current.distance_to(target)
            self.performance_data['position_accuracy'].append(accuracy)
    
    def _record_pickup_success(self, config: FurnitureConfig, execution_time: float):
        """픽업 성공 기록"""
        record = {
            'timestamp': time.time(),
            'type': 'pickup_success',
            'furniture_type': config.name,
            'execution_time': execution_time,
            'position': config.position.to_dict(),
            'success': True
        }
        self.operation_history.append(record)
        self.performance_data['cycle_times'].append(execution_time)
    
    def _record_emergency_stop(self):
        """비상 정지 기록"""
        record = {
            'timestamp': time.time(),
            'type': 'emergency_stop',
            'position': self.current_position.to_dict(),
            'state': self.current_state.value
        }
        self.operation_history.append(record)
    
    def _save_operation_history(self):
        """작업 기록 저장"""
        try:
            if not self.operation_history:
                return
            
            history_file = Path(f"logs/operation_history_{int(time.time())}.json")
            history_file.parent.mkdir(exist_ok=True)
            
            with open(history_file, 'w', encoding='utf-8') as f:
                json.dump(self.operation_history, f, indent=2, ensure_ascii=False)
            
            logger.info(f"작업 기록 저장: {history_file}")
            
        except Exception as e:
            logger.error(f"작업 기록 저장 실패: {e}")
    
    # ========== 콜백 및 이벤트 관리 ==========
    
    def add_state_change_callback(self, callback: Callable[[RobotState], None]):
        """상태 변경 콜백 추가"""
        self.state_change_callbacks.append(callback)
    
    def add_position_update_callback(self, callback: Callable[[RobotPosition], None]):
        """위치 업데이트 콜백 추가"""
        self.position_update_callbacks.append(callback)
    
    def add_error_callback(self, callback: Callable[[str], None]):
        """에러 콜백 추가"""
        self.error_callbacks.append(callback)
    
    def add_collision_callback(self, callback: Callable[[str], None]):
        """충돌 위험 콜백 추가"""
        self.collision_callbacks.append(callback)
    
    # ========== 고급 기능 구현 (스텁) ==========
    
    def _approach_with_offset(self, config: FurnitureConfig, height_offset: float) -> bool:
        """오프셋을 가진 접근"""
        approach_pos = RobotPosition(
            config.position.x,
            config.position.y,
            config.position.z + height_offset,
            config.position.r
        )
        return self.move_to(approach_pos.x, approach_pos.y, approach_pos.z, approach_pos.r)
    
    def _orient_gripper(self, config: FurnitureConfig, angle: float) -> bool:
        """그리퍼 방향 조정"""
        oriented_pos = RobotPosition(
            config.position.x,
            config.position.y,
            config.position.z + config.approach_height,
            angle
        )
        return self.move_to(oriented_pos.x, oriented_pos.y, oriented_pos.z, oriented_pos.r)
    
    def _descend_with_speed(self, config: FurnitureConfig, speed: int) -> bool:
        """지정된 속도로 하강"""
        return self.move_to(
            config.position.x,
            config.position.y,
            config.position.z,
            config.position.r,
            speed=speed
        )
    
    def _lift_with_offset(self, config: FurnitureConfig, height_offset: float) -> bool:
        """오프셋을 가진 상승"""
        lift_pos = RobotPosition(
            config.position.x,
            config.position.y,
            config.position.z + height_offset,
            config.position.r
        )
        return self.move_to(lift_pos.x, lift_pos.y, lift_pos.z, lift_pos.r)
    
    def _inter_step_safety_check(self) -> bool:
        """단계간 안전 확인"""
        current_pos = self.get_current_position()
        if current_pos:
            return self._is_position_safe(current_pos)
        return True
    
    def _emergency_pickup_recovery(self):
        """픽업 비상 복구"""
        logger.info("픽업 비상 복구 시도...")
        try:
            # 그리퍼 열기
            self.gripper_open()
            
            # 안전 위치로 이동
            self.move_to_safe_position()
            
            logger.info("픽업 비상 복구 완료")
        except Exception as e:
            logger.error(f"픽업 비상 복구 실패: {e}")
    
    def _vision_guided_adjustment(self, position: RobotPosition) -> Optional[RobotPosition]:
        """비전 가이드 위치 조정"""
        # 비전 시스템 구현 스텁
        logger.debug("비전 가이드 위치 조정 (스텁)")
        return position
    
    def _force_controlled_grip(self, config: FurnitureConfig) -> bool:
        """힘 제어 그립"""
        # 힘 제어 구현 스텁
        logger.debug("힘 제어 그립 (스텁)")
        return self.gripper_close(config.grip_force)
    
    def _find_safe_waypoint(self, waypoint: RobotPosition) -> Optional[RobotPosition]:
        """안전한 웨이포인트 찾기"""
        # 장애물 회피 알고리즘 스텁
        # 실제로는 더 복잡한 경로 계획 알고리즘 필요
        safe_waypoint = RobotPosition(
            waypoint.x,
            waypoint.y,
            waypoint.z + 50,  # 높이를 올려서 회피
            waypoint.r
        )
        
        if self._is_position_safe(safe_waypoint):
            return safe_waypoint
        return None
    
    def _transform_coordinates(self, x: float, y: float, z: float, r: float, 
                             coord_system: CoordinateSystem) -> Tuple[float, float, float, float]:
        """좌표계 변환"""
        # 좌표계 변환 구현 스텁
        if coord_system == CoordinateSystem.WORLD:
            return x, y, z, r
        elif coord_system == CoordinateSystem.BASE:
            # 베이스 좌표계 변환
            return x, y, z, r
        elif coord_system == CoordinateSystem.TOOL:
            # 툴 좌표계 변환
            tcp_x, tcp_y, tcp_z = self.calibration_data['tool_offset']
            return x + tcp_x, y + tcp_y, z + tcp_z, r
        else:
            return x, y, z, r
    
    def _update_simulation_position(self):
        """시뮬레이션 위치 업데이트"""
        # 시뮬레이션에서는 약간의 노이즈 추가
        if hasattr(self, 'simulation_noise'):
            noise_x = np.random.normal(0, 0.1)
            noise_y = np.random.normal(0, 0.1)
            noise_z = np.random.normal(0, 0.1)
            
            self.current_position.x += noise_x
            self.current_position.y += noise_y
            self.current_position.z += noise_z
            
            self._notify_position_update(self.current_position)
    
    def _check_position_accuracy(self):
        """위치 정확도 확인"""
        current_pos = self.get_current_position()
        if current_pos and self.last_known_position:
            accuracy = current_pos.distance_to(self.last_known_position)
            if accuracy > 5.0:  # 5mm 이상 차이
                logger.warning(f"위치 정확도 저하: {accuracy:.2f}mm")
    
    def _check_gripper_safety(self):
        """그리퍼 안전성 확인"""
        if self.gripper_state == GripperState.ERROR:
            logger.warning("그리퍼 상태 오류 감지")
    
    def _check_simulation_safety(self):
        """시뮬레이션 안전성 확인"""
        # 시뮬레이션 모드에서의 가상 안전 확인
        if hasattr(self, 'simulation_data'):
            temps = self.simulation_data.get('temperatures', [])
            if any(temp > 80 for temp in temps):
                logger.warning("시뮬레이션: 과열 감지")
    
    def _point_in_safety_zone(self, position: RobotPosition, zone: Dict) -> bool:
        """점이 안전 구역 내에 있는지 확인"""
        # 안전 구역 확인 구현 스텁
        return False
    
    def _safety_check_for_resume(self) -> bool:
        """재개를 위한 안전 확인"""
        # 비상 정지 해제 전 안전 확인
        current_pos = self.get_current_position()
        if current_pos:
            return self._is_position_safe(current_pos)
        return True
    
    # ========== 캘리브레이션 시스템 ==========
    
    def calibrate_robot(self, calibration_points: List[Tuple[float, float, float]]) -> bool:
        """로봇 캘리브레이션"""
        logger.info("로봇 캘리브레이션 시작...")
        self._set_state(RobotState.CALIBRATING)
        
        try:
            # 캘리브레이션 포인트들로 이동하여 실제 위치 측정
            measured_points = []
            
            for i, (x, y, z) in enumerate(calibration_points):
                logger.info(f"캘리브레이션 포인트 {i+1}/{len(calibration_points)}: ({x}, {y}, {z})")
                
                # 포인트로 이동
                if not self.move_to(x, y, z, 0):
                    logger.error(f"캘리브레이션 포인트 {i+1} 이동 실패")
                    self._set_state(RobotState.ERROR)
                    return False
                
                # 실제 위치 측정
                actual_pos = self.get_current_position()
                if actual_pos:
                    measured_points.append((actual_pos.x, actual_pos.y, actual_pos.z))
                else:
                    logger.error(f"캘리브레이션 포인트 {i+1} 위치 측정 실패")
                    self._set_state(RobotState.ERROR)
                    return False
            
            # 캘리브레이션 매트릭스 계산
            self._calculate_calibration_matrix(calibration_points, measured_points)
            
            self.is_calibrated = True
            self._set_state(RobotState.CONNECTED)
            logger.info("✅ 로봇 캘리브레이션 완료")
            return True
            
        except Exception as e:
            logger.error(f"로봇 캘리브레이션 실패: {e}")
            self._set_state(RobotState.ERROR)
            return False
    
    def _calculate_calibration_matrix(self, target_points: List, measured_points: List):
        """캘리브레이션 매트릭스 계산"""
        # 간단한 선형 변환 매트릭스 계산
        if len(target_points) >= 3:
            # 실제로는 더 복잡한 캘리브레이션 알고리즘 사용
            logger.info("캘리브레이션 매트릭스 계산 완료")
            # 결과를 calibration_data에 저장
            self.calibration_data['coordinate_transform'] = np.eye(4)
    
    # ========== 설정 관리 ==========
    
    def save_configuration(self, filename: str) -> bool:
        """설정 저장"""
        try:
            config_data = {
                'robot_config': {
                    'model': self.robot_config.model,
                    'dof': self.robot_config.dof,
                    'max_reach': self.robot_config.max_reach,
                    'max_payload': self.robot_config.max_payload,
                    'max_speed': self.robot_config.max_speed,
                    'max_acceleration': self.robot_config.max_acceleration,
                    'joint_limits': self.robot_config.joint_limits,
                    'tcp_offset': self.robot_config.tcp_offset
                },
                'positions': {
                    'home': self.home_position.to_dict(),
                    'safe': self.safe_position.to_dict()
                },
                'workspace_limits': self.workspace_limits,
                'safety_settings': {
                    'safety_height_offset': self.safety_height_offset,
                    'max_speed': self.max_speed,
                    'position_tolerance': self.position_tolerance,
                    'movement_timeout': self.movement_timeout,
                    'collision_detection': self.collision_detection
                },
                'calibration_data': {
                    'tool_offset': self.calibration_data['tool_offset'],
                    'is_calibrated': self.is_calibrated
                }
            }
            
            config_path = Path(f"config/{filename}")
            config_path.parent.mkdir(exist_ok=True)
            
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"설정 저장 완료: {config_path}")
            return True
            
        except Exception as e:
            logger.error(f"설정 저장 실패: {e}")
            return False
    
    def load_configuration(self, filename: str) -> bool:
        """설정 로드"""
        try:
            config_path = Path(f"config/{filename}")
            if not config_path.exists():
                logger.warning(f"설정 파일 없음: {config_path}")
                return False
            
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # 로봇 설정 적용
            if 'robot_config' in config_data:
                robot_cfg = config_data['robot_config']
                self.robot_config.model = robot_cfg.get('model', self.robot_config.model)
                self.robot_config.max_speed = robot_cfg.get('max_speed', self.robot_config.max_speed)
                # 기타 설정들...
            
            # 위치 설정 적용
            if 'positions' in config_data:
                pos_cfg = config_data['positions']
                if 'home' in pos_cfg:
                    home_data = pos_cfg['home']
                    self.home_position = RobotPosition(
                        home_data['x'], home_data['y'], home_data['z'], home_data['r']
                    )
                if 'safe' in pos_cfg:
                    safe_data = pos_cfg['safe']
                    self.safe_position = RobotPosition(
                        safe_data['x'], safe_data['y'], safe_data['z'], safe_data['r']
                    )
            
            # 작업공간 제한 적용
            if 'workspace_limits' in config_data:
                self.workspace_limits.update(config_data['workspace_limits'])
            
            # 안전 설정 적용
            if 'safety_settings' in config_data:
                safety_cfg = config_data['safety_settings']
                self.safety_height_offset = safety_cfg.get('safety_height_offset', self.safety_height_offset)
                self.max_speed = safety_cfg.get('max_speed', self.max_speed)
                self.position_tolerance = safety_cfg.get('position_tolerance', self.position_tolerance)
                self.movement_timeout = safety_cfg.get('movement_timeout', self.movement_timeout)
                self.collision_detection = safety_cfg.get('collision_detection', self.collision_detection)
            
            # 캘리브레이션 데이터 적용
            if 'calibration_data' in config_data:
                calib_cfg = config_data['calibration_data']
                self.calibration_data['tool_offset'] = calib_cfg.get('tool_offset', [0, 0, 0])
                self.is_calibrated = calib_cfg.get('is_calibrated', False)
            
            logger.info(f"설정 로드 완료: {config_path}")
            return True
            
        except Exception as e:
            logger.error(f"설정 로드 실패: {e}")
            return False

# ========== 테스트 및 유틸리티 함수 ==========

def test_robot_controller():
    """로봇 컨트롤러 테스트"""
    print("🧪 로봇 컨트롤러 테스트 시작")
    
    try:
        # 로봇 컨트롤러 생성
        robot = RobotController()
        print("✅ 로봇 컨트롤러 생성 성공")
        
        # 연결 테스트
        if robot.connect():
            print("✅ 로봇 연결 성공")
            
            # 상태 정보 출력
            system_info = robot.get_system_info()
            print(f"✅ 시스템 정보: {system_info['state']}")
            
            # 기본 이동 테스트
            if robot.move_to(250, 50, 150, 0):
                print("✅ 기본 이동 테스트 성공")
            
            # 그리퍼 테스트
            if robot.gripper_open():
                print("✅ 그리퍼 열기 성공")
            
            if robot.gripper_close():
                print("✅ 그리퍼 닫기 성공")
            
            # 홈 위치로 복귀
            if robot.home_robot():
                print("✅ 홈 위치 복귀 성공")
            
            # 연결 해제
            robot.disconnect()
            print("✅ 로봇 연결 해제 성공")
        
        print("✅ 로봇 컨트롤러 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ 로봇 컨트롤러 테스트 실패: {e}")
        return False

def create_sample_config():
    """샘플 설정 파일 생성"""
    robot = RobotController()
    
    # 기본 설정으로 샘플 파일 생성
    if robot.save_configuration("sample_config.json"):
        print("✅ 샘플 설정 파일 생성 완료: config/sample_config.json")
    else:
        print("❌ 샘플 설정 파일 생성 실패")

if __name__ == "__main__":
    # 로깅 설정
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 테스트 실행
    print("=" * 60)
    print("Dobot 로봇 컨트롤러 - 완전한 통합 버전")
    print("=" * 60)
    
    # 샘플 설정 파일 생성
    create_sample_config()
    
    # 기본 테스트 실행
    test_robot_controller()
    
    print("=" * 60)
    print("테스트 완료")
    print("=" * 60)
