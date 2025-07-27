#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot 로봇팔 제어 시스템 설정
모든 상수 및 설정값을 중앙 관리
"""

import os
from typing import Dict, List, Tuple


class AppConfig:
    """애플리케이션 설정 클래스"""
    
    # === 로봇 연결 설정 ===
    ROBOT_IP = "192.168.1.6"
    DASHBOARD_PORT = 29999
    MOVE_PORT = 30003
    FEED_PORT = 30004
    
    # === 연결 타임아웃 설정 ===
    CONNECTION_TIMEOUT = 5.0  # 초
    COMMAND_TIMEOUT = 10.0   # 초
    MOVEMENT_TIMEOUT = 30.0  # 초
    
    # === 로봇 작업 공간 제한 (mm) ===
    X_MIN = -400
    X_MAX = 400
    Y_MIN = -400
    Y_MAX = 400
    Z_MIN = -100
    Z_MAX = 200
    
    # === 기본 위치 설정 ===
    HOME_POSITION = [250.0, 0.0, 50.0, 0.0]  # [X, Y, Z, R]
    SAFE_POSITION = [250.0, 0.0, 100.0, 0.0]
    
    # === 움직임 설정 ===
    MOVEMENT_SPEED = 100.0      # mm/s
    JOINT_SPEED = 50.0          # degree/s
    ACCELERATION = 100.0        # mm/s²
    POSITION_TOLERANCE = 1.0    # mm
    SAFETY_HEIGHT_OFFSET = 50.0 # mm
    
    # === 그리퍼 설정 ===
    GRIPPER_OPEN_DELAY = 0.5    # 초
    GRIPPER_CLOSE_DELAY = 1.0   # 초
    GRIPPER_FORCE = 50.0        # %
    
    # === 가구 픽업 좌표 설정 ===
    FURNITURE_POSITIONS = {
        "sofa": {
            "pickup": [300.0, 100.0, 20.0, 0.0],
            "place": [350.0, 0.0, 30.0, 0.0]
        },
        "chair": {
            "pickup": [250.0, 150.0, 25.0, 45.0],
            "place": [300.0, -50.0, 35.0, 45.0]
        },
        "desk": {
            "pickup": [200.0, -100.0, 30.0, 90.0],
            "place": [350.0, 50.0, 40.0, 90.0]
        },
        "bed": {
            "pickup": [350.0, -150.0, 15.0, 0.0],
            "place": [300.0, 100.0, 25.0, 0.0]
        },
        "detected": {  # 감지된 객체 기본 배치 위치
            "place": [350.0, 0.0, 50.0, 0.0]
        }
    }
    
    # === YOLOv8 설정 ===
    YOLO_MODEL_PATH = "yolov8n.pt"  # YOLOv8 모델 파일 경로
    CAMERA_INDEX = 0  # 카메라 인덱스 (0=기본 카메라)
    
    # YOLOv8 감지 설정
    YOLO_CONFIDENCE_THRESHOLD = 0.5  # 감지 신뢰도 임계값
    YOLO_IOU_THRESHOLD = 0.45  # IoU 임계값
    YOLO_MAX_DETECTIONS = 10  # 최대 감지 객체 수
    
    # 카메라 설정
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 30
    
    # 카메라-로봇 좌표 변환 설정
    CAMERA_ROBOT_CALIBRATION = {
        'offset_x': 0.0,  # X축 오프셋 (mm)
        'offset_y': 0.0,  # Y축 오프셋 (mm)
        'scale_x': 1.0,   # X축 스케일
        'scale_y': 1.0,   # Y축 스케일
        'rotation': 0.0   # 회전각 (도)
    }
    
    # === 시퀀스 설정 ===
    PICKUP_SEQUENCE_DELAYS = {
        "before_move": 0.5,      # 이동 전 대기
        "at_target": 1.0,        # 타겟 도달 후 대기
        "gripper_operation": 1.5, # 그리퍼 동작 후 대기
        "after_pickup": 1.0,     # 픽업 후 대기
        "before_place": 0.5,     # 배치 전 대기
        "after_place": 1.0       # 배치 후 대기
    }
    
    # === 재시도 설정 ===
    MAX_RETRY_ATTEMPTS = 3
    RETRY_DELAY = 2.0  # 초
    
    # === 로깅 설정 ===
    LOG_LEVEL = "INFO"
    LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    LOG_DIR = "logs"
    LOG_FILE = "dobot_system.log"
    MAX_LOG_SIZE = 10 * 1024 * 1024  # 10MB
    LOG_BACKUP_COUNT = 5
    
    # === GUI 설정 ===
    WINDOW_TITLE = "Dobot 로봇팔 제어 시스템"
    WINDOW_SIZE = "1200x800"
    WINDOW_MIN_SIZE = (800, 600)
    
    UI_THEME = "clam"  # tkinter 테마
    ACCENT_COLOR = "#007ACC"
    ERROR_COLOR = "#FF4444"
    SUCCESS_COLOR = "#28A745"
    WARNING_COLOR = "#FFC107"
    
    # === 폰트 설정 ===
    DEFAULT_FONT = ("Arial", 10)
    HEADER_FONT = ("Arial", 12, "bold")
    BUTTON_FONT = ("Arial", 10, "bold")
    LOG_FONT = ("Consolas", 9)
    
    # === 파일 경로 ===
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(BASE_DIR, "data")
    CONFIG_DIR = os.path.join(BASE_DIR, "config")
    
    # === 시뮬레이션 설정 ===
    SIMULATION_MODE = True
    SIMULATION_MOVE_DELAY = 1.0  # 초
    SIMULATION_GRIPPER_DELAY = 0.5  # 초
    
    # === 에러 핸들링 설정 ===
    ERROR_RECOVERY_ENABLED = True
    AUTO_RECONNECT_ENABLED = True
    AUTO_RECONNECT_INTERVAL = 5.0  # 초
    
    # === 성능 설정 ===
    STATUS_UPDATE_INTERVAL = 1.0  # 초
    POSITION_UPDATE_INTERVAL = 0.5  # 초
    MAX_LOG_ENTRIES_GUI = 1000  # GUI 로그 최대 라인 수
    
    @classmethod
    def get_log_file_path(cls) -> str:
        """로그 파일 전체 경로 반환"""
        os.makedirs(cls.LOG_DIR, exist_ok=True)
        return os.path.join(cls.LOG_DIR, cls.LOG_FILE)
    
    @classmethod
    def get_data_file_path(cls, filename: str) -> str:
        """데이터 파일 전체 경로 반환"""
        os.makedirs(cls.DATA_DIR, exist_ok=True)
        return os.path.join(cls.DATA_DIR, filename)
    
    @classmethod
    def validate_coordinates(cls, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """좌표 유효성 검증"""
        return (cls.X_MIN <= x <= cls.X_MAX and
                cls.Y_MIN <= y <= cls.Y_MAX and
                cls.Z_MIN <= z <= cls.Z_MAX and
                -180 <= r <= 180)
    
    @classmethod
    def get_safe_position(cls, x: float, y: float, z: float, r: float = 0.0) -> List[float]:
        """안전 위치 계산 (Z축에 오프셋 추가)"""
        return [x, y, z + cls.SAFETY_HEIGHT_OFFSET, r]
    
    @classmethod
    def clamp_coordinates(cls, x: float, y: float, z: float, r: float = 0.0) -> Tuple[float, float, float, float]:
        """좌표를 작업 영역 내로 제한"""
        x = max(cls.X_MIN, min(cls.X_MAX, x))
        y = max(cls.Y_MIN, min(cls.Y_MAX, y))
        z = max(cls.Z_MIN, min(cls.Z_MAX, z))
        r = max(-180, min(180, r))
        return x, y, z, r


class DobotCommands:
    """Dobot 명령어 상수"""
    
    # === 기본 명령어 ===
    ENABLE_ROBOT = "EnableRobot()"
    DISABLE_ROBOT = "DisableRobot()"
    CLEAR_ERROR = "ClearError()"
    RESET_ROBOT = "ResetRobot()"
    EMERGENCY_STOP = "EmergencyStop()"
    
    # === 이동 명령어 ===
    MOVE_J = "MovJ({},{},{},{})"  # 관절 이동
    MOVE_L = "MovL({},{},{},{})"  # 직선 이동
    
    # === 상태 조회 명령어 ===
    GET_POSE = "GetPose()"
    GET_ANGLES = "GetAngle()"
    
    # === 디지털 IO 명령어 ===
    SET_DO = "DO({},{})"  # 디지털 출력 설정
    GET_DI = "DI({})"     # 디지털 입력 조회
    
    # === 속도 설정 명령어 ===
    SET_SPEED = "SpeedJ({})"      # 관절 속도 설정
    SET_SPEED_L = "SpeedL({})"    # 직선 속도 설정
    
    @classmethod
    def move_j(cls, x: float, y: float, z: float, r: float = 0.0) -> str:
        """관절 이동 명령어 생성"""
        return cls.MOVE_J.format(x, y, z, r)
    
    @classmethod
    def move_l(cls, x: float, y: float, z: float, r: float = 0.0) -> str:
        """직선 이동 명령어 생성"""
        return cls.MOVE_L.format(x, y, z, r)
    
    @classmethod
    def set_gripper(cls, open_state: bool) -> str:
        """그리퍼 제어 명령어 생성"""
        return cls.SET_DO.format(1, 1 if open_state else 0)


class ErrorCodes:
    """에러 코드 정의"""
    
    # === 연결 에러 ===
    CONNECTION_FAILED = 1001
    CONNECTION_TIMEOUT = 1002
    CONNECTION_LOST = 1003
    
    # === 이동 에러 ===
    MOVEMENT_FAILED = 2001
    MOVEMENT_TIMEOUT = 2002
    POSITION_OUT_OF_RANGE = 2003
    COLLISION_DETECTED = 2004
    
    # === 그리퍼 에러 ===
    GRIPPER_FAILED = 3001
    GRIPPER_TIMEOUT = 3002
    
    # === 시스템 에러 ===
    ROBOT_NOT_ENABLED = 4001
    EMERGENCY_STOP_ACTIVE = 4002
    UNKNOWN_ERROR = 9999
    
    ERROR_MESSAGES = {
        CONNECTION_FAILED: "로봇 연결에 실패했습니다.",
        CONNECTION_TIMEOUT: "연결 시간이 초과되었습니다.",
        CONNECTION_LOST: "로봇과의 연결이 끊어졌습니다.",
        
        MOVEMENT_FAILED: "로봇 이동에 실패했습니다.",
        MOVEMENT_TIMEOUT: "이동 시간이 초과되었습니다.",
        POSITION_OUT_OF_RANGE: "목표 위치가 작업 영역을 벗어났습니다.",
        COLLISION_DETECTED: "충돌이 감지되었습니다.",
        
        GRIPPER_FAILED: "그리퍼 제어에 실패했습니다.",
        GRIPPER_TIMEOUT: "그리퍼 동작 시간이 초과되었습니다.",
        
        ROBOT_NOT_ENABLED: "로봇이 활성화되지 않았습니다.",
        EMERGENCY_STOP_ACTIVE: "비상 정지가 활성화되어 있습니다.",
        UNKNOWN_ERROR: "알 수 없는 오류가 발생했습니다."
    }
    
    @classmethod
    def get_message(cls, error_code: int) -> str:
        """에러 코드에 해당하는 메시지 반환"""
        return cls.ERROR_MESSAGES.get(error_code, cls.ERROR_MESSAGES[cls.UNKNOWN_ERROR])


# === 버전 정보 ===
VERSION = "2.0.0"
BUILD_DATE = "2024-01-01"
AUTHOR = "Dobot Controller Team"

# === 개발자 설정 ===
DEBUG_MODE = False
VERBOSE_LOGGING = False
ENABLE_PROFILING = False
