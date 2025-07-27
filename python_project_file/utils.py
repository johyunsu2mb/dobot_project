#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
유틸리티 함수 및 헬퍼 클래스
좌표 관리, 에러 처리, 공통 기능 제공
"""

import time
import math
import threading
import traceback
from typing import List, Tuple, Optional, Dict, Any, Callable
from dataclasses import dataclass
from enum import Enum
import logging
from tkinter import messagebox

from config import AppConfig, ErrorCodes


class CoordinateManager:
    """좌표 관리 및 검증 클래스"""
    
    def __init__(self):
        self.current_position = AppConfig.HOME_POSITION.copy()
        self.target_position = AppConfig.HOME_POSITION.copy()
        self.position_history = []
        self.max_history = 100
        
    def validate_coordinates(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """좌표 유효성 검증"""
        return AppConfig.validate_coordinates(x, y, z, r)
    
    def clamp_coordinates(self, x: float, y: float, z: float, r: float = 0.0) -> Tuple[float, float, float, float]:
        """좌표를 작업 영역 내로 제한"""
        return AppConfig.clamp_coordinates(x, y, z, r)
    
    def calculate_distance(self, pos1: List[float], pos2: List[float]) -> float:
        """두 점 사이의 3D 거리 계산"""
        if len(pos1) < 3 or len(pos2) < 3:
            return 0.0
            
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_move_time(self, start_pos: List[float], end_pos: List[float], 
                           speed: float = None) -> float:
        """이동 시간 계산"""
        if speed is None:
            speed = AppConfig.MOVEMENT_SPEED
            
        distance = self.calculate_distance(start_pos, end_pos)
        return max(distance / speed, 0.1)  # 최소 0.1초
    
    def update_position(self, new_position: List[float]):
        """현재 위치 업데이트"""
        if len(new_position) >= 4:
            # 히스토리 저장
            self.position_history.append({
                'position': self.current_position.copy(),
                'timestamp': time.time()
            })
            
            # 히스토리 크기 제한
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
            
            self.current_position = new_position.copy()
    
    def get_safe_position(self, x: float, y: float, z: float, r: float = 0.0) -> List[float]:
        """안전 위치 계산"""
        return AppConfig.get_safe_position(x, y, z, r)
    
    def is_within_workspace(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """작업 영역 내 여부 확인"""
        return self.validate_coordinates(x, y, z, r)
    
    def get_position_history(self, limit: int = 10) -> List[Dict]:
        """위치 히스토리 반환"""
        return self.position_history[-limit:] if self.position_history else []
    
    def interpolate_path(self, start_pos: List[float], end_pos: List[float], 
                        steps: int = 10) -> List[List[float]]:
        """두 점 사이의 보간 경로 생성"""
        if len(start_pos) < 4 or len(end_pos) < 4:
            return [start_pos, end_pos]
            
        path = []
        for i in range(steps + 1):
            t = i / steps
            interpolated = [
                start_pos[j] + t * (end_pos[j] - start_pos[j]) 
                for j in range(4)
            ]
            path.append(interpolated)
            
        return path


class ErrorHandler:
    """에러 처리 및 복구 클래스"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.error_count = 0
        self.last_error_time = 0
        self.error_threshold = 5  # 연속 에러 임계값
        self.error_reset_time = 300  # 5분 후 에러 카운트 리셋
        
    def handle_error(self, error: Exception, context: str = "", 
                    show_dialog: bool = True) -> bool:
        """통합 에러 처리"""
        try:
            # 에러 카운트 관리
            current_time = time.time()
            if current_time - self.last_error_time > self.error_reset_time:
                self.error_count = 0
                
            self.error_count += 1
            self.last_error_time = current_time
            
            # 에러 로깅
            error_msg = f"{context}: {str(error)}" if context else str(error)
            self.logger.error(error_msg)
            self.logger.debug(f"Error traceback: {traceback.format_exc()}")
            
            # 에러 분류 및 처리
            error_code = self._classify_error(error)
            error_message = ErrorCodes.get_message(error_code)
            
            # 사용자에게 알림 (선택적)
            if show_dialog:
                self._show_error_dialog(context, error_message)
            
            # 복구 시도
            recovery_success = self._attempt_recovery(error_code, error)
            
            # 연속 에러 체크
            if self.error_count >= self.error_threshold:
                self._handle_critical_error()
                
            return recovery_success
            
        except Exception as e:
            self.logger.critical(f"Error handler failed: {e}")
            return False
    
    def _classify_error(self, error: Exception) -> int:
        """에러 분류"""
        error_type = type(error).__name__
        error_message = str(error).lower()
        
        # 연결 관련 에러
        if any(keyword in error_message for keyword in ['connection', 'socket', 'timeout']):
            if 'timeout' in error_message:
                return ErrorCodes.CONNECTION_TIMEOUT
            return ErrorCodes.CONNECTION_FAILED
        
        # 이동 관련 에러
        if any(keyword in error_message for keyword in ['move', 'position', 'coordinate']):
            if 'timeout' in error_message:
                return ErrorCodes.MOVEMENT_TIMEOUT
            if 'range' in error_message or 'limit' in error_message:
                return ErrorCodes.POSITION_OUT_OF_RANGE
            return ErrorCodes.MOVEMENT_FAILED
        
        # 그리퍼 관련 에러
        if any(keyword in error_message for keyword in ['gripper', 'grip', 'clamp']):
            return ErrorCodes.GRIPPER_FAILED
        
        return ErrorCodes.UNKNOWN_ERROR
    
    def _show_error_dialog(self, context: str, message: str):
        """에러 대화상자 표시"""
        title = "오류 발생" if context else "시스템 오류"
        full_message = f"{context}\n\n{message}" if context else message
        
        try:
            messagebox.showerror(title, full_message)
        except Exception:
            # GUI가 없는 환경에서는 로그만 출력
            self.logger.error(f"Cannot show error dialog: {full_message}")
    
    def _attempt_recovery(self, error_code: int, error: Exception) -> bool:
        """에러 복구 시도"""
        if not AppConfig.ERROR_RECOVERY_ENABLED:
            return False
            
        try:
            if error_code in [ErrorCodes.CONNECTION_FAILED, ErrorCodes.CONNECTION_LOST]:
                return self._recover_connection()
            elif error_code == ErrorCodes.MOVEMENT_TIMEOUT:
                return self._recover_movement()
            elif error_code == ErrorCodes.POSITION_OUT_OF_RANGE:
                return self._recover_position()
            
        except Exception as e:
            self.logger.error(f"Recovery attempt failed: {e}")
            
        return False
    
    def _recover_connection(self) -> bool:
        """연결 복구"""
        self.logger.info("Attempting connection recovery...")
        # 실제 복구 로직은 호출하는 클래스에서 구현
        return False
    
    def _recover_movement(self) -> bool:
        """이동 복구"""
        self.logger.info("Attempting movement recovery...")
        # 안전 위치로 이동 등의 복구 로직
        return False
    
    def _recover_position(self) -> bool:
        """위치 복구"""
        self.logger.info("Attempting position recovery...")
        # 홈 위치로 이동 등의 복구 로직
        return False
    
    def _handle_critical_error(self):
        """심각한 에러 처리"""
        self.logger.critical(f"Critical error threshold reached: {self.error_count} errors")
        try:
            messagebox.showerror(
                "심각한 오류", 
                f"연속 {self.error_count}개의 오류가 발생했습니다.\n"
                "시스템을 리셋하거나 애플리케이션을 재시작하세요."
            )
        except Exception:
            pass
    
    def reset_error_count(self):
        """에러 카운트 리셋"""
        self.error_count = 0
        self.last_error_time = 0
        self.logger.info("Error count reset")


class PerformanceMonitor:
    """성능 모니터링 클래스"""
    
    def __init__(self):
        self.start_times = {}
        self.execution_times = {}
        self.logger = logging.getLogger(__name__)
        
    def start_timer(self, operation: str):
        """타이머 시작"""
        self.start_times[operation] = time.time()
        
    def end_timer(self, operation: str) -> float:
        """타이머 종료 및 실행 시간 반환"""
        if operation not in self.start_times:
            return 0.0
            
        execution_time = time.time() - self.start_times[operation]
        
        if operation not in self.execution_times:
            self.execution_times[operation] = []
        
        self.execution_times[operation].append(execution_time)
        
        # 최근 100개만 유지
        if len(self.execution_times[operation]) > 100:
            self.execution_times[operation].pop(0)
            
        if AppConfig.VERBOSE_LOGGING:
            self.logger.debug(f"{operation} execution time: {execution_time:.3f}s")
            
        return execution_time
    
    def get_average_time(self, operation: str) -> float:
        """평균 실행 시간 반환"""
        if operation not in self.execution_times:
            return 0.0
        
        times = self.execution_times[operation]
        return sum(times) / len(times) if times else 0.0
    
    def get_performance_report(self) -> Dict[str, Dict[str, float]]:
        """성능 리포트 생성"""
        report = {}
        
        for operation, times in self.execution_times.items():
            if times:
                report[operation] = {
                    'count': len(times),
                    'average': sum(times) / len(times),
                    'min': min(times),
                    'max': max(times),
                    'last': times[-1]
                }
                
        return report


class ThreadSafeLogger:
    """스레드 안전 로거 래퍼"""
    
    def __init__(self, logger_name: str):
        self.logger = logging.getLogger(logger_name)
        self.lock = threading.Lock()
        
    def _log(self, level: int, message: str, *args, **kwargs):
        """스레드 안전 로깅"""
        with self.lock:
            self.logger.log(level, message, *args, **kwargs)
    
    def debug(self, message: str, *args, **kwargs):
        self._log(logging.DEBUG, message, *args, **kwargs)
    
    def info(self, message: str, *args, **kwargs):
        self._log(logging.INFO, message, *args, **kwargs)
    
    def warning(self, message: str, *args, **kwargs):
        self._log(logging.WARNING, message, *args, **kwargs)
    
    def error(self, message: str, *args, **kwargs):
        self._log(logging.ERROR, message, *args, **kwargs)
    
    def critical(self, message: str, *args, **kwargs):
        self._log(logging.CRITICAL, message, *args, **kwargs)


class RetryManager:
    """재시도 관리 클래스"""
    
    @staticmethod
    def retry_operation(operation: Callable, max_attempts: int = None, 
                       delay: float = None, backoff_factor: float = 1.0) -> Any:
        """재시도 로직을 가진 연산 실행"""
        if max_attempts is None:
            max_attempts = AppConfig.MAX_RETRY_ATTEMPTS
        if delay is None:
            delay = AppConfig.RETRY_DELAY
            
        last_exception = None
        
        for attempt in range(max_attempts):
            try:
                return operation()
            except Exception as e:
                last_exception = e
                
                if attempt < max_attempts - 1:  # 마지막 시도가 아닌 경우
                    wait_time = delay * (backoff_factor ** attempt)
                    time.sleep(wait_time)
                    
        # 모든 시도 실패
        if last_exception:
            raise last_exception


class ConfigValidator:
    """설정 검증 클래스"""
    
    @staticmethod
    def validate_robot_config() -> bool:
        """로봇 설정 검증"""
        try:
            # IP 주소 형식 검증
            ip_parts = AppConfig.ROBOT_IP.split('.')
            if len(ip_parts) != 4:
                return False
            
            for part in ip_parts:
                if not (0 <= int(part) <= 255):
                    return False
            
            # 포트 번호 검증
            if not (1 <= AppConfig.DASHBOARD_PORT <= 65535):
                return False
            if not (1 <= AppConfig.MOVE_PORT <= 65535):
                return False
            
            # 작업 영역 검증
            if AppConfig.X_MIN >= AppConfig.X_MAX:
                return False
            if AppConfig.Y_MIN >= AppConfig.Y_MAX:
                return False
            if AppConfig.Z_MIN >= AppConfig.Z_MAX:
                return False
            
            return True
            
        except (ValueError, AttributeError):
            return False
    
    @staticmethod
    def validate_furniture_positions() -> bool:
        """가구 위치 설정 검증"""
        try:
            for furniture, positions in AppConfig.FURNITURE_POSITIONS.items():
                if 'pickup' not in positions or 'place' not in positions:
                    return False
                
                for pos_type, coords in positions.items():
                    if len(coords) != 4:
                        return False
                    
                    if not AppConfig.validate_coordinates(*coords):
                        return False
            
            return True
            
        except (TypeError, KeyError):
            return False


# === 유틸리티 함수 ===

def format_position(position: List[float]) -> str:
    """위치 정보를 문자열로 포맷"""
    if len(position) >= 4:
        return f"X:{position[0]:.1f}, Y:{position[1]:.1f}, Z:{position[2]:.1f}, R:{position[3]:.1f}"
    return "Invalid position"

def format_time_duration(seconds: float) -> str:
    """시간을 읽기 쉬운 형태로 포맷"""
    if seconds < 60:
        return f"{seconds:.1f}초"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}분 {secs:.1f}초"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        return f"{hours}시간 {minutes}분"

def safe_float_convert(value: Any, default: float = 0.0) -> float:
    """안전한 float 변환"""
    try:
        return float(value)
    except (ValueError, TypeError):
        return default

def clamp_value(value: float, min_val: float, max_val: float) -> float:
    """값을 범위 내로 제한"""
    return max(min_val, min(max_val, value))

def degrees_to_radians(degrees: float) -> float:
    """도를 라디안으로 변환"""
    return degrees * math.pi / 180.0

def radians_to_degrees(radians: float) -> float:
    """라디안을 도로 변환"""
    return radians * 180.0 / math.pi

def normalize_angle(angle: float) -> float:
    """각도를 -180~180 범위로 정규화"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def create_directory_if_not_exists(directory: str):
    """디렉토리가 없으면 생성"""
    import os
    if not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)

def get_timestamp() -> str:
    """현재 시간의 타임스탬프 문자열 반환"""
    import datetime
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
