#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot 로봇팔 제어 클래스
시뮬레이션 모드와 실제 로봇 제어를 통합 관리
"""

import time
import threading
import socket
from typing import Optional, List, Tuple, Dict, Any
import logging
from dataclasses import dataclass
from enum import Enum

from config import AppConfig


class RobotState(Enum):
    """로봇 상태 열거형"""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    MOVING = "moving"
    ERROR = "error"
    EMERGENCY = "emergency"


@dataclass
class RobotPosition:
    """로봇 위치 데이터 클래스"""
    x: float
    y: float
    z: float
    r: float
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.r]
    
    def __str__(self) -> str:
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.r:.2f})"


class SimulationRobot:
    """시뮬레이션 로봇 클래스"""
    
    def __init__(self):
        self.position = RobotPosition(250.0, 0.0, 50.0, 0.0)
        self.gripper_open = True
        self.is_moving = False
        self.move_speed = 100.0  # mm/s
        
    def move_to(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """시뮬레이션 이동"""
        try:
            # 이동 시간 계산 (거리 기반)
            distance = ((x - self.position.x)**2 + 
                       (y - self.position.y)**2 + 
                       (z - self.position.z)**2)**0.5
            move_time = distance / self.move_speed
            
            self.is_moving = True
            
            # 실제로는 threading을 사용하여 점진적 이동 구현
            def simulate_move():
                time.sleep(min(move_time, 3.0))  # 최대 3초
                self.position = RobotPosition(x, y, z, r)
                self.is_moving = False
                
            threading.Thread(target=simulate_move, daemon=True).start()
            return True
            
        except Exception:
            self.is_moving = False
            return False
    
    def set_gripper(self, open_state: bool) -> bool:
        """그리퍼 제어"""
        time.sleep(0.5)  # 그리퍼 동작 시뮬레이션
        self.gripper_open = open_state
        return True
    
    def get_position(self) -> RobotPosition:
        """현재 위치 반환"""
        return self.position
    
    def is_connected(self) -> bool:
        """연결 상태 (항상 True)"""
        return True


class DobotAPI:
    """실제 Dobot API 래퍼 클래스"""
    
    def __init__(self, ip: str, dashboard_port: int, move_port: int, logger: logging.Logger):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.logger = logger
        
        self.dashboard_socket: Optional[socket.socket] = None
        self.move_socket: Optional[socket.socket] = None
        self.connected = False
        
        self.current_position = RobotPosition(250.0, 0.0, 50.0, 0.0)
        self.gripper_open = True
        
    def connect(self) -> bool:
        """로봇에 연결"""
        try:
            # Dashboard 포트 연결
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(5.0)
            self.dashboard_socket.connect((self.ip, self.dashboard_port))
            
            # Move 포트 연결
            self.move_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.move_socket.settimeout(5.0)
            self.move_socket.connect((self.ip, self.move_port))
            
            # 로봇 활성화
            self._send_command("EnableRobot()")
            time.sleep(1)
            
            self.connected = True
            self.logger.info(f"Dobot 연결 성공: {self.ip}")
            return True
            
        except Exception as e:
            self.logger.error(f"Dobot 연결 실패: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """연결 해제"""
        try:
            if self.dashboard_socket:
                self.dashboard_socket.close()
                self.dashboard_socket = None
                
            if self.move_socket:
                self.move_socket.close()
                self.move_socket = None
                
            self.connected = False
            self.logger.info("Dobot 연결 해제")
            
        except Exception as e:
            self.logger.error(f"연결 해제 중 오류: {e}")
    
    def _send_command(self, command: str) -> Optional[str]:
        """명령 전송"""
        try:
            if not self.dashboard_socket:
                return None
                
            self.dashboard_socket.send(f"{command}\n".encode())
            response = self.dashboard_socket.recv(1024).decode().strip()
            return response
            
        except Exception as e:
            self.logger.error(f"명령 전송 실패: {command}, 오류: {e}")
            return None
    
    def move_to(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """지정 위치로 이동"""
        try:
            if not self.connected:
                return False
                
            # MovJ 명령 사용 (관절 이동)
            command = f"MovJ({x},{y},{z},{r})"
            response = self._send_command(command)
            
            if response and "0" in response:  # 성공 응답
                self.current_position = RobotPosition(x, y, z, r)
                self.logger.debug(f"이동 명령 전송: {command}")
                return True
            else:
                self.logger.warning(f"이동 명령 실패: {command}, 응답: {response}")
                return False
                
        except Exception as e:
            self.logger.error(f"이동 중 오류: {e}")
            return False
    
    def set_gripper(self, open_state: bool) -> bool:
        """그리퍼 제어"""
        try:
            if not self.connected:
                return False
                
            # DO 명령 사용 (디지털 출력)
            command = f"DO(1,{1 if open_state else 0})"
            response = self._send_command(command)
            
            if response and "0" in response:
                self.gripper_open = open_state
                time.sleep(0.5)  # 그리퍼 동작 대기
                return True
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"그리퍼 제어 오류: {e}")
            return False
    
    def get_position(self) -> RobotPosition:
        """현재 위치 조회"""
        try:
            if not self.connected:
                return self.current_position
                
            response = self._send_command("GetPose()")
            if response:
                # 응답 파싱 (예: "{250.0,0.0,50.0,0.0}")
                coords = response.strip('{}').split(',')
                if len(coords) >= 4:
                    x, y, z, r = map(float, coords[:4])
                    self.current_position = RobotPosition(x, y, z, r)
                    
            return self.current_position
            
        except Exception as e:
            self.logger.error(f"위치 조회 오류: {e}")
            return self.current_position
    
    def emergency_stop(self) -> bool:
        """비상 정지"""
        try:
            response = self._send_command("EmergencyStop()")
            return response and "0" in response
        except Exception as e:
            self.logger.error(f"비상 정지 오류: {e}")
            return False
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.connected


class RobotController:
    """통합 로봇 제어 클래스"""
    
    def __init__(self, simulation_mode: bool = True, ip_address: str = None, 
                 dashboard_port: int = 29999, move_port: int = 30003, 
                 logger: logging.Logger = None):
        
        self.logger = logger or logging.getLogger(__name__)
        self.simulation_mode = simulation_mode
        self.state = RobotState.DISCONNECTED
        
        # 로봇 인스턴스 초기화
        if simulation_mode:
            self.robot = SimulationRobot()
            self.state = RobotState.CONNECTED
            self.logger.info("시뮬레이션 모드로 초기화")
        else:
            self.robot = DobotAPI(
                ip=ip_address or AppConfig.ROBOT_IP,
                dashboard_port=dashboard_port,
                move_port=move_port,
                logger=self.logger
            )
            self.logger.info("실제 로봇 모드로 초기화")
    
    def connect(self) -> bool:
        """로봇 연결"""
        try:
            if self.simulation_mode:
                self.state = RobotState.CONNECTED
                return True
            else:
                success = self.robot.connect()
                self.state = RobotState.CONNECTED if success else RobotState.DISCONNECTED
                return success
                
        except Exception as e:
            self.logger.error(f"연결 실패: {e}")
            self.state = RobotState.ERROR
            return False
    
    def disconnect(self):
        """연결 해제"""
        try:
            if not self.simulation_mode and hasattr(self.robot, 'disconnect'):
                self.robot.disconnect()
            self.state = RobotState.DISCONNECTED
            
        except Exception as e:
            self.logger.error(f"연결 해제 실패: {e}")
    
    def move_to(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """지정 위치로 이동"""
        try:
            if self.state not in [RobotState.CONNECTED]:
                self.logger.warning("로봇이 연결되지 않음")
                return False
                
            # 좌표 범위 검증
            if not self._validate_position(x, y, z, r):
                self.logger.warning(f"좌표 범위 초과: ({x}, {y}, {z}, {r})")
                return False
            
            self.state = RobotState.MOVING
            success = self.robot.move_to(x, y, z, r)
            
            if success:
                self.state = RobotState.CONNECTED
                self.logger.info(f"이동 완료: ({x}, {y}, {z}, {r})")
            else:
                self.state = RobotState.ERROR
                self.logger.error(f"이동 실패: ({x}, {y}, {z}, {r})")
                
            return success
            
        except Exception as e:
            self.logger.error(f"이동 중 예외: {e}")
            self.state = RobotState.ERROR
            return False
    
    def _validate_position(self, x: float, y: float, z: float, r: float) -> bool:
        """좌표 유효성 검증"""
        return (AppConfig.X_MIN <= x <= AppConfig.X_MAX and
                AppConfig.Y_MIN <= y <= AppConfig.Y_MAX and
                AppConfig.Z_MIN <= z <= AppConfig.Z_MAX and
                -180 <= r <= 180)
    
    def set_gripper(self, open_state: bool) -> bool:
        """그리퍼 제어"""
        try:
            if self.state not in [RobotState.CONNECTED, RobotState.MOVING]:
                return False
                
            success = self.robot.set_gripper(open_state)
            if success:
                state_text = "열림" if open_state else "닫힘"
                self.logger.info(f"그리퍼 상태 변경: {state_text}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"그리퍼 제어 실패: {e}")
            return False
    
    def get_gripper_state(self) -> bool:
        """그리퍼 상태 조회"""
        try:
            return getattr(self.robot, 'gripper_open', True)
        except Exception:
            return True
    
    def get_current_position(self) -> Optional[List[float]]:
        """현재 위치 조회"""
        try:
            if hasattr(self.robot, 'get_position'):
                pos = self.robot.get_position()
                if isinstance(pos, RobotPosition):
                    return pos.to_list()
                return pos
            else:
                # 시뮬레이션 로봇의 경우
                return [self.robot.position.x, self.robot.position.y, 
                       self.robot.position.z, self.robot.position.r]
                
        except Exception as e:
            self.logger.error(f"위치 조회 실패: {e}")
            return None
    
    def pickup_sequence(self, target_position: List[float], 
                       final_position: List[float]) -> bool:
        """픽업 시퀀스 실행"""
        try:
            self.logger.info("픽업 시퀀스 시작")
            
            # 1. 안전 높이로 이동
            safe_pos = target_position.copy()
            safe_pos[2] += AppConfig.SAFETY_HEIGHT_OFFSET
            
            if not self.move_to(*safe_pos):
                return False
            time.sleep(1)
            
            # 2. 그리퍼 열기
            if not self.set_gripper(True):
                return False
            time.sleep(1)
            
            # 3. 타겟 위치로 하강
            if not self.move_to(*target_position):
                return False
            time.sleep(1)
            
            # 4. 그리퍼 닫기 (물체 집기)
            if not self.set_gripper(False):
                return False
            time.sleep(1.5)
            
            # 5. 안전 높이로 상승
            if not self.move_to(*safe_pos):
                return False
            time.sleep(1)
            
            # 6. 최종 위치로 이동
            final_safe = final_position.copy()
            final_safe[2] += AppConfig.SAFETY_HEIGHT_OFFSET
            
            if not self.move_to(*final_safe):
                return False
            time.sleep(1)
            
            # 7. 최종 위치로 하강
            if not self.move_to(*final_position):
                return False
            time.sleep(1)
            
            # 8. 그리퍼 열기 (물체 놓기)
            if not self.set_gripper(True):
                return False
            time.sleep(1)
            
            # 9. 안전 높이로 상승
            if not self.move_to(*final_safe):
                return False
            
            self.logger.info("픽업 시퀀스 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"픽업 시퀀스 실패: {e}")
            return False
    
    def emergency_stop(self) -> bool:
        """비상 정지"""
        try:
            if hasattr(self.robot, 'emergency_stop'):
                success = self.robot.emergency_stop()
            else:
                success = True  # 시뮬레이션에서는 항상 성공
                
            if success:
                self.state = RobotState.EMERGENCY
                self.logger.warning("비상 정지 실행")
            
            return success
            
        except Exception as e:
            self.logger.error(f"비상 정지 실패: {e}")
            return False
    
    def reset(self) -> bool:
        """로봇 리셋"""
        try:
            # 비상 정지 해제 (실제 로봇의 경우)
            if not self.simulation_mode and hasattr(self.robot, '_send_command'):
                self.robot._send_command("ClearError()")
                time.sleep(1)
                self.robot._send_command("EnableRobot()")
                time.sleep(1)
            
            self.state = RobotState.CONNECTED
            self.logger.info("로봇 리셋 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"로봇 리셋 실패: {e}")
            return False
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.state in [RobotState.CONNECTED, RobotState.MOVING]
    
    def get_state(self) -> RobotState:
        """현재 상태 반환"""
        return self.state
