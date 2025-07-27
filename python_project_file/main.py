#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot 로봇팔 제어 시스템 - 메인 애플리케이션
YOLOv8 기능 제거, 좌표 동기화 문제 해결, 연결 상태 개선
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from typing import Optional, Tuple, List, Dict
import logging

from robot_controller import RobotController
from ui_components import ModernUI
from config import AppConfig
from logger_setup import setup_logger
from utils import CoordinateManager, ErrorHandler
from yolo_detector import YOLOSystem, DetectedObject


class DobotApp:
    """Dobot 로봇팔 제어 메인 애플리케이션"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.robot: Optional[RobotController] = None
        self.yolo_system: Optional[YOLOSystem] = None
        self.coordinate_manager = CoordinateManager()
        self.error_handler = ErrorHandler()
        self.logger = setup_logger('DobotApp')
        
        # 애플리케이션 상태
        self.is_connected = False
        self.is_simulation = True
        self.current_position = [250.0, 0.0, 50.0, 0.0]
        self.camera_active = False
        self.detected_objects = []
        
        self._setup_ui()
        self._initialize_robot()
        self._initialize_yolo()
        self._start_monitoring()
        
    def _setup_ui(self):
        """UI 초기화"""
        self.root.title("Dobot 로봇팔 제어 시스템 v2.0")
        self.root.geometry("1200x800")
        self.root.configure(bg='#f0f0f0')
        
        # 현대적인 UI 컴포넌트 생성
        self.ui = ModernUI(self.root, self)
        self.ui.create_main_interface()
        
        # 상태 표시줄
        self._create_status_bar()
        
    def _create_status_bar(self):
        """상태 표시줄 생성"""
        self.status_frame = ttk.Frame(self.root)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=2)
        
        # 연결 상태
        self.connection_label = ttk.Label(
            self.status_frame, 
            text="연결 상태: 시뮬레이션 모드",
            foreground="orange"
        )
        self.connection_label.pack(side=tk.LEFT, padx=5)
        
        # 현재 좌표
        self.position_label = ttk.Label(
            self.status_frame, 
            text="위치: X=250, Y=0, Z=50, R=0"
        )
        self.position_label.pack(side=tk.RIGHT, padx=5)
        
    def _initialize_robot(self):
        """로봇 초기화"""
        try:
            self.robot = RobotController(
                simulation_mode=True,  # 기본적으로 시뮬레이션 모드
                logger=self.logger
            )
            self.is_simulation = True
            self.logger.info("로봇 컨트롤러 초기화 완료 (시뮬레이션 모드)")
            
        except Exception as e:
            self.error_handler.handle_error(e, "로봇 초기화 실패")
            self.logger.error(f"로봇 초기화 실패: {e}")
    
    def _initialize_yolo(self):
        """YOLOv8 시스템 초기화"""
        try:
            from yolo_detector import create_yolo_system, YOLO_AVAILABLE
            
            if YOLO_AVAILABLE:
                self.yolo_system = create_yolo_system(
                    camera_index=AppConfig.CAMERA_INDEX,
                    model_path=AppConfig.YOLO_MODEL_PATH
                )
                
                # 객체 감지 콜백 설정
                self.yolo_system.add_detection_callback(self._on_objects_detected)
                
                self.logger.info("YOLOv8 시스템 초기화 완료")
            else:
                self.logger.warning("YOLOv8를 사용할 수 없습니다. 카메라 기능이 제한됩니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "YOLOv8 초기화 실패")
            self.logger.error(f"YOLOv8 초기화 실패: {e}")
    
    def _on_objects_detected(self, detections: List[DetectedObject]):
        """객체 감지 콜백"""
        self.detected_objects = detections
        
        # UI 업데이트 (메인 스레드에서 실행)
        self.root.after(0, self._update_detection_display)
            
    def _start_monitoring(self):
        """백그라운드 모니터링 시작"""
        def monitor():
            while True:
                try:
                    self._update_status()
                    time.sleep(1)  # 1초마다 업데이트
                except Exception as e:
                    self.logger.error(f"모니터링 오류: {e}")
                time.sleep(5)
                
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        
    def _update_status(self):
        """상태 업데이트"""
        if self.robot:
            # 현재 위치 업데이트
            try:
                position = self.robot.get_current_position()
                if position:
                    self.current_position = position
                    self.coordinate_manager.update_position(position)
                    
                    # UI 업데이트 (메인 스레드에서 실행)
                    self.root.after(0, self._update_ui_status)
                    
            except Exception as e:
                self.logger.warning(f"위치 업데이트 실패: {e}")
                
    def _update_ui_status(self):
        """UI 상태 업데이트 (메인 스레드에서 실행)"""
        # 연결 상태 업데이트
        if self.is_simulation:
            status_text = "연결 상태: 시뮬레이션 모드"
            status_color = "orange"
        elif self.is_connected:
            status_text = "연결 상태: 연결됨"
            status_color = "green"
        else:
            status_text = "연결 상태: 연결 끊김"
            status_color = "red"
            
        self.connection_label.config(text=status_text, foreground=status_color)
        
        # 좌표 업데이트
        x, y, z, r = self.current_position
        pos_text = f"위치: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, R={r:.1f}"
        self.position_label.config(text=pos_text)
        
        # UI 컴포넌트의 좌표 표시 업데이트
        if hasattr(self.ui, 'update_coordinate_display'):
            self.ui.update_coordinate_display(self.current_position)
    
    def _update_detection_display(self):
        """객체 감지 결과 UI 업데이트"""
        if hasattr(self.ui, 'update_detection_display'):
            self.ui.update_detection_display(self.detected_objects)
    
    # === 로봇 제어 메서드 ===
    
    def move_to_position(self, x: float, y: float, z: float, r: float = 0.0) -> bool:
        """지정된 위치로 이동"""
        try:
            # 좌표 검증
            if not self.coordinate_manager.validate_coordinates(x, y, z, r):
                messagebox.showerror("오류", "좌표값이 작업 영역을 벗어났습니다.")
                return False
                
            if self.robot:
                success = self.robot.move_to(x, y, z, r)
                if success:
                    self.current_position = [x, y, z, r]
                    self.logger.info(f"이동 완료: ({x}, {y}, {z}, {r})")
                    return True
                else:
                    messagebox.showerror("오류", "로봇 이동에 실패했습니다.")
                    return False
            else:
                messagebox.showerror("오류", "로봇이 연결되어 있지 않습니다.")
                return False
                
        except Exception as e:
            self.error_handler.handle_error(e, "로봇 이동 실패")
            return False
    
    def pickup_furniture(self, furniture_type: str):
        """가구 픽업 시퀀스 실행"""
        try:
            if not self.robot:
                messagebox.showerror("오류", "로봇이 연결되어 있지 않습니다.")
                return
                
            # 가구별 좌표 설정
            furniture_positions = AppConfig.FURNITURE_POSITIONS.get(furniture_type)
            if not furniture_positions:
                messagebox.showerror("오류", f"알 수 없는 가구 타입: {furniture_type}")
                return
                
            self.logger.info(f"{furniture_type} 픽업 시작")
            
            # 픽업 시퀀스 실행
            success = self.robot.pickup_sequence(
                target_position=furniture_positions['pickup'],
                final_position=furniture_positions['place']
            )
            
            if success:
                messagebox.showinfo("완료", f"{furniture_type} 픽업이 완료되었습니다.")
                self.logger.info(f"{furniture_type} 픽업 완료")
            else:
                messagebox.showerror("오류", f"{furniture_type} 픽업에 실패했습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, f"{furniture_type} 픽업 실패")
    
    def toggle_gripper(self):
        """그리퍼 토글"""
        try:
            if self.robot:
                current_state = self.robot.get_gripper_state()
                new_state = not current_state
                success = self.robot.set_gripper(new_state)
                
                if success:
                    state_text = "열림" if new_state else "닫힘"
                    self.logger.info(f"그리퍼 상태 변경: {state_text}")
                else:
                    messagebox.showerror("오류", "그리퍼 제어에 실패했습니다.")
            else:
                messagebox.showerror("오류", "로봇이 연결되어 있지 않습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "그리퍼 제어 실패")
    
    def try_connect_robot(self):
        """실제 로봇 연결 시도"""
        try:
            # 새로운 로봇 컨트롤러 생성 (실제 연결 모드)
            test_robot = RobotController(
                simulation_mode=False,
                ip_address=AppConfig.ROBOT_IP,
                dashboard_port=AppConfig.DASHBOARD_PORT,
                move_port=AppConfig.MOVE_PORT,
                logger=self.logger
            )
            
            # 연결 테스트
            if test_robot.connect():
                # 기존 로봇 연결 해제
                if self.robot:
                    self.robot.disconnect()
                    
                self.robot = test_robot
                self.is_connected = True
                self.is_simulation = False
                
                messagebox.showinfo("성공", "로봇에 성공적으로 연결되었습니다.")
                self.logger.info("실제 로봇 연결 성공")
            else:
                test_robot.disconnect()
                messagebox.showwarning("실패", "로봇 연결에 실패했습니다. 시뮬레이션 모드를 유지합니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "로봇 연결 실패")
            messagebox.showerror("오류", f"로봇 연결 중 오류가 발생했습니다: {str(e)}")
    
    def go_home(self):
        """홈 위치로 이동"""
        home_pos = AppConfig.HOME_POSITION
        self.move_to_position(*home_pos)
    
    def emergency_stop(self):
        """비상 정지"""
        try:
            if self.robot:
                self.robot.emergency_stop()
                self.logger.warning("비상 정지 실행")
                messagebox.showwarning("비상 정지", "로봇이 비상 정지되었습니다.")
            else:
                messagebox.showwarning("경고", "로봇이 연결되어 있지 않습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "비상 정지 실패")
    
    def reset_system(self):
        """시스템 리셋"""
        try:
            if self.robot:
                self.robot.reset()
                self.go_home()
                self.logger.info("시스템 리셋 완료")
                messagebox.showinfo("완료", "시스템이 리셋되었습니다.")
            else:
                messagebox.showwarning("경고", "로봇이 연결되어 있지 않습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "시스템 리셋 실패")
    
    # === YOLOv8 관련 메서드 ===
    
    def toggle_camera(self):
        """카메라 토글 (시작/중지)"""
        try:
            if not self.yolo_system:
                messagebox.showerror("오류", "YOLOv8 시스템이 초기화되지 않았습니다.")
                return
            
            if not self.camera_active:
                # 카메라 시작
                if self.yolo_system.start_detection():
                    self.camera_active = True
                    self.logger.info("카메라 및 객체 감지 시작")
                    messagebox.showinfo("시작", "카메라 및 객체 감지를 시작했습니다.")
                else:
                    messagebox.showerror("오류", "카메라를 시작할 수 없습니다.")
            else:
                # 카메라 중지
                self.yolo_system.stop_detection()
                self.camera_active = False
                self.detected_objects = []
                self.logger.info("카메라 및 객체 감지 중지")
                messagebox.showinfo("중지", "카메라 및 객체 감지를 중지했습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "카메라 토글 실패")
    
    def pickup_detected_object(self, object_index: int):
        """감지된 객체 픽업"""
        try:
            if not self.detected_objects or object_index >= len(self.detected_objects):
                messagebox.showerror("오류", "선택된 객체가 없습니다.")
                return
            
            detected_obj = self.detected_objects[object_index]
            
            if not self.yolo_system:
                messagebox.showerror("오류", "YOLOv8 시스템이 없습니다.")
                return
            
            # 카메라 프레임 크기 가져오기
            frame = self.yolo_system.camera.get_current_frame()
            if frame is None:
                messagebox.showerror("오류", "카메라 프레임을 가져올 수 없습니다.")
                return
            
            height, width = frame.shape[:2]
            
            # 픽업 좌표 계산
            pickup_coords = self.yolo_system.detector.get_pickup_coordinates(
                detected_obj, width, height
            )
            
            # 픽업 시퀀스 실행
            target_pos = [pickup_coords['x'], pickup_coords['y'], pickup_coords['z'], pickup_coords['r']]
            final_pos = AppConfig.FURNITURE_POSITIONS['detected']['place']  # 기본 배치 위치
            
            success = self.robot.pickup_sequence(target_pos, final_pos)
            
            if success:
                korean_name = self.yolo_system.detector.class_names_korean.get(
                    detected_obj.class_name, detected_obj.class_name
                )
                messagebox.showinfo("완료", f"{korean_name} 픽업이 완료되었습니다.")
                self.logger.info(f"감지된 객체 픽업 완료: {detected_obj.class_name}")
            else:
                messagebox.showerror("오류", "객체 픽업에 실패했습니다.")
                
        except Exception as e:
            self.error_handler.handle_error(e, "객체 픽업 실패")
    
    def move_to_detected_object(self, object_index: int):
        """감지된 객체 위치로 이동"""
        try:
            if not self.detected_objects or object_index >= len(self.detected_objects):
                messagebox.showerror("오류", "선택된 객체가 없습니다.")
                return
            
            detected_obj = self.detected_objects[object_index]
            
            if not self.yolo_system:
                messagebox.showerror("오류", "YOLOv8 시스템이 없습니다.")
                return
            
            # 카메라 프레임 크기 가져오기
            frame = self.yolo_system.camera.get_current_frame()
            if frame is None:
                messagebox.showerror("오류", "카메라 프레임을 가져올 수 없습니다.")
                return
            
            height, width = frame.shape[:2]
            
            # 이동 좌표 계산
            coords = self.yolo_system.detector.get_pickup_coordinates(
                detected_obj, width, height
            )
            
            # 안전 높이로 이동
            safe_z = coords['z'] + AppConfig.SAFETY_HEIGHT_OFFSET
            success = self.move_to_position(coords['x'], coords['y'], safe_z, coords['r'])
            
            if success:
                korean_name = self.yolo_system.detector.class_names_korean.get(
                    detected_obj.class_name, detected_obj.class_name
                )
                self.logger.info(f"감지된 객체 위치로 이동: {detected_obj.class_name}")
            
        except Exception as e:
            self.error_handler.handle_error(e, "객체 위치 이동 실패")
    
    def run(self):
        """애플리케이션 실행"""
        try:
            self.logger.info("Dobot 애플리케이션 시작")
            self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
            self.root.mainloop()
        except Exception as e:
            self.error_handler.handle_error(e, "애플리케이션 실행 오류")
    
    def _on_closing(self):
        """애플리케이션 종료 시 처리"""
        try:
            if self.yolo_system and self.camera_active:
                self.yolo_system.stop_detection()
            if self.robot:
                self.robot.disconnect()
            self.logger.info("Dobot 애플리케이션 종료")
            self.root.destroy()
        except Exception as e:
            self.logger.error(f"종료 중 오류: {e}")
            self.root.destroy()


def main():
    """메인 함수"""
    try:
        app = DobotApp()
        app.run()
    except Exception as e:
        logging.error(f"애플리케이션 시작 실패: {e}")
        messagebox.showerror("오류", f"애플리케이션을 시작할 수 없습니다: {str(e)}")


if __name__ == "__main__":
    main()
