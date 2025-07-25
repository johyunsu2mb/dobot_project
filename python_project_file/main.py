"""
main.py - 메인 애플리케이션 (버그 수정됨)
Enhanced Dobot Robot & YOLO Object Detection System
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import logging
from datetime import datetime
from typing import List

# 로컬 모듈 임포트
from config import (
    RobotConfig, FURNITURE_INFO, UI_COLORS, DEPENDENCIES
)
from logger_setup import OrderLogger, system_logger
from robot_controller import RobotController
from yolo_detector import YOLODetector
from ui_components import RobotArmVisualizer, CameraDisplay, LogDisplay, DetectionDisplay
from utils import (
    create_font_directory, register_font_file, safe_float_conversion, 
    darken_color, RobotConnectionError, RobotMovementError, 
    InvalidPositionError, GripperError, TimeoutError, validate_position
)

# 조건부 임포트
if DEPENDENCIES['CV2_AVAILABLE']:
    import cv2

if DEPENDENCIES['PIL_AVAILABLE']:
    from PIL import Image, ImageTk

class FurnitureOrderSystem:
    def __init__(self, root):
        self.root = root
        self.root.title("Enhanced Dobot Robot & YOLO Object Detection System (Stability Improved)")
        self.root.geometry("1600x1000")
        self.root.configure(bg=UI_COLORS['primary_bg'])
        self.root.resizable(True, True)
        
        # 로거 설정 (안전한 로깅)
        self.logger = logging.getLogger('robot_system.main')
        
        # 폰트 설정
        create_font_directory()
        self.korean_font = register_font_file()
        try:
            self.logger.info(f"Font being used: {self.korean_font}")
        except:
            print(f"Font being used: {self.korean_font}")
       
        # 상태 변수들
        self.current_order = None
        self.is_processing = False
        self.camera_active = False
        self.cap = None
        self.total_orders = 0
        self.successful_orders = 0
       
        # 향상된 로봇 컨트롤러
        self.robot_config = RobotConfig()
        self.robot_controller = RobotController(self.robot_config)
       
        # YOLO 객체 인식기
        self.yolo_detector = YOLODetector()
       
        # 로거 초기화
        self.order_logger = OrderLogger()
       
        # UI 컴포넌트들 초기화
        self.coord_entries = {}
        self.robot_status_label = None
        self.robot_connection_label = None
        self.yolo_status_label = None
        self.camera_btn = None
        self.reconnect_btn = None
        self.reset_btn = None
        self.labels_status_label = None
        self.status_time_label = None
        self.status_system_label = None
        
        # 로봇 연결 시도
        self.connect_robot()
       
        self.setup_ui()
        
        # 시작 메시지
        self.root.after(1000, self.show_welcome_message)
   
    def connect_robot(self):
        """로봇 연결 (안전한 로깅)"""
        try:
            success = self.robot_controller.connect()
            if success:
                try:
                    self.logger.info("Real Dobot robot connection successful!")
                except:
                    print("Real Dobot robot connection successful!")
            else:
                try:
                    self.logger.warning("Robot connection failed, running in simulation mode")
                except:
                    print("Robot connection failed, running in simulation mode")
        except RobotConnectionError as e:
            try:
                self.logger.error(f"Robot connection error: {e}")
            except:
                print(f"Robot connection error: {e}")
            messagebox.showwarning("Connection Warning", 
                f"Robot connection failed:\n{str(e)}\n\nContinuing in simulation mode.")

    def execute_pickup_sequence(self, furniture_name: str):
        """향상된 가구 픽업 시퀀스 실행 (안전한 로깅)"""
        if self.is_processing:
            messagebox.showwarning("경고", "현재 다른 작업을 처리 중입니다!")
            return
       
        if furniture_name not in FURNITURE_INFO:
            self.log_display.add_message(f"[ERROR] 알 수 없는 가구: {furniture_name}")
            return
       
        # 입력 유효성 검증
        info = FURNITURE_INFO[furniture_name]
        position = info['position']
        
        if not validate_position(position):
            self.log_display.add_message(f"[ERROR] 잘못된 가구 위치: {furniture_name} - {position}")
            messagebox.showerror("오류", f"가구 위치가 작업 공간을 벗어났습니다: {position}")
            return
       
        self.is_processing = True
        self.current_order = furniture_name
        self.order_logger.log_order(furniture_name, "시작")
       
        self.log_display.add_message(f"[TARGET] {furniture_name} 향상된 픽업 시퀀스 시작")
        self.log_display.add_message(f"[PIN] 목표 위치: {position}")
       
        from config import RobotStatus
        self.robot_controller.status = RobotStatus.PICKING
        self.update_robot_status(f"로봇 상태: {furniture_name} 픽업 작업 중...", UI_COLORS['warning'])
       
        # 백그라운드 스레드에서 픽업 시퀀스 실행
        pickup_thread = threading.Thread(
            target=self._enhanced_pickup_sequence_worker, 
            args=(furniture_name, position),
            daemon=True
        )
        pickup_thread.start()

    def _enhanced_pickup_sequence_worker(self, furniture_name: str, position: List[float]):
        """향상된 백그라운드 픽업 시퀀스 (수정된 로직, 안전한 로깅)"""
        try:
            # 1. 안전 위치로 이동
            safe_position = [
                position[0], 
                position[1], 
                position[2] + self.robot_config.safety_height_offset, 
                position[3]
            ]
            
            self.log_display.add_message(f"1. 안전 위치로 이동: {safe_position}")
            self.robot_controller.move_to_position(safe_position)
            time.sleep(0.5)
           
            # 2. 그리퍼 열기
            self.log_display.add_message("2. 그리퍼 열기")
            self.robot_controller.control_gripper(False)
           
            # 3. 목표 위치로 하강
            self.log_display.add_message(f"3. {furniture_name} 위치로 하강: {position}")
            self.robot_controller.move_to_position(position)
            time.sleep(0.5)
           
            # 4. 그리퍼 닫기 (물체 집기)
            self.log_display.add_message(f"4. {furniture_name} 집기 - 그리퍼 활성화")
            self.robot_controller.control_gripper(True)
            from config import RobotStatus
            self.robot_controller.status = RobotStatus.CARRYING
           
            # 5. 안전 위치로 상승
            self.log_display.add_message("5. 물체를 들고 안전 위치로 상승")
            self.robot_controller.move_to_position(safe_position)
            time.sleep(0.5)
           
            # 6. 베이스 위치로 이동
            base_position = [300, -30, 5, 0]
            self.log_display.add_message(f"6. 베이스 위치로 이동: {base_position}")
            self.robot_controller.move_to_position(base_position)
            time.sleep(0.5)
           
            # 7. 수정된 로직: 최종 배치 위치로 이동 (그리퍼 유지)
            # [350, 0, 물건의 Z좌표값, 임의값]으로 이동
            final_position = [350, 0, position[2], position[3]]  # 물건의 Z좌표 사용
            self.log_display.add_message(f"7. 최종 배치 위치로 이동: {final_position}")
            self.robot_controller.move_to_position(final_position)
            self.robot_controller.status = RobotStatus.PLACING
            time.sleep(1.0)
           
            # 8. 작업 완료 (그리퍼는 열지 않음)
            self.log_display.add_message(f"[SUCCESS] {furniture_name} 픽업 및 배치 시퀀스 완료!")
            self.log_display.add_message(f"[INFO] 최종 위치: {final_position} (그리퍼 유지)")
           
            self.order_logger.log_order(furniture_name, "완료", f"최종위치: {final_position}")
            
            # UI 업데이트를 메인 스레드에서 실행
            self.root.after(0, lambda: self._pickup_sequence_complete(furniture_name, True))
           
        except (RobotMovementError, InvalidPositionError, GripperError, TimeoutError) as e:
            error_msg = f"[ERROR] {furniture_name} 픽업 실패: {str(e)}"
            self.log_display.add_message(error_msg)
            self.order_logger.log_order(furniture_name, "실패", str(e))
            self.root.after(0, lambda: self._pickup_sequence_complete(furniture_name, False))
        except Exception as e:
            error_msg = f"[ERROR] {furniture_name} 픽업 중 예상치 못한 오류: {str(e)}"
            self.log_display.add_message(error_msg)
            self.logger.error(error_msg)
            self.order_logger.log_order(furniture_name, "오류", str(e))
            self.root.after(0, lambda: self._pickup_sequence_complete(furniture_name, False))

    def _pickup_sequence_complete(self, furniture_name: str, success: bool):
        """픽업 시퀀스 완료 처리 (안전한 메시지)"""
        self.is_processing = False
        from config import RobotStatus
        self.robot_controller.status = RobotStatus.IDLE
       
        if success:
            self.successful_orders += 1
            self.update_robot_status(f"로봇 상태: {furniture_name} 작업 완료", UI_COLORS['success'])
            messagebox.showinfo(
                "작업 완료",
                f"[PARTY] {furniture_name} 픽업 및 배치 작업이 성공적으로 완료되었습니다!\n"
                f"최종 위치: [350, 0, 물건Z좌표, 회전값]"
            )
        else:
            self.update_robot_status("로봇 상태: 작업 실패", UI_COLORS['error'])
            messagebox.showerror(
                "작업 실패",
                f"[CROSS] {furniture_name} 픽업 작업이 실패했습니다.\n"
                f"로그를 확인하여 원인을 파악해주세요."
            )
       
        self.total_orders += 1
        
        # 3초 후 상태 리셋
        self.root.after(3000, self.reset_robot_status)

    def move_to_manual_coordinates(self):
        """향상된 수동 좌표 이동 (안전한 로깅)"""
        if self.is_processing:
            messagebox.showwarning("경고", "현재 다른 작업을 처리 중입니다!")
            return
       
        try:
            x = safe_float_conversion(self.coord_entries['X'].get())
            y = safe_float_conversion(self.coord_entries['Y'].get())
            z = safe_float_conversion(self.coord_entries['Z'].get())
            r = safe_float_conversion(self.coord_entries['R'].get())
           
            position = [x, y, z, r]
           
            # 위치 유효성 검증
            if not validate_position(position):
                messagebox.showerror("오류", 
                    f"입력된 좌표가 작업 공간을 벗어났습니다!\n"
                    f"허용 범위:\n"
                    f"X: -400 ~ 400mm\n"
                    f"Y: -400 ~ 400mm\n"
                    f"Z: -200 ~ 200mm\n"
                    f"R: -180 ~ 180°")
                return
           
            self.log_display.add_message(f"[PIN] 수동 좌표 이동: {position}")
            self.update_robot_status("로봇 상태: 수동 이동 중", UI_COLORS['warning'])
           
            # 백그라운드에서 이동 실행
            move_thread = threading.Thread(
                target=self._manual_move_worker, 
                args=(position,), 
                daemon=True
            )
            move_thread.start()
           
        except Exception as e:
            self.logger.error(f"수동 이동 입력 처리 오류: {e}")
            messagebox.showerror("오류", f"좌표 입력 처리 중 오류가 발생했습니다:\n{str(e)}")

    def _manual_move_worker(self, position: List[float]):
        """백그라운드에서 수동 이동 실행 (안전한 로깅)"""
        try:
            self.robot_controller.move_to_position(position)
            self.log_display.add_message(f"[CHECK] 수동 이동 완료: {position}")
            self.root.after(0, lambda: self.update_robot_status("로봇 상태: 수동 이동 완료", UI_COLORS['success']))
            
        except (RobotMovementError, InvalidPositionError, TimeoutError) as e:
            self.log_display.add_message(f"[CROSS] 수동 이동 실패: {str(e)}")
            self.root.after(0, lambda: self.update_robot_status("로봇 상태: 이동 실패", UI_COLORS['error']))
            
        except Exception as e:
            self.logger.error(f"수동 이동 중 예상치 못한 오류: {e}")
            self.log_display.add_message(f"[CROSS] 수동 이동 오류: {str(e)}")
            self.root.after(0, lambda: self.update_robot_status("로봇 상태: 이동 오류", UI_COLORS['error']))
       
        # 3초 후 상태 리셋
        self.root.after(3000, self.reset_robot_status)

    def setup_ui(self):
        """완전한 UI 설정"""
        # 스타일 설정
        style = ttk.Style()
        style.theme_use('clam')
       
        # 메인 컨테이너
        main_container = tk.Frame(self.root, bg=UI_COLORS['primary_bg'])
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
       
        # 헤더 프레임
        self.setup_header(main_container)
       
        # 메인 콘텐츠 프레임 (3분할: 좌측-중앙-우측)
        content_frame = tk.Frame(main_container, bg=UI_COLORS['primary_bg'])
        content_frame.pack(fill=tk.BOTH, expand=True, pady=10)
       
        # 좌측 패널 (주문 및 상태)
        left_panel = tk.Frame(content_frame, bg=UI_COLORS['secondary_bg'], relief='raised', bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
       
        # 중앙 패널 (로봇팔 시각화)
        center_panel = tk.Frame(content_frame, bg=UI_COLORS['secondary_bg'], relief='raised', bd=2)
        center_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
       
        # 우측 패널 (카메라 및 YOLO)
        right_panel = tk.Frame(content_frame, bg=UI_COLORS['secondary_bg'], relief='raised', bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
       
        # UI 구성 요소들 설정
        self.setup_left_panel(left_panel)
        self.setup_center_panel(center_panel)
        self.setup_right_panel(right_panel)
       
        # 하단 상태바
        self.setup_status_bar(main_container)

    def setup_header(self, parent):
        """헤더 영역 설정"""
        header_frame = tk.Frame(parent, bg=UI_COLORS['secondary_bg'], height=80, relief='raised', bd=2)
        header_frame.pack(fill=tk.X, pady=(0, 10))
        header_frame.pack_propagate(False)
       
        # 제목
        title_text = "Enhanced Dobot Robot & YOLO Object Detection System"
        if not self.robot_controller.is_connected:
            title_text += " (Simulation Mode)"
           
        title_label = tk.Label(
            header_frame,
            text=title_text,
            font=(self.korean_font, 18, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary']
        )
        title_label.pack(side=tk.LEFT, padx=20, pady=20)
       
        # 연결 상태 표시
        connection_frame = tk.Frame(header_frame, bg=UI_COLORS['secondary_bg'])
        connection_frame.pack(side=tk.RIGHT, padx=20, pady=20)
       
        robot_status_text = "[GREEN] Robot Connected" if self.robot_controller.is_connected else "[RED] Simulation"
        robot_status_color = UI_COLORS['success'] if self.robot_controller.is_connected else UI_COLORS['error']
       
        self.robot_connection_label = tk.Label(
            connection_frame,
            text=robot_status_text,
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=robot_status_color
        )
        self.robot_connection_label.pack()
       
        yolo_status_text = "[GREEN] YOLO Active" if DEPENDENCIES['YOLO_AVAILABLE'] else "[RED] YOLO Inactive"
        yolo_status_color = UI_COLORS['success'] if DEPENDENCIES['YOLO_AVAILABLE'] else UI_COLORS['error']
       
        self.yolo_status_label = tk.Label(
            connection_frame,
            text=yolo_status_text,
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=yolo_status_color
        )
        self.yolo_status_label.pack()

    def setup_left_panel(self, parent):
        """좌측 패널 설정"""
        parent.configure(width=350)
        parent.pack_propagate(False)
       
        # 스크롤바를 위한 캔버스와 프레임
        canvas = tk.Canvas(parent, bg=UI_COLORS['secondary_bg'], highlightthickness=0, width=330)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=UI_COLORS['secondary_bg'])
       
        canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
       
        def configure_scroll_region(event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas_width = canvas.winfo_width()
            if canvas_width > 1:
                canvas.itemconfig(canvas_window, width=canvas_width)
       
        scrollable_frame.bind("<Configure>", configure_scroll_region)
        canvas.bind("<Configure>", lambda e: configure_scroll_region())
        canvas.configure(yscrollcommand=scrollbar.set)
       
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
       
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
       
        # 패널 제목
        panel_title = tk.Label(
            scrollable_frame,
            text="[GAMEPAD] Robot Control Panel",
            font=(self.korean_font, 16, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary']
        )
        panel_title.pack(pady=10)
       
        # 가구 주문 프레임
        order_frame = tk.LabelFrame(
            scrollable_frame,
            text=" [CHAIR] Furniture Selection & Pickup ",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=8
        )
        order_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
       
        # 가구 버튼들 - 2x2 그리드
        button_grid = tk.Frame(order_frame, bg=UI_COLORS['secondary_bg'])
        button_grid.pack()
       
        furniture_items = list(FURNITURE_INFO.items())
        for i, (item_name, info) in enumerate(furniture_items):
            row = i // 2
            col = i % 2
           
            btn_text = f"{info['emoji']} {item_name}\nPos: {info['position'][:2]}"
           
            btn = tk.Button(
                button_grid,
                text=btn_text,
                font=(self.korean_font, 9, 'bold'),
                bg=info['color'],
                fg='white',
                width=14,
                height=3,
                relief='raised',
                bd=2,
                activebackground=darken_color(info['color']),
                activeforeground='white',
                command=lambda item=item_name: self.execute_pickup_sequence(item)
            )
            btn.grid(row=row, column=col, padx=3, pady=3)
       
        # 수동 좌표 제어 프레임
        manual_frame = tk.LabelFrame(
            scrollable_frame,
            text=" [TARGET] Manual Coordinate Control ",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=8
        )
        manual_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
       
        # 좌표 입력 필드들
        coord_input_frame = tk.Frame(manual_frame, bg=UI_COLORS['secondary_bg'])
        coord_input_frame.pack(pady=5)
       
        # X, Y, Z, R 좌표
        coords = [('X', '0'), ('Y', '0'), ('Z', '0'), ('R', '0')]
       
        for i, (label, default) in enumerate(coords):
            tk.Label(coord_input_frame, text=f"{label}:",
                    font=(self.korean_font, 9), bg=UI_COLORS['secondary_bg'], fg=UI_COLORS['text_primary']).grid(row=i//2, column=(i%2)*2, padx=2, sticky='e')
            entry = tk.Entry(coord_input_frame, width=8, font=(self.korean_font, 9))
            entry.grid(row=i//2, column=(i%2)*2+1, padx=2)
            entry.insert(0, default)
            self.coord_entries[label] = entry
       
        # 수동 이동 버튼
        manual_move_btn = tk.Button(
            manual_frame,
            text="[PIN] Move to Coordinates",
            font=(self.korean_font, 10, 'bold'),
            bg=UI_COLORS['info'],
            fg='white',
            width=20,
            height=1,
            command=self.move_to_manual_coordinates
        )
        manual_move_btn.pack(pady=5)
       
        # 그리퍼 제어 프레임
        gripper_frame = tk.LabelFrame(
            scrollable_frame,
            text=" [PINCH] Gripper Control ",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=8
        )
        gripper_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
       
        gripper_btn_frame = tk.Frame(gripper_frame, bg=UI_COLORS['secondary_bg'])
        gripper_btn_frame.pack()
       
        self.gripper_on_btn = tk.Button(
            gripper_btn_frame,
            text="[GREEN] Gripper ON",
            font=(self.korean_font, 9, 'bold'),
            bg=UI_COLORS['success'],
            fg='white',
            width=12,
            height=2,
            command=lambda: self.robot_controller.control_gripper(True)
        )
        self.gripper_on_btn.grid(row=0, column=0, padx=2)
       
        self.gripper_off_btn = tk.Button(
            gripper_btn_frame,
            text="[RED] Gripper OFF",
            font=(self.korean_font, 9, 'bold'),
            bg=UI_COLORS['error'],
            fg='white',
            width=12,
            height=2,
            command=lambda: self.robot_controller.control_gripper(False)
        )
        self.gripper_off_btn.grid(row=0, column=1, padx=2)
       
        # YOLO 라벨 파일 로드 프레임
        yolo_frame = tk.LabelFrame(
            scrollable_frame,
            text=" [LABEL] YOLO Label Settings ",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=8
        )
        yolo_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
       
        self.load_labels_btn = tk.Button(
            yolo_frame,
            text="[FOLDER] Load Label File",
            font=(self.korean_font, 10, 'bold'),
            bg='#9b59b6',
            fg='white',
            width=20,
            height=2,
            command=self.load_yolo_labels
        )
        self.load_labels_btn.pack(pady=5)
       
        self.labels_status_label = tk.Label(
            yolo_frame,
            text="Using default COCO labels",
            font=(self.korean_font, 9),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_secondary']
        )
        self.labels_status_label.pack()
       
        # 시스템 제어 프레임
        system_frame = tk.LabelFrame(
            scrollable_frame,
            text=" [GEAR] System Control ",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=8
        )
        system_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
       
        # 카메라 제어 버튼
        self.camera_btn = tk.Button(
            system_frame,
            text="[CAMERA] Start Camera & YOLO",
            font=(self.korean_font, 10, 'bold'),
            bg=UI_COLORS['warning'],
            fg='white',
            width=22,
            height=2,
            command=self.toggle_camera
        )
        self.camera_btn.pack(fill=tk.X, pady=2)
       
        # 로봇 재연결 버튼
        self.reconnect_btn = tk.Button(
            system_frame,
            text="[PLUG] Reconnect Robot",
            font=(self.korean_font, 10, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg='white',
            width=22,
            height=2,
            command=self.reconnect_robot
        )
        self.reconnect_btn.pack(fill=tk.X, pady=2)
       
        # 전체 리셋 버튼
        self.reset_btn = tk.Button(
            system_frame,
            text="[REFRESH] Full Reset",
            font=(self.korean_font, 10, 'bold'),
            bg=UI_COLORS['error'],
            fg='white',
            width=22,
            height=2,
            command=self.reset_system
        )
        self.reset_btn.pack(fill=tk.X, pady=2)
       
        # 패널에 스크롤바 배치
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def setup_center_panel(self, parent):
        """중앙 패널 설정 (로봇팔 시각화)"""
        panel_title = tk.Label(
            parent,
            text="[ROBOT] Dobot Robot Arm Real-time Control",
            font=(self.korean_font, 18, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary']
        )
        panel_title.pack(pady=15)
       
        # 로봇팔 시각화 프레임
        visualization_frame = tk.Frame(parent, bg=UI_COLORS['primary_bg'], relief='sunken', bd=2)
        visualization_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(0, 15))
       
        # 로봇팔 시각화 초기화
        self.robot_visualizer = RobotArmVisualizer(visualization_frame)
       
        # 로봇팔 상태 정보
        robot_info_frame = tk.Frame(parent, bg=UI_COLORS['secondary_bg'])
        robot_info_frame.pack(fill=tk.X, padx=15, pady=(0, 15))
       
        self.robot_status_label = tk.Label(
            robot_info_frame,
            text="Robot Status: Standby",
            font=(self.korean_font, 12, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_secondary']
        )
        self.robot_status_label.pack()

    def setup_right_panel(self, parent):
        """우측 패널 설정 (카메라 및 YOLO)"""
        parent.configure(width=450)
       
        # YOLO 카메라 프레임
        camera_frame = tk.LabelFrame(
            parent,
            text=" [CAMERA] Real-time Camera & YOLO Object Detection ",
            font=(self.korean_font, 14, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=5,
            pady=5
        )
        camera_frame.pack(fill=tk.X, padx=15, pady=15)
        camera_frame.pack_propagate(False)
        camera_frame.configure(height=350)
       
        camera_container = tk.Frame(camera_frame, bg=UI_COLORS['primary_bg'])
        camera_container.pack(fill=tk.BOTH, expand=True)
        camera_container.pack_propagate(False)
       
        # 카메라 디스플레이 초기화
        self.camera_display = CameraDisplay(camera_container)
       
        # 객체 인식 결과 프레임
        detection_frame = tk.LabelFrame(
            parent,
            text=" [TARGET] Object Detection Results ",
            font=(self.korean_font, 14, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=10
        )
        detection_frame.pack(fill=tk.X, padx=15, pady=(0, 15))
       
        # 객체 인식 결과 디스플레이 초기화
        self.detection_display = DetectionDisplay(detection_frame, self.korean_font)
       
        # 로그 프레임
        log_frame = tk.LabelFrame(
            parent,
            text=" [CLIPBOARD] System Log ",
            font=(self.korean_font, 14, 'bold'),
            bg=UI_COLORS['secondary_bg'],
            fg=UI_COLORS['text_primary'],
            padx=10,
            pady=10
        )
        log_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(0, 15))
       
        # 로그 디스플레이 초기화
        self.log_display = LogDisplay(log_frame, self.korean_font)

    def setup_status_bar(self, parent):
        """하단 상태바 설정"""
        status_bar = tk.Frame(parent, bg=UI_COLORS['primary_bg'], height=30)
        status_bar.pack(fill=tk.X, pady=(10, 0))
        status_bar.pack_propagate(False)
       
        self.status_time_label = tk.Label(
            status_bar,
            text=f"Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            font=(self.korean_font, 10),
            bg=UI_COLORS['primary_bg'],
            fg=UI_COLORS['text_secondary']
        )
        self.status_time_label.pack(side=tk.LEFT, padx=10, pady=5)
       
        self.status_system_label = tk.Label(
            status_bar,
            text="System Status: Normal Operation",
            font=(self.korean_font, 10),
            bg=UI_COLORS['primary_bg'],
            fg=UI_COLORS['success']
        )
        self.status_system_label.pack(side=tk.RIGHT, padx=10, pady=5)

    def update_robot_status(self, text: str, color: str = None):
        """로봇 상태 업데이트"""
        if color is None:
            color = UI_COLORS['text_secondary']
        if hasattr(self, 'robot_status_label') and self.robot_status_label:
            self.robot_status_label.config(text=text, fg=color)

    def reset_robot_status(self):
        """로봇 상태 리셋"""
        self.current_order = None
        from config import RobotStatus
        self.robot_controller.status = RobotStatus.IDLE
        self.update_robot_status("로봇 상태: 대기 중")

    def reset_system(self):
        """향상된 전체 시스템 리셋"""
        if self.is_processing:
            result = messagebox.askyesno(
                "확인",
                "현재 작업 중입니다. 정말 리셋하시겠습니까?\n"
                "진행 중인 작업이 중단될 수 있습니다."
            )
            if not result:
                return
       
        try:
            # 카메라 정지
            if self.camera_active:
                self.stop_camera()
       
            # 로봇 안전 정지
            self.robot_controller.disconnect()
       
            # 상태 리셋
            self.is_processing = False
            self.current_order = None
            from config import RobotStatus
            self.robot_controller.status = RobotStatus.IDLE
       
            # 시각화 리셋
            if hasattr(self, 'robot_visualizer'):
                self.robot_visualizer.reset_simulation()
       
            self.log_display.add_message("[REFRESH] 시스템이 완전히 리셋되었습니다.")
            self.logger.info("시스템 전체 리셋 완료")
            
            # 로봇 재연결 시도
            self.root.after(1000, self.connect_robot)
       
            self.update_robot_status("로봇 상태: 리셋 완료")
            messagebox.showinfo("리셋 완료", "[REFRESH] 시스템이 안전하게 리셋되었습니다!")
            
        except Exception as e:
            self.logger.error(f"시스템 리셋 중 오류: {e}")
            messagebox.showerror("리셋 오류", f"시스템 리셋 중 오류가 발생했습니다:\n{str(e)}")

    def load_yolo_labels(self):
        """YOLO 라벨 파일 로드"""
        if not DEPENDENCIES['YOLO_AVAILABLE']:
            messagebox.showwarning("Warning", "YOLO is not installed, cannot load label file.")
            return
       
        file_path = filedialog.askopenfilename(
            title="Select YOLO Label File",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
       
        if file_path:
            success = self.yolo_detector.load_custom_labels(file_path)
            if success:
                self.labels_status_label.config(
                    text=f"Custom labels loaded: {len(self.yolo_detector.custom_labels)} classes",
                    fg=UI_COLORS['success']
                )
                self.log_display.add_message(f"[FOLDER] YOLO label file loaded: {file_path}")
                self.log_display.add_message(f"[LABEL] Loaded classes: {len(self.yolo_detector.custom_labels)}")
            else:
                self.labels_status_label.config(
                    text="Label loading failed",
                    fg=UI_COLORS['error']
                )
                self.log_display.add_message(f"[CROSS] YOLO label file loading failed: {file_path}")

    def reconnect_robot(self):
        """로봇 재연결"""
        if self.is_processing:
            messagebox.showwarning("Warning", "Cannot reconnect while operation is in progress!")
            return
       
        self.log_display.add_message("[PLUG] Attempting robot reconnection...")
       
        # 기존 연결 해제
        self.robot_controller.disconnect()
       
        # 재연결 시도
        self.connect_robot()
       
        # 상태 업데이트
        if self.robot_controller.is_connected:
            self.robot_connection_label.config(
                text="[GREEN] Robot Connected",
                fg=UI_COLORS['success']
            )
            self.log_display.add_message("[CHECK] Robot reconnection successful!")
        else:
            self.robot_connection_label.config(
                text="[RED] Simulation",
                fg=UI_COLORS['error']
            )
            self.log_display.add_message("[CROSS] Robot reconnection failed, maintaining simulation mode")

    def toggle_camera(self):
        """카메라 및 YOLO 켜기/끄기"""
        if not DEPENDENCIES['CV2_AVAILABLE']:
            messagebox.showwarning("Warning", "OpenCV is not installed, camera function unavailable.")
            return
       
        if not self.camera_active:
            self.start_camera()
        else:
            self.stop_camera()

    def start_camera(self):
        """카메라 및 YOLO 시작"""
        if not DEPENDENCIES['CV2_AVAILABLE']:
            return
       
        try:
            self.cap = cv2.VideoCapture(0)  # 기본 카메라 사용
            if not self.cap.isOpened():
                # 다른 카메라 시도
                self.cap = cv2.VideoCapture(1)
           
            if self.cap.isOpened():
                self.camera_active = True
                self.camera_btn.config(
                    bg=UI_COLORS['error'],
                    text="[CAMERA] Stop Camera & YOLO"
                )
                self.log_display.add_message("[CAMERA] Camera and YOLO object detection activated.")
               
                if DEPENDENCIES['YOLO_AVAILABLE']:
                    self.log_display.add_message("[TARGET] YOLO object detection started.")
                else:
                    self.log_display.add_message("[WARNING] YOLO disabled, only camera display active.")
               
                self.update_camera()
            else:
                messagebox.showerror("Error", "No camera found.")
        except Exception as e:
            messagebox.showerror("Error", f"Camera start failed: {str(e)}")

    def update_camera(self):
        """카메라 화면 및 YOLO 객체인식 업데이트"""
        if not DEPENDENCIES['CV2_AVAILABLE'] or not DEPENDENCIES['PIL_AVAILABLE']:
            return
       
        if self.camera_active and self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                # YOLO 객체 인식 수행
                if DEPENDENCIES['YOLO_AVAILABLE'] and self.yolo_detector.model_loaded:
                    annotated_frame, detections = self.yolo_detector.detect_objects(frame)
                   
                    # 인식 결과 로깅
                    if detections:
                        detection_info = []
                        for detection in detections:
                            class_name = detection['class']
                            confidence = detection['confidence']
                            detection_info.append(f"{class_name}({confidence:.2f})")
                       
                        detection_summary = f"Detected objects: {', '.join(detection_info)}"
                        self.detection_display.add_detection(detection_summary)
                else:
                    annotated_frame = frame
               
                # 프레임 크기 조정 및 표시
                self.display_camera_frame(annotated_frame)
               
                # 다음 프레임 스케줄링
                self.root.after(33, self.update_camera)  # 약 30 FPS

    def display_camera_frame(self, frame):
        """카메라 프레임을 GUI에 표시"""
        # 원본 프레임 크기
        original_height, original_width = frame.shape[:2]
       
        # 라벨 크기 가져오기
        self.camera_display.camera_label.update_idletasks()
        label_width = self.camera_display.camera_label.winfo_width()
        label_height = self.camera_display.camera_label.winfo_height()
       
        # 최소 크기 보장
        if label_width < 100:
            label_width = 400
        if label_height < 100:
            label_height = 300
       
        # 비율 유지하며 리사이징
        width_ratio = label_width / original_width
        height_ratio = label_height / original_height
       
        if width_ratio < height_ratio:
            new_width = label_width
            new_height = int(original_height * width_ratio)
        else:
            new_height = label_height
            new_width = int(original_width * height_ratio)
       
        # 최소 크기 보장
        new_width = max(new_width, 320)
        new_height = max(new_height, 240)
       
        # 프레임 리사이징
        resized_frame = cv2.resize(frame, (new_width, new_height))
       
        # BGR to RGB 변환
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
       
        # PIL 이미지로 변환
        pil_image = Image.fromarray(rgb_frame)
       
        # PhotoImage로 변환
        photo = ImageTk.PhotoImage(pil_image)
       
        # 카메라 디스플레이 업데이트
        self.camera_display.update_frame(photo)

    def stop_camera(self):
        """카메라 및 YOLO 중지"""
        self.camera_active = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
       
        self.camera_display.show_off_message()
        self.camera_btn.config(
            bg=UI_COLORS['warning'],
            text="[CAMERA] Start Camera & YOLO"
        )
        self.log_display.add_message("[CAMERA] Camera and YOLO deactivated.")

    def show_welcome_message(self):
        """환영 메시지 표시 (인코딩 안전성 향상)"""
        # 이모지를 텍스트로 대체한 안전한 메시지들
        messages = [
            "[PARTY] 향상된 Dobot 로봇 & YOLO 시스템이 시작되었습니다!",
            "[ROBOT] 실제 Dobot 로봇이 연결되었습니다." if self.robot_controller.is_connected 
            else "[WARNING] 로봇 연결 실패, 시뮬레이션 모드로 실행됩니다.",
            "[TARGET] YOLOv8 객체 인식 시스템이 준비되었습니다." if DEPENDENCIES['YOLO_AVAILABLE'] 
            else "[WARNING] YOLO 라이브러리가 없어 객체 인식이 비활성화됩니다.",
            "[CLIPBOARD] 가구 버튼을 클릭하여 향상된 픽업 작업을 시작하세요.",
            "[TARGET] 새로운 로직: 베이스에서 [350, 0, 물건Z좌표, 회전값]으로 이동"
        ]
        
        for message in messages:
            self.log_display.add_message(message)

    def on_closing(self):
        """향상된 프로그램 종료 처리"""
        if self.is_processing:
            result = messagebox.askyesno(
                "종료 확인",
                "현재 작업 중입니다. 정말 종료하시겠습니까?\n"
                "진행 중인 작업이 중단될 수 있습니다."
            )
            if not result:
                return
       
        try:
            # 카메라 정리
            if self.camera_active:
                self.stop_camera()
       
            # 로봇 안전 정지
            self.robot_controller.disconnect()
            
            self.logger.info("프로그램이 안전하게 종료되었습니다.")
            
        except Exception as e:
            self.logger.error(f"프로그램 종료 중 오류: {e}")
        finally:
            self.root.destroy()

def main():
    """향상된 메인 함수 (인코딩 안전성 향상)"""
    try:
        # 로깅 초기화
        logger = system_logger
        
        # 안전한 로깅 메시지들
        safe_messages = [
            "=== Enhanced Dobot Robot & YOLO System Started ===",
            "System initialization complete, GUI starting",
            f"Dobot API Status: {'Available' if DEPENDENCIES['DOBOT_AVAILABLE'] else 'Unavailable'}",
            f"YOLO Status: {'Available' if DEPENDENCIES['YOLO_AVAILABLE'] else 'Unavailable'}",
            "New pickup logic: Base -> [350, 0, Object_Z_coord, Rotation]"
        ]
        
        for message in safe_messages:
            try:
                logger.info(message)
            except:
                print(f"Safe log: {message}")
        
        root = tk.Tk()
        app = FurnitureOrderSystem(root)
       
        # 종료 이벤트 핸들러 등록
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        
        root.mainloop()
       
    except Exception as e:
        error_msg = f"System startup error: {e}"
        print(error_msg)
        try:
            if 'logger' in locals():
                logger.error(error_msg)
        except:
            pass
        
        # 안전한 에러 메시지 박스
        try:
            messagebox.showerror("System Error", f"Program startup error:\n{str(e)}")
        except:
            print(f"Critical error: {e}")

if __name__ == "__main__":
    main()
