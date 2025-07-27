#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
현대적이고 경량화된 GUI 컴포넌트
YOLOv8 기능 제거, 좌표 동기화 개선, 직관적인 인터페이스
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from typing import Callable, List, Optional, Tuple
import threading
import cv2
import numpy as np
from PIL import Image, ImageTk
from config import AppConfig


class CoordinateFrame:
    """좌표 입력 및 표시 프레임"""
    
    def __init__(self, parent, app_instance):
        self.parent = parent
        self.app = app_instance
        self.vars = {}
        self.create_widgets()
        
    def create_widgets(self):
        """좌표 위젯 생성"""
        # 메인 프레임
        self.frame = ttk.LabelFrame(self.parent, text="로봇 제어", padding="10")
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 현재 위치 표시
        self.current_frame = ttk.Frame(self.frame)
        self.current_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(self.current_frame, text="현재 위치:", 
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        
        self.current_label = ttk.Label(self.current_frame, 
                                      text="X: 250.0, Y: 0.0, Z: 50.0, R: 0.0",
                                      foreground="blue")
        self.current_label.pack(side=tk.LEFT, padx=(10, 0))
        
        # 좌표 입력 그리드
        self.input_frame = ttk.Frame(self.frame)
        self.input_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 좌표 레이블 및 입력 필드
        coordinates = [
            ("X (mm)", "x", 250.0, AppConfig.X_MIN, AppConfig.X_MAX),
            ("Y (mm)", "y", 0.0, AppConfig.Y_MIN, AppConfig.Y_MAX),
            ("Z (mm)", "z", 50.0, AppConfig.Z_MIN, AppConfig.Z_MAX),
            ("R (deg)", "r", 0.0, -180, 180)
        ]
        
        for i, (label, var_name, default, min_val, max_val) in enumerate(coordinates):
            # 레이블
            ttk.Label(self.input_frame, text=label).grid(row=i, column=0, 
                                                        sticky=tk.W, pady=2)
            
            # 입력 필드
            var = tk.DoubleVar(value=default)
            self.vars[var_name] = var
            
            entry = ttk.Entry(self.input_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, padx=(5, 10), pady=2)
            
            # 범위 표시
            range_label = ttk.Label(self.input_frame, 
                                   text=f"({min_val} ~ {max_val})",
                                   foreground="gray", font=("Arial", 8))
            range_label.grid(row=i, column=2, sticky=tk.W, padx=5, pady=2)
            
            # 증감 버튼
            btn_frame = ttk.Frame(self.input_frame)
            btn_frame.grid(row=i, column=3, padx=5, pady=2)
            
            increment = 10.0 if var_name in ['x', 'y', 'z'] else 5.0
            
            ttk.Button(btn_frame, text="-", width=3,
                      command=lambda v=var, inc=increment: v.set(v.get() - inc)
                      ).pack(side=tk.LEFT)
            
            ttk.Button(btn_frame, text="+", width=3,
                      command=lambda v=var, inc=increment: v.set(v.get() + inc)
                      ).pack(side=tk.LEFT, padx=(2, 0))
        
        # 제어 버튼 프레임
        self.button_frame = ttk.Frame(self.frame)
        self.button_frame.pack(fill=tk.X, pady=10)
        
        # 이동 버튼
        ttk.Button(self.button_frame, text="이동", 
                  command=self.move_to_coordinates,
                  style="Accent.TButton").pack(side=tk.LEFT, padx=(0, 5))
        
        # 홈 버튼
        ttk.Button(self.button_frame, text="홈", 
                  command=self.go_home).pack(side=tk.LEFT, padx=5)
        
        # 현재 위치 동기화 버튼
        ttk.Button(self.button_frame, text="현재 위치 가져오기", 
                  command=self.sync_current_position).pack(side=tk.LEFT, padx=5)
        
        # 그리퍼 제어
        self.gripper_frame = ttk.Frame(self.frame)
        self.gripper_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(self.gripper_frame, text="그리퍼:", 
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        
        ttk.Button(self.gripper_frame, text="토글", 
                  command=self.app.toggle_gripper).pack(side=tk.LEFT, padx=(10, 0))
        
    def move_to_coordinates(self):
        """입력된 좌표로 이동"""
        try:
            x = self.vars['x'].get()
            y = self.vars['y'].get()
            z = self.vars['z'].get()
            r = self.vars['r'].get()
            
            self.app.move_to_position(x, y, z, r)
            
        except tk.TclError:
            messagebox.showerror("오류", "올바른 숫자를 입력하세요.")
    
    def go_home(self):
        """홈 위치로 이동 및 UI 업데이트"""
        self.app.go_home()
        # 홈 좌표로 UI 업데이트
        home_pos = AppConfig.HOME_POSITION
        self.update_input_fields(home_pos)
    
    def sync_current_position(self):
        """현재 위치를 입력 필드에 동기화"""
        current_pos = self.app.current_position
        self.update_input_fields(current_pos)
    
    def update_input_fields(self, position: List[float]):
        """입력 필드 업데이트"""
        coord_names = ['x', 'y', 'z', 'r']
        for i, name in enumerate(coord_names):
            if i < len(position):
                self.vars[name].set(position[i])
    
    def update_current_display(self, position: List[float]):
        """현재 위치 표시 업데이트"""
        x, y, z, r = position
        display_text = f"X: {x:.1f}, Y: {y:.1f}, Z: {z:.1f}, R: {r:.1f}"
        self.current_label.config(text=display_text)


class FurnitureFrame:
    """가구 픽업 프레임"""
    
    def __init__(self, parent, app_instance):
        self.parent = parent
        self.app = app_instance
        self.create_widgets()
        
    def create_widgets(self):
        """가구 제어 위젯 생성"""
        # 메인 프레임
        self.frame = ttk.LabelFrame(self.parent, text="가구 픽업", padding="10")
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 설명 라벨
        desc_label = ttk.Label(self.frame, 
                              text="가구를 선택하여 자동 픽업 시퀀스를 실행합니다.",
                              foreground="gray")
        desc_label.pack(pady=(0, 10))
        
        # 가구 버튼 그리드
        self.furniture_frame = ttk.Frame(self.frame)
        self.furniture_frame.pack(fill=tk.X)
        
        furniture_items = [
            ("소파", "sofa", "#FF6B6B"),
            ("의자", "chair", "#4ECDC4"),
            ("책상", "desk", "#45B7D1"),
            ("침대", "bed", "#96CEB4")
        ]
        
        for i, (name, key, color) in enumerate(furniture_items):
            btn = tk.Button(
                self.furniture_frame,
                text=name,
                bg=color,
                fg="white",
                font=("Arial", 12, "bold"),
                width=8,
                height=2,
                relief=tk.RAISED,
                bd=2,
                command=lambda k=key: self.pickup_furniture(k)
            )
            
            row = i // 2
            col = i % 2
            btn.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
        
        # 컬럼 가중치 설정
        self.furniture_frame.columnconfigure(0, weight=1)
        self.furniture_frame.columnconfigure(1, weight=1)
        
    def pickup_furniture(self, furniture_type: str):
        """가구 픽업 실행 (백그라운드)"""
        def pickup_task():
            self.app.pickup_furniture(furniture_type)
            
        threading.Thread(target=pickup_task, daemon=True).start()


class CameraFrame:
    """카메라 및 객체 감지 프레임"""
    
    def __init__(self, parent, app_instance):
        self.parent = parent
        self.app = app_instance
        self.current_image = None
        self.create_widgets()
        
    def create_widgets(self):
        """카메라 위젯 생성"""
        # 메인 프레임
        self.frame = ttk.LabelFrame(self.parent, text="카메라 및 객체 감지", padding="10")
        self.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 상단 제어 버튼
        self.control_frame = ttk.Frame(self.frame)
        self.control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 카메라 토글 버튼
        self.camera_btn = tk.Button(
            self.control_frame,
            text="카메라 시작",
            bg="#28A745",
            fg="white",
            font=("Arial", 10, "bold"),
            command=self.toggle_camera
        )
        self.camera_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # 상태 라벨
        self.status_label = ttk.Label(
            self.control_frame,
            text="카메라 비활성",
            foreground="gray"
        )
        self.status_label.pack(side=tk.LEFT)
        
        # 카메라 화면과 감지 결과를 위한 프레임
        content_frame = ttk.Frame(self.frame)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # 왼쪽: 카메라 화면
        camera_frame = ttk.LabelFrame(content_frame, text="카메라 화면", padding="5")
        camera_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # 카메라 이미지 라벨
        self.camera_label = ttk.Label(camera_frame, text="카메라가 비활성 상태입니다", 
                                     font=("Arial", 12), anchor=tk.CENTER)
        self.camera_label.pack(fill=tk.BOTH, expand=True)
        
        # 오른쪽: 감지 결과
        detection_frame = ttk.LabelFrame(content_frame, text="감지된 객체", padding="5")
        detection_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        detection_frame.configure(width=250)
        
        # 감지 결과 리스트박스
        self.detection_listbox = tk.Listbox(
            detection_frame,
            height=8,
            font=("Arial", 9),
            selectmode=tk.SINGLE
        )
        self.detection_listbox.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 객체 제어 버튼들
        button_frame = ttk.Frame(detection_frame)
        button_frame.pack(fill=tk.X)
        
        ttk.Button(button_frame, text="선택된 객체로 이동", 
                  command=self.move_to_selected).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(button_frame, text="선택된 객체 픽업", 
                  command=self.pickup_selected).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(button_frame, text="새로고침", 
                  command=self.refresh_detections).pack(fill=tk.X)
        
    def toggle_camera(self):
        """카메라 토글"""
        self.app.toggle_camera()
        
        # 버튼 상태 업데이트
        if self.app.camera_active:
            self.camera_btn.config(text="카메라 중지", bg="#DC3545")
            self.status_label.config(text="카메라 활성", foreground="green")
            # 카메라 화면 업데이트 시작
            self._update_camera_display()
        else:
            self.camera_btn.config(text="카메라 시작", bg="#28A745")
            self.status_label.config(text="카메라 비활성", foreground="gray")
            self.camera_label.config(image="", text="카메라가 비활성 상태입니다")
            self.current_image = None
    
    def _update_camera_display(self):
        """카메라 화면 업데이트"""
        if not self.app.camera_active or not self.app.yolo_system:
            return
        
        try:
            # 주석이 달린 프레임 가져오기
            annotated_frame = self.app.yolo_system.get_annotated_frame()
            
            if annotated_frame is not None:
                # OpenCV BGR을 RGB로 변환
                frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                
                # 이미지 크기 조정 (320x240)
                frame_resized = cv2.resize(frame_rgb, (320, 240))
                
                # PIL Image로 변환
                pil_image = Image.fromarray(frame_resized)
                
                # Tkinter PhotoImage로 변환
                tk_image = ImageTk.PhotoImage(pil_image)
                
                # 라벨 업데이트
                self.camera_label.config(image=tk_image, text="")
                self.camera_label.image = tk_image  # 참조 유지
                
        except Exception as e:
            print(f"카메라 표시 오류: {e}")
        
        # 100ms 후 다시 업데이트
        if self.app.camera_active:
            self.camera_label.after(100, self._update_camera_display)
    
    def update_detections(self, detections):
        """감지 결과 업데이트"""
        # 리스트박스 내용 지우기
        self.detection_listbox.delete(0, tk.END)
        
        # 감지된 객체들 추가
        for i, detection in enumerate(detections):
            korean_name = "객체"
            if hasattr(self.app, 'yolo_system') and self.app.yolo_system:
                korean_name = self.app.yolo_system.detector.class_names_korean.get(
                    detection.class_name, detection.class_name
                )
            
            confidence_percent = int(detection.confidence * 100)
            item_text = f"{korean_name} ({confidence_percent}%)"
            self.detection_listbox.insert(tk.END, item_text)
    
    def move_to_selected(self):
        """선택된 객체로 이동"""
        selection = self.detection_listbox.curselection()
        if not selection:
            messagebox.showwarning("경고", "이동할 객체를 선택하세요.")
            return
        
        object_index = selection[0]
        self.app.move_to_detected_object(object_index)
    
    def pickup_selected(self):
        """선택된 객체 픽업"""
        selection = self.detection_listbox.curselection()
        if not selection:
            messagebox.showwarning("경고", "픽업할 객체를 선택하세요.")
            return
        
        object_index = selection[0]
        self.app.pickup_detected_object(object_index)
    
    def refresh_detections(self):
        """감지 결과 새로고침"""
        if hasattr(self.app, 'detected_objects'):
            self.update_detections(self.app.detected_objects)


class ControlFrame:
    """시스템 제어 프레임"""
    
    def __init__(self, parent, app_instance):
        self.parent = parent
        self.app = app_instance
        self.create_widgets()
        
    def create_widgets(self):
        """제어 위젯 생성"""
        # 메인 프레임
        self.frame = ttk.LabelFrame(self.parent, text="시스템 제어", padding="10")
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 연결 제어
        self.connection_frame = ttk.Frame(self.frame)
        self.connection_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(self.connection_frame, text="로봇 연결", 
                  command=self.app.try_connect_robot).pack(side=tk.LEFT, padx=(0, 5))
        
        ttk.Button(self.connection_frame, text="시스템 리셋", 
                  command=self.app.reset_system).pack(side=tk.LEFT, padx=5)
        
        # 비상 제어
        self.emergency_frame = ttk.Frame(self.frame)
        self.emergency_frame.pack(fill=tk.X)
        
        emergency_btn = tk.Button(
            self.emergency_frame,
            text="비상 정지",
            bg="#FF4444",
            fg="white",
            font=("Arial", 12, "bold"),
            relief=tk.RAISED,
            bd=3,
            command=self.app.emergency_stop
        )
        emergency_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # 상태 정보
        self.info_label = ttk.Label(
            self.emergency_frame,
            text="시뮬레이션 모드에서는 모든 동작이 가상으로 실행됩니다.",
            foreground="gray",
            font=("Arial", 9)
        )
        self.info_label.pack(side=tk.LEFT)


class LogFrame:
    """로그 표시 프레임"""
    
    def __init__(self, parent):
        self.parent = parent
        self.create_widgets()
        
    def create_widgets(self):
        """로그 위젯 생성"""
        # 메인 프레임
        self.frame = ttk.LabelFrame(self.parent, text="시스템 로그", padding="10")
        self.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 로그 텍스트 위젯
        self.log_text = scrolledtext.ScrolledText(
            self.frame,
            height=10,
            width=50,
            font=("Consolas", 9),
            bg="#f8f8f8",
            fg="#333333"
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 초기 메시지
        self.add_log("INFO", "Dobot 로봇팔 제어 시스템 시작")
        self.add_log("INFO", "시뮬레이션 모드로 초기화됨")
        
    def add_log(self, level: str, message: str):
        """로그 메시지 추가"""
        import datetime
        
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # 최대 라인 수 제한 (메모리 절약)
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 1000:
            self.log_text.delete("1.0", "100.0")
    
    def clear_log(self):
        """로그 지우기"""
        self.log_text.delete("1.0", tk.END)


class ModernUI:
    """통합 모던 UI 클래스"""
    
    def __init__(self, root, app_instance):
        self.root = root
        self.app = app_instance
        self.setup_styles()
        
    def setup_styles(self):
        """모던 스타일 설정"""
        style = ttk.Style()
        
        # 사용 가능한 테마 중 모던한 것 선택
        available_themes = style.theme_names()
        if "clam" in available_themes:
            style.theme_use("clam")
        elif "alt" in available_themes:
            style.theme_use("alt")
            
        # 커스텀 스타일 정의
        style.configure("Accent.TButton", 
                       foreground="white", 
                       background="#007ACC",
                       font=("Arial", 10, "bold"))
        
        style.map("Accent.TButton",
                 background=[("active", "#005A9E")])
        
    def create_main_interface(self):
        """메인 인터페이스 생성"""
        # 메인 컨테이너
        main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 왼쪽 패널 (제어)
        left_frame = ttk.Frame(main_container)
        main_container.add(left_frame, weight=2)
        
        # 중앙 패널 (카메라)
        center_frame = ttk.Frame(main_container)
        main_container.add(center_frame, weight=3)
        
        # 오른쪽 패널 (로그)
        right_frame = ttk.Frame(main_container)
        main_container.add(right_frame, weight=2)
        
        # 왼쪽 패널 컴포넌트
        self.coordinate_frame = CoordinateFrame(left_frame, self.app)
        self.furniture_frame = FurnitureFrame(left_frame, self.app)
        self.control_frame = ControlFrame(left_frame, self.app)
        
        # 중앙 패널 컴포넌트 (카메라)
        self.camera_frame = CameraFrame(center_frame, self.app)
        
        # 오른쪽 패널 컴포넌트
        self.log_frame = LogFrame(right_frame)
        
        # 로그 핸들러 연결
        self._setup_log_handler()
        
    def _setup_log_handler(self):
        """로그 핸들러 설정"""
        import logging
        
        class GUILogHandler(logging.Handler):
            def __init__(self, log_frame):
                super().__init__()
                self.log_frame = log_frame
                
            def emit(self, record):
                try:
                    # UI 업데이트는 메인 스레드에서 실행
                    self.log_frame.frame.after(
                        0, 
                        lambda: self.log_frame.add_log(
                            record.levelname, 
                            record.getMessage()
                        )
                    )
                except Exception:
                    pass  # GUI 로그 실패 시 무시
        
        # 루트 로거에 GUI 핸들러 추가
        gui_handler = GUILogHandler(self.log_frame)
        gui_handler.setLevel(logging.INFO)
        
        # 포맷터 설정
        formatter = logging.Formatter('%(name)s: %(message)s')
        gui_handler.setFormatter(formatter)
        
        # 로거에 핸들러 추가
        logging.getLogger().addHandler(gui_handler)
        
    def update_coordinate_display(self, position: List[float]):
        """좌표 표시 업데이트"""
        if hasattr(self, 'coordinate_frame'):
            self.coordinate_frame.update_current_display(position)
    
    def update_detection_display(self, detections):
        """객체 감지 결과 업데이트"""
        if hasattr(self, 'camera_frame'):
            self.camera_frame.update_detections(detections)
    
    def show_message(self, title: str, message: str, msg_type: str = "info"):
        """메시지 표시"""
        if msg_type == "error":
            messagebox.showerror(title, message)
        elif msg_type == "warning":
            messagebox.showwarning(title, message)
        else:
            messagebox.showinfo(title, message)
    
    def confirm_action(self, title: str, message: str) -> bool:
        """확인 대화상자"""
        return messagebox.askyesno(title, message)
