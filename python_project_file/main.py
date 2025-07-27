import tkinter as tk
from tkinter import ttk, messagebox, filedialog, colorchooser
import threading
import time
import sys
import os
import json
import sqlite3
import csv
from datetime import datetime
import math
import random

# Dobot 라이브러리 import 시도
try:
    import DobotDllType as dType
    DOBOT_AVAILABLE = True
except ImportError:
    print("Dobot DLL을 찾을 수 없습니다. 시뮬레이션 모드로 실행됩니다.")
    DOBOT_AVAILABLE = False

# 선택적 라이브러리들
try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False

class DatabaseManager:
    """데이터베이스 관리 클래스"""
    def __init__(self, db_path="furniture_system.db"):
        self.db_path = db_path
        self.init_database()
    
    def init_database(self):
        """데이터베이스 초기화"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # 가구 정보 테이블
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS furniture_items (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                type TEXT NOT NULL,
                width REAL,
                height REAL,
                depth REAL,
                weight REAL,
                pickup_x REAL,
                pickup_y REAL,
                pickup_z REAL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # 작업 로그 테이블
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS work_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                furniture_id INTEGER,
                action TEXT,
                status TEXT,
                duration REAL,
                notes TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (furniture_id) REFERENCES furniture_items (id)
            )
        ''')
        
        # 시스템 설정 테이블
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS system_settings (
                key TEXT PRIMARY KEY,
                value TEXT,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def add_furniture(self, name, furniture_type, width, height, depth, weight, pickup_pos):
        """가구 정보 추가"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO furniture_items (name, type, width, height, depth, weight, pickup_x, pickup_y, pickup_z)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (name, furniture_type, width, height, depth, weight, pickup_pos[0], pickup_pos[1], pickup_pos[2]))
        conn.commit()
        furniture_id = cursor.lastrowid
        conn.close()
        return furniture_id
    
    def get_furniture_list(self):
        """가구 목록 조회"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT * FROM furniture_items ORDER BY created_at DESC')
        items = cursor.fetchall()
        conn.close()
        return items
    
    def log_work(self, furniture_id, action, status, duration=0, notes=""):
        """작업 로그 기록"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO work_logs (furniture_id, action, status, duration, notes)
            VALUES (?, ?, ?, ?, ?)
        ''', (furniture_id, action, status, duration, notes))
        conn.commit()
        conn.close()

class VisionSystem:
    """비전 시스템 시뮬레이션 클래스"""
    def __init__(self):
        self.camera_connected = False
        self.detected_objects = []
    
    def connect_camera(self):
        """카메라 연결 시뮬레이션"""
        if OPENCV_AVAILABLE:
            try:
                # 실제 카메라 연결 시도
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    self.camera_connected = True
                    return True
            except:
                pass
        
        # 시뮬레이션 모드
        self.camera_connected = True
        return True
    
    def scan_for_objects(self):
        """물체 스캔 시뮬레이션"""
        if not self.camera_connected:
            return []
        
        # 시뮬레이션된 감지 결과
        objects = [
            {"type": "의자", "confidence": 0.95, "position": (250, 100, 30), "size": (40, 45, 80)},
            {"type": "테이블", "confidence": 0.87, "position": (150, -50, 40), "size": (120, 80, 75)},
        ]
        
        self.detected_objects = objects
        return objects
    
    def get_object_position(self, object_type):
        """특정 물체의 위치 반환"""
        for obj in self.detected_objects:
            if obj["type"] == object_type:
                return obj["position"]
        return None

class MotionPlanner:
    """모션 플래닝 클래스"""
    def __init__(self):
        self.waypoints = []
        self.current_plan = []
    
    def plan_pickup_sequence(self, target_pos, approach_height=100):
        """픽업 시퀀스 계획"""
        x, y, z = target_pos
        
        sequence = [
            {"name": "접근", "pos": (x, y, approach_height, 0), "speed": 50, "gripper": False},
            {"name": "하강", "pos": (x, y, z, 0), "speed": 20, "gripper": False},
            {"name": "그립", "pos": (x, y, z, 0), "speed": 10, "gripper": True},
            {"name": "상승", "pos": (x, y, approach_height, 0), "speed": 30, "gripper": True},
            {"name": "운반", "pos": (0, 200, approach_height, 0), "speed": 50, "gripper": True},
            {"name": "배치", "pos": (0, 200, z, 0), "speed": 20, "gripper": True},
            {"name": "해제", "pos": (0, 200, z, 0), "speed": 10, "gripper": False},
            {"name": "복귀", "pos": (0, 200, approach_height, 0), "speed": 30, "gripper": False},
        ]
        
        self.current_plan = sequence
        return sequence

class DobotFurnitureSystem:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Dobot 가구 픽업 시스템 - Professional Edition")
        self.root.geometry("1400x900")
        self.root.state('zoomed') if sys.platform == "win32" else self.root.attributes('-zoomed', True)
        
        # 시스템 상태 변수들
        self.is_connected = False
        self.is_homed = False
        self.is_moving = False
        self.current_position = {"x": 0, "y": 0, "z": 0, "r": 0}
        self.target_position = {"x": 200, "y": 0, "z": 50, "r": 0}
        self.gripper_state = False
        self.emergency_stop_active = False
        
        # 시스템 설정
        self.settings = {
            "speed": 50,
            "acceleration": 50,
            "z_offset": 10,
            "approach_height": 100,
            "grip_force": 50,
            "auto_home": True,
            "safety_limits": True
        }
        
        # 하위 시스템들
        self.db_manager = DatabaseManager()
        self.vision_system = VisionSystem()
        self.motion_planner = MotionPlanner()
        
        # 작업 큐
        self.work_queue = []
        self.current_job = None
        
        # GUI 생성
        self.create_menu()
        self.create_widgets()
        self.load_settings()
        self.check_dobot_connection()
        
        # 시작 시 상태 업데이트
        self.update_status()
        
        # 종료 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def create_menu(self):
        """메뉴바 생성"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # 파일 메뉴
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="파일", menu=file_menu)
        file_menu.add_command(label="설정 저장", command=self.save_settings)
        file_menu.add_command(label="설정 불러오기", command=self.load_settings)
        file_menu.add_separator()
        file_menu.add_command(label="로그 내보내기", command=self.export_logs)
        file_menu.add_command(label="가구 데이터 내보내기", command=self.export_furniture_data)
        file_menu.add_separator()
        file_menu.add_command(label="종료", command=self.on_closing)
        
        # 로봇 메뉴
        robot_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="로봇", menu=robot_menu)
        robot_menu.add_command(label="연결", command=self.connect_dobot)
        robot_menu.add_command(label="연결 해제", command=self.disconnect_dobot)
        robot_menu.add_separator()
        robot_menu.add_command(label="홈 위치", command=self.move_to_home)
        robot_menu.add_command(label="캘리브레이션", command=self.calibrate_robot)
        robot_menu.add_command(label="비상 정지", command=self.emergency_stop)
        
        # 비전 메뉴
        vision_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="비전", menu=vision_menu)
        vision_menu.add_command(label="카메라 연결", command=self.connect_vision)
        vision_menu.add_command(label="객체 스캔", command=self.scan_objects)
        vision_menu.add_command(label="카메라 보기", command=self.show_camera_view)
        
        # 도구 메뉴
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="도구", menu=tools_menu)
        tools_menu.add_command(label="가구 관리", command=self.open_furniture_manager)
        tools_menu.add_command(label="작업 기록", command=self.open_work_history)
        tools_menu.add_command(label="시스템 설정", command=self.open_settings)
        tools_menu.add_separator()
        tools_menu.add_command(label="3D 시각화", command=self.open_3d_viewer)
        
        # 도움말 메뉴
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="도움말", menu=help_menu)
        help_menu.add_command(label="사용법", command=self.show_help)
        help_menu.add_command(label="정보", command=self.show_about)
        
    def create_widgets(self):
        """GUI 위젯들을 생성"""
        
        # 메인 패널드 윈도우
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 왼쪽 패널
        left_frame = ttk.Frame(main_paned, width=400)
        main_paned.add(left_frame, weight=1)
        
        # 오른쪽 패널
        right_frame = ttk.Frame(main_paned, width=1000)
        main_paned.add(right_frame, weight=2)
        
        # === 왼쪽 패널 구성 ===
        self.create_left_panel(left_frame)
        
        # === 오른쪽 패널 구성 ===
        self.create_right_panel(right_frame)
        
    def create_left_panel(self, parent):
        """왼쪽 패널 생성"""
        
        # 연결 상태 프레임
        connection_frame = ttk.LabelFrame(parent, text="시스템 상태", padding="10")
        connection_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 상태 표시
        status_frame = ttk.Frame(connection_frame)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(status_frame, text="로봇 상태:").grid(row=0, column=0, sticky=tk.W)
        self.robot_status_label = ttk.Label(status_frame, text="연결 안됨", foreground="red")
        self.robot_status_label.grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(status_frame, text="비전 상태:").grid(row=1, column=0, sticky=tk.W)
        self.vision_status_label = ttk.Label(status_frame, text="비활성", foreground="orange")
        self.vision_status_label.grid(row=1, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(status_frame, text="작업 상태:").grid(row=2, column=0, sticky=tk.W)
        self.work_status_label = ttk.Label(status_frame, text="대기", foreground="blue")
        self.work_status_label.grid(row=2, column=1, sticky=tk.W, padx=(10, 0))
        
        # 연결 버튼들
        button_frame = ttk.Frame(connection_frame)
        button_frame.pack(fill=tk.X)
        
        self.connect_btn = ttk.Button(button_frame, text="로봇 연결", command=self.connect_dobot)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.disconnect_btn = ttk.Button(button_frame, text="연결 해제", 
                                        command=self.disconnect_dobot, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        self.emergency_btn = ttk.Button(button_frame, text="비상정지", 
                                       command=self.emergency_stop, state=tk.DISABLED)
        self.emergency_btn.pack(side=tk.RIGHT)
        
        # 현재 위치 프레임
        position_frame = ttk.LabelFrame(parent, text="현재 위치", padding="10")
        position_frame.pack(fill=tk.X, pady=(0, 10))
        
        pos_grid = ttk.Frame(position_frame)
        pos_grid.pack(fill=tk.X)
        
        for i, axis in enumerate(['X', 'Y', 'Z', 'R']):
            ttk.Label(pos_grid, text=f"{axis}:").grid(row=i, column=0, sticky=tk.W, pady=2)
            label = ttk.Label(pos_grid, text="0.0", font=("Courier", 10))
            label.grid(row=i, column=1, sticky=tk.E, padx=(10, 0), pady=2)
            setattr(self, f"{axis.lower()}_current_label", label)
        
        # 수동 제어 프레임
        manual_frame = ttk.LabelFrame(parent, text="수동 제어", padding="10")
        manual_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 좌표 입력
        coord_frame = ttk.Frame(manual_frame)
        coord_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.coord_entries = {}
        for i, (axis, default) in enumerate([('X', '200'), ('Y', '0'), ('Z', '50'), ('R', '0')]):
            ttk.Label(coord_frame, text=f"{axis}:").grid(row=i//2, column=(i%2)*2, sticky=tk.W, pady=2)
            entry = ttk.Entry(coord_frame, width=8)
            entry.grid(row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=(5, 15), pady=2)
            entry.insert(0, default)
            self.coord_entries[axis.lower()] = entry
        
        # 움직임 버튼들
        move_frame = ttk.Frame(manual_frame)
        move_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(move_frame, text="이동", command=self.move_to_position).pack(side=tk.LEFT)
        ttk.Button(move_frame, text="홈", command=self.move_to_home).pack(side=tk.LEFT, padx=(5, 0))
        
        # 조그 버튼들
        jog_frame = ttk.LabelFrame(manual_frame, text="조그 제어", padding="5")
        jog_frame.pack(fill=tk.X)
        
        jog_step_frame = ttk.Frame(jog_frame)
        jog_step_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(jog_step_frame, text="이동량:").pack(side=tk.LEFT)
        self.jog_step_var = tk.StringVar(value="10")
        jog_step_combo = ttk.Combobox(jog_step_frame, textvariable=self.jog_step_var, 
                                     values=["1", "5", "10", "20", "50"], width=8)
        jog_step_combo.pack(side=tk.LEFT, padx=(5, 0))
        
        # XY 조그
        xy_jog_frame = ttk.Frame(jog_frame)
        xy_jog_frame.pack(pady=5)
        
        ttk.Button(xy_jog_frame, text="Y+", width=5, 
                  command=lambda: self.jog_move('y', 1)).grid(row=0, column=1)
        ttk.Button(xy_jog_frame, text="X-", width=5, 
                  command=lambda: self.jog_move('x', -1)).grid(row=1, column=0)
        ttk.Button(xy_jog_frame, text="X+", width=5, 
                  command=lambda: self.jog_move('x', 1)).grid(row=1, column=2)
        ttk.Button(xy_jog_frame, text="Y-", width=5, 
                  command=lambda: self.jog_move('y', -1)).grid(row=2, column=1)
        
        # Z 조그
        z_jog_frame = ttk.Frame(jog_frame)
        z_jog_frame.pack(pady=5)
        ttk.Button(z_jog_frame, text="Z+", command=lambda: self.jog_move('z', 1)).pack(side=tk.LEFT)
        ttk.Button(z_jog_frame, text="Z-", command=lambda: self.jog_move('z', -1)).pack(side=tk.LEFT, padx=(5, 0))
        
        # 그리퍼 제어
        gripper_frame = ttk.LabelFrame(parent, text="그리퍼 제어", padding="10")
        gripper_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.gripper_status_label = ttk.Label(gripper_frame, text="상태: 열림")
        self.gripper_status_label.pack(pady=(0, 5))
        
        gripper_btn_frame = ttk.Frame(gripper_frame)
        gripper_btn_frame.pack(fill=tk.X)
        
        ttk.Button(gripper_btn_frame, text="잡기", command=self.grip_object).pack(side=tk.LEFT)
        ttk.Button(gripper_btn_frame, text="놓기", command=self.release_object).pack(side=tk.LEFT, padx=(5, 0))
        
        # 속도/가속도 설정
        speed_frame = ttk.LabelFrame(parent, text="동작 설정", padding="10")
        speed_frame.pack(fill=tk.X)
        
        ttk.Label(speed_frame, text="속도:").grid(row=0, column=0, sticky=tk.W)
        self.speed_var = tk.IntVar(value=self.settings["speed"])
        speed_scale = ttk.Scale(speed_frame, from_=1, to=100, variable=self.speed_var, orient=tk.HORIZONTAL)
        speed_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(5, 0))
        self.speed_label = ttk.Label(speed_frame, text="50%")
        self.speed_label.grid(row=0, column=2, padx=(5, 0))
        
        ttk.Label(speed_frame, text="가속도:").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        self.accel_var = tk.IntVar(value=self.settings["acceleration"])
        accel_scale = ttk.Scale(speed_frame, from_=1, to=100, variable=self.accel_var, orient=tk.HORIZONTAL)
        accel_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=(5, 0), pady=(5, 0))
        self.accel_label = ttk.Label(speed_frame, text="50%")
        self.accel_label.grid(row=1, column=2, padx=(5, 0), pady=(5, 0))
        
        speed_frame.columnconfigure(1, weight=1)
        
        # 스케일 업데이트 바인딩
        speed_scale.bind("<Motion>", lambda e: self.speed_label.config(text=f"{self.speed_var.get()}%"))
        accel_scale.bind("<Motion>", lambda e: self.accel_label.config(text=f"{self.accel_var.get()}%"))
        
    def create_right_panel(self, parent):
        """오른쪽 패널 생성"""
        
        # 노트북 위젯 (탭)
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 가구 픽업 탭
        self.create_pickup_tab()
        
        # 작업 관리 탭
        self.create_job_management_tab()
        
        # 시스템 로그 탭
        self.create_log_tab()
        
        # 시각화 탭
        self.create_visualization_tab()
        
    def create_pickup_tab(self):
        """가구 픽업 탭 생성"""
        pickup_frame = ttk.Frame(self.notebook)
        self.notebook.add(pickup_frame, text="가구 픽업")
        
        # 가구 선택 프레임
        selection_frame = ttk.LabelFrame(pickup_frame, text="가구 선택", padding="10")
        selection_frame.pack(fill=tk.X, pady=(0, 10))
        
        select_grid = ttk.Frame(selection_frame)
        select_grid.pack(fill=tk.X)
        
        ttk.Label(select_grid, text="가구 종류:").grid(row=0, column=0, sticky=tk.W)
        self.furniture_var = tk.StringVar(value="의자")
        furniture_combo = ttk.Combobox(select_grid, textvariable=self.furniture_var, 
                                      values=["의자", "테이블", "책상", "서랍", "소파", "책장"])
        furniture_combo.grid(row=0, column=1, sticky=tk.W, padx=(10, 20))
        
        ttk.Button(select_grid, text="비전 스캔", command=self.scan_objects).grid(row=0, column=2, padx=10)
        ttk.Button(select_grid, text="수동 입력", command=self.manual_position_input).grid(row=0, column=3, padx=10)
        
        # 감지된 객체 목록
        detected_frame = ttk.LabelFrame(pickup_frame, text="감지된 객체", padding="10")
        detected_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 트리뷰로 객체 목록 표시
        columns = ("type", "confidence", "position", "size")
        self.objects_tree = ttk.Treeview(detected_frame, columns=columns, show="headings", height=6)
        
        self.objects_tree.heading("type", text="종류")
        self.objects_tree.heading("confidence", text="신뢰도")
        self.objects_tree.heading("position", text="위치 (X, Y, Z)")
        self.objects_tree.heading("size", text="크기 (W, H, D)")
        
        self.objects_tree.column("type", width=80)
        self.objects_tree.column("confidence", width=80)
        self.objects_tree.column("position", width=150)
        self.objects_tree.column("size", width=150)
        
        objects_scrollbar = ttk.Scrollbar(detected_frame, orient="vertical", command=self.objects_tree.yview)
        self.objects_tree.configure(yscrollcommand=objects_scrollbar.set)
        
        self.objects_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        objects_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 픽업 제어 프레임
        pickup_control_frame = ttk.LabelFrame(pickup_frame, text="픽업 제어", padding="10")
        pickup_control_frame.pack(fill=tk.X)
        
        control_grid = ttk.Frame(pickup_control_frame)
        control_grid.pack(fill=tk.X)
        
        ttk.Button(control_grid, text="픽업 시퀀스 계획", 
                  command=self.plan_pickup_sequence).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(control_grid, text="픽업 실행", 
                  command=self.execute_pickup).grid(row=0, column=1, padx=10)
        ttk.Button(control_grid, text="중지", 
                  command=self.stop_operation).grid(row=0, column=2, padx=10)
        
        # 진행률 표시
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(pickup_control_frame, variable=self.progress_var, 
                                           maximum=100, length=300)
        self.progress_bar.pack(pady=(10, 0))
        
        self.progress_label = ttk.Label(pickup_control_frame, text="대기 중...")
        self.progress_label.pack(pady=(5, 0))
        
    def create_job_management_tab(self):
        """작업 관리 탭 생성"""
        job_frame = ttk.Frame(self.notebook)
        self.notebook.add(job_frame, text="작업 관리")
        
        # 작업 큐 프레임
        queue_frame = ttk.LabelFrame(job_frame, text="작업 큐", padding="10")
        queue_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 작업 큐 트리뷰
        queue_columns = ("id", "type", "target", "status", "priority", "created")
        self.queue_tree = ttk.Treeview(queue_frame, columns=queue_columns, show="headings", height=8)
        
        for col in queue_columns:
            self.queue_tree.heading(col, text=col.title())
            self.queue_tree.column(col, width=100)
        
        queue_scrollbar = ttk.Scrollbar(queue_frame, orient="vertical", command=self.queue_tree.yview)
        self.queue_tree.configure(yscrollcommand=queue_scrollbar.set)
        
        self.queue_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        queue_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 작업 제어 버튼들
        job_control_frame = ttk.Frame(job_frame)
        job_control_frame.pack(fill=tk.X)
        
        ttk.Button(job_control_frame, text="작업 추가", 
                  command=self.add_job_dialog).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(job_control_frame, text="작업 편집", 
                  command=self.edit_job_dialog).pack(side=tk.LEFT, padx=10)
        ttk.Button(job_control_frame, text="작업 삭제", 
                  command=self.delete_job).pack(side=tk.LEFT, padx=10)
        ttk.Button(job_control_frame, text="큐 실행", 
                  command=self.execute_queue).pack(side=tk.RIGHT, padx=10)
        ttk.Button(job_control_frame, text="큐 정지", 
                  command=self.stop_queue).pack(side=tk.RIGHT)
        
    def create_log_tab(self):
        """시스템 로그 탭 생성"""
        log_frame = ttk.Frame(self.notebook)
        self.notebook.add(log_frame, text="시스템 로그")
        
        # 로그 필터 프레임
        filter_frame = ttk.Frame(log_frame)
        filter_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(filter_frame, text="레벨:").pack(side=tk.LEFT)
        self.log_level_var = tk.StringVar(value="ALL")
        log_level_combo = ttk.Combobox(filter_frame, textvariable=self.log_level_var,
                                      values=["ALL", "INFO", "WARNING", "ERROR"], width=10)
        log_level_combo.pack(side=tk.LEFT, padx=(5, 20))
        
        ttk.Button(filter_frame, text="로그 지우기", 
                  command=self.clear_logs).pack(side=tk.RIGHT, padx=(10, 0))
        ttk.Button(filter_frame, text="로그 저장", 
                  command=self.save_logs).pack(side=tk.RIGHT)
        
        # 로그 텍스트 위젯
        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(log_text_frame, wrap=tk.WORD, font=("Courier", 9))
        log_scrollbar_y = ttk.Scrollbar(log_text_frame, orient="vertical", command=self.log_text.yview)
        log_scrollbar_x = ttk.Scrollbar(log_text_frame, orient="horizontal", command=self.log_text.xview)
        
        self.log_text.configure(yscrollcommand=log_scrollbar_y.set, xscrollcommand=log_scrollbar_x.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_scrollbar_y.grid(row=0, column=1, sticky=(tk.N, tk.S))
        log_scrollbar_x.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        log_text_frame.columnconfigure(0, weight=1)
        log_text_frame.rowconfigure(0, weight=1)
        
        # 로그 태그 설정 (색상)
        self.log_text.tag_config("INFO", foreground="black")
        self.log_text.tag_config("WARNING", foreground="orange")
        self.log_text.tag_config("ERROR", foreground="red")
        self.log_text.tag_config("SUCCESS", foreground="green")
        
    def create_visualization_tab(self):
        """시각화 탭 생성"""
        viz_frame = ttk.Frame(self.notebook)
        self.notebook.add(viz_frame, text="시각화")
        
        if MATPLOTLIB_AVAILABLE:
            # 플롯 프레임
            plot_frame = ttk.LabelFrame(viz_frame, text="로봇 위치 트래킹", padding="10")
            plot_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
            
            # Matplotlib 그래프
            self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
            self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # 데이터 저장용
            self.position_history = {'time': [], 'x': [], 'y': [], 'z': []}
            
            # 3D 궤적 플롯 초기화
            self.ax1.set_title("XY 평면 궤적")
            self.ax1.set_xlabel("X (mm)")
            self.ax1.set_ylabel("Y (mm)")
            self.ax1.grid(True)
            
            self.ax2.set_title("시간별 Z 위치")
            self.ax2.set_xlabel("시간")
            self.ax2.set_ylabel("Z (mm)")
            self.ax2.grid(True)
            
        else:
            # Matplotlib가 없을 때
            no_plot_frame = ttk.LabelFrame(viz_frame, text="시각화", padding="20")
            no_plot_frame.pack(fill=tk.BOTH, expand=True)
            ttk.Label(no_plot_frame, text="Matplotlib가 설치되지 않았습니다.\npip install matplotlib", 
                     justify=tk.CENTER).pack(expand=True)
        
        # 통계 프레임
        stats_frame = ttk.LabelFrame(viz_frame, text="작업 통계", padding="10")
        stats_frame.pack(fill=tk.X)
        
        stats_grid = ttk.Frame(stats_frame)
        stats_grid.pack(fill=tk.X)
        
        self.stats_labels = {}
        stats_items = [
            ("총 작업 수", "0"), ("성공률", "0%"), 
            ("평균 시간", "0.0s"), ("오늘 작업", "0")
        ]
        
        for i, (label, value) in enumerate(stats_items):
            ttk.Label(stats_grid, text=f"{label}:").grid(row=i//2, column=(i%2)*2, sticky=tk.W, pady=2)
            stat_label = ttk.Label(stats_grid, text=value, font=("Arial", 10, "bold"))
            stat_label.grid(row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=(10, 30), pady=2)
            self.stats_labels[label] = stat_label
        
    def log_message(self, message, level="INFO"):
        """로그 메시지를 추가"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] [{level}] {message}\n"
        
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, log_entry, level)
            self.log_text.see(tk.END)
            
        print(f"{level}: {message}")  # 콘솔에도 출력
        
    def update_status(self):
        """상태 업데이트"""
        if hasattr(self, 'root'):
            # 로봇 상태
            if self.is_connected:
                if self.emergency_stop_active:
                    self.robot_status_label.config(text="비상정지", foreground="red")
                elif self.is_moving:
                    self.robot_status_label.config(text="이동중", foreground="orange")
                else:
                    self.robot_status_label.config(text="연결됨", foreground="green")
            else:
                self.robot_status_label.config(text="연결안됨", foreground="red")
            
            # 위치 업데이트
            if hasattr(self, 'x_current_label'):
                self.x_current_label.config(text=f"{self.current_position['x']:.1f}")
                self.y_current_label.config(text=f"{self.current_position['y']:.1f}")
                self.z_current_label.config(text=f"{self.current_position['z']:.1f}")
                self.r_current_label.config(text=f"{self.current_position['r']:.1f}")
            
            # 그리퍼 상태
            if hasattr(self, 'gripper_status_label'):
                status = "닫힘" if self.gripper_state else "열림"
                self.gripper_status_label.config(text=f"상태: {status}")
            
            # 위치 이력 업데이트 (시각화용)
            if MATPLOTLIB_AVAILABLE and hasattr(self, 'position_history'):
                current_time = time.time()
                self.position_history['time'].append(current_time)
                self.position_history['x'].append(self.current_position['x'])
                self.position_history['y'].append(self.current_position['y'])
                self.position_history['z'].append(self.current_position['z'])
                
                # 최근 100개 포인트만 유지
                if len(self.position_history['time']) > 100:
                    for key in self.position_history:
                        self.position_history[key] = self.position_history[key][-100:]
                
                self.update_plots()
            
            # 주기적 업데이트
            self.root.after(1000, self.update_status)
    
    def update_plots(self):
        """플롯 업데이트"""
        if not MATPLOTLIB_AVAILABLE or not hasattr(self, 'ax1'):
            return
            
        try:
            # XY 궤적 플롯
            self.ax1.clear()
            self.ax1.plot(self.position_history['x'], self.position_history['y'], 'b-', alpha=0.7)
            if self.position_history['x']:
                self.ax1.plot(self.position_history['x'][-1], self.position_history['y'][-1], 'ro', markersize=8)
            self.ax1.set_title("XY 평면 궤적")
            self.ax1.set_xlabel("X (mm)")
            self.ax1.set_ylabel("Y (mm)")
            self.ax1.grid(True)
            
            # Z 위치 플롯
            self.ax2.clear()
            if self.position_history['time']:
                times = [(t - self.position_history['time'][0]) for t in self.position_history['time']]
                self.ax2.plot(times, self.position_history['z'], 'g-', alpha=0.7)
            self.ax2.set_title("시간별 Z 위치")
            self.ax2.set_xlabel("시간 (초)")
            self.ax2.set_ylabel("Z (mm)")
            self.ax2.grid(True)
            
            self.canvas.draw()
        except Exception as e:
            pass  # 플롯 오류 무시
    
    def connect_dobot(self):
        """Dobot에 연결"""
        if DOBOT_AVAILABLE:
            try:
                state = dType.ConnectDobot(port="", baudrate=115200)
                if state == dType.DobotConnect.DobotConnect_NoError:
                    self.is_connected = True
                    self.connect_btn.config(state=tk.DISABLED)
                    self.disconnect_btn.config(state=tk.NORMAL)
                    self.emergency_btn.config(state=tk.NORMAL)
                    self.log_message("Dobot에 성공적으로 연결되었습니다.", "SUCCESS")
                    
                    # 자동 홈 이동
                    if self.settings.get("auto_home", True):
                        self.move_to_home()
                    return True
                else:
                    self.log_message("Dobot 연결에 실패했습니다.", "ERROR")
                    return False
            except Exception as e:
                self.log_message(f"연결 중 오류 발생: {str(e)}", "ERROR")
                return False
        else:
            # 시뮬레이션 모드
            self.is_connected = True
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.emergency_btn.config(state=tk.NORMAL)
            self.log_message("시뮬레이션 모드로 연결되었습니다.", "INFO")
            return True
    
    def disconnect_dobot(self):
        """Dobot 연결 해제"""
        if DOBOT_AVAILABLE and self.is_connected:
            try:
                dType.DisconnectDobot()
            except:
                pass
        
        self.is_connected = False
        self.is_homed = False
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.emergency_btn.config(state=tk.DISABLED)
        self.log_message("Dobot 연결이 해제되었습니다.", "INFO")
    
    def check_dobot_connection(self):
        """Dobot 연결 상태 확인"""
        if DOBOT_AVAILABLE:
            self.log_message("Dobot DLL을 찾았습니다.", "INFO")
        else:
            self.log_message("Dobot DLL을 찾을 수 없습니다. 시뮬레이션 모드로 실행됩니다.", "WARNING")
    
    def move_to_position(self):
        """지정된 좌표로 이동"""
        if not self.is_connected:
            messagebox.showwarning("경고", "먼저 Dobot에 연결해주세요.")
            return
        
        try:
            x = float(self.coord_entries['x'].get())
            y = float(self.coord_entries['y'].get())
            z = float(self.coord_entries['z'].get())
            r = float(self.coord_entries['r'].get())
            
            # 안전 제한 확인
            if self.settings.get("safety_limits", True):
                if not self.check_safety_limits(x, y, z):
                    return
            
            if DOBOT_AVAILABLE:
                # 속도 설정
                speed = self.speed_var.get()
                accel = self.accel_var.get()
                dType.SetPTPCoordinateParams(speed, speed, speed, speed, accel, accel, accel, accel, isQueued=0)
                
                # 이동 명령
                dType.SetPTPCmd(dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued=1)
            
            # 위치 업데이트 (시뮬레이션)
            self.target_position = {"x": x, "y": y, "z": z, "r": r}
            self.is_moving = True
            
            # 시뮬레이션된 이동
            self.simulate_movement(x, y, z, r)
            
            self.log_message(f"목표 위치로 이동: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, R={r:.1f}", "INFO")
            
        except ValueError:
            messagebox.showerror("오류", "올바른 숫자를 입력해주세요.")
            self.log_message("잘못된 좌표 입력", "ERROR")
    
    def simulate_movement(self, target_x, target_y, target_z, target_r):
        """이동 시뮬레이션"""
        def move():
            steps = 20
            start_pos = self.current_position.copy()
            
            for i in range(steps + 1):
                progress = i / steps
                
                self.current_position['x'] = start_pos['x'] + (target_x - start_pos['x']) * progress
                self.current_position['y'] = start_pos['y'] + (target_y - start_pos['y']) * progress
                self.current_position['z'] = start_pos['z'] + (target_z - start_pos['z']) * progress
                self.current_position['r'] = start_pos['r'] + (target_r - start_pos['r']) * progress
                
                time.sleep(0.1)
            
            self.is_moving = False
            
        threading.Thread(target=move, daemon=True).start()
    
    def check_safety_limits(self, x, y, z):
        """안전 제한 확인"""
        # 일반적인 Dobot Magician 작업 영역
        if not (-300 <= x <= 300 and -300 <= y <= 300 and -10 <= z <= 300):
            messagebox.showerror("안전 오류", "작업 영역을 벗어난 좌표입니다.")
            self.log_message(f"안전 제한 위반: X={x}, Y={y}, Z={z}", "ERROR")
            return False
        return True
    
    def jog_move(self, axis, direction):
        """조그 이동"""
        if not self.is_connected:
            return
        
        step = float(self.jog_step_var.get()) * direction
        current = self.current_position[axis]
        
        if axis == 'x':
            self.coord_entries['x'].delete(0, tk.END)
            self.coord_entries['x'].insert(0, str(current + step))
        elif axis == 'y':
            self.coord_entries['y'].delete(0, tk.END)
            self.coord_entries['y'].insert(0, str(current + step))
        elif axis == 'z':
            self.coord_entries['z'].delete(0, tk.END)
            self.coord_entries['z'].insert(0, str(current + step))
        
        self.move_to_position()
    
    def move_to_home(self):
        """홈 위치로 이동"""
        if not self.is_connected:
            messagebox.showwarning("경고", "먼저 Dobot에 연결해주세요.")
            return
        
        if DOBOT_AVAILABLE:
            dType.SetHOMECmd()
        
        # 홈 위치 시뮬레이션
        self.simulate_movement(0, 200, 100, 0)
        self.is_homed = True
        self.log_message("홈 위치로 이동 중...", "INFO")
    
    def grip_object(self):
        """물체 잡기"""
        if not self.is_connected:
            messagebox.showwarning("경고", "먼저 Dobot에 연결해주세요.")
            return
        
        if DOBOT_AVAILABLE:
            dType.SetEndEffectorGripper(enable=True, grip=True, isQueued=1)
        
        self.gripper_state = True
        self.log_message("그리퍼가 물체를 잡았습니다.", "INFO")
    
    def release_object(self):
        """물체 놓기"""
        if not self.is_connected:
            messagebox.showwarning("경고", "먼저 Dobot에 연결해주세요.")
            return
        
        if DOBOT_AVAILABLE:
            dType.SetEndEffectorGripper(enable=True, grip=False, isQueued=1)
        
        self.gripper_state = False
        self.log_message("그리퍼가 물체를 놓았습니다.", "INFO")
    
    def emergency_stop(self):
        """비상 정지"""
        if DOBOT_AVAILABLE and self.is_connected:
            try:
                dType.SetQueuedCmdClear()
            except:
                pass
        
        self.emergency_stop_active = True
        self.is_moving = False
        self.log_message("비상 정지가 실행되었습니다!", "ERROR")
        messagebox.showwarning("비상 정지", "모든 동작이 정지되었습니다.\n시스템을 재시작하려면 연결을 해제하고 다시 연결하세요.")
    
    def connect_vision(self):
        """비전 시스템 연결"""
        if self.vision_system.connect_camera():
            self.vision_status_label.config(text="활성", foreground="green")
            self.log_message("비전 시스템이 연결되었습니다.", "SUCCESS")
        else:
            self.log_message("비전 시스템 연결에 실패했습니다.", "ERROR")
    
    def scan_objects(self):
        """물체 스캔"""
        if not hasattr(self, 'objects_tree'):
            return
        
        # 기존 항목 삭제
        for item in self.objects_tree.get_children():
            self.objects_tree.delete(item)
        
        self.log_message("물체 스캔을 시작합니다...", "INFO")
        
        # 스캔 시뮬레이션
        def scan_simulation():
            time.sleep(2)  # 스캔 시간 시뮬레이션
            
            objects = self.vision_system.scan_for_objects()
            
            # 트리뷰에 결과 추가
            for obj in objects:
                pos_str = f"({obj['position'][0]:.1f}, {obj['position'][1]:.1f}, {obj['position'][2]:.1f})"
                size_str = f"({obj['size'][0]:.1f}, {obj['size'][1]:.1f}, {obj['size'][2]:.1f})"
                
                self.objects_tree.insert("", tk.END, values=(
                    obj['type'], 
                    f"{obj['confidence']:.2f}",
                    pos_str,
                    size_str
                ))
            
            self.log_message(f"스캔 완료: {len(objects)}개 객체 발견", "SUCCESS")
        
        threading.Thread(target=scan_simulation, daemon=True).start()
    
    def plan_pickup_sequence(self):
        """픽업 시퀀스 계획"""
        selected_items = self.objects_tree.selection()
        if not selected_items:
            messagebox.showwarning("경고", "픽업할 객체를 선택해주세요.")
            return
        
        # 선택된 객체 정보 가져오기
        item = self.objects_tree.item(selected_items[0])
        values = item['values']
        
        # 위치 파싱
        pos_str = values[2].strip("()")
        x, y, z = map(float, pos_str.split(", "))
        
        # 모션 플래닝
        sequence = self.motion_planner.plan_pickup_sequence((x, y, z), self.settings["approach_height"])
        
        self.log_message(f"픽업 시퀀스 계획 완료: {len(sequence)} 단계", "INFO")
        for i, step in enumerate(sequence):
            self.log_message(f"  {i+1}. {step['name']}: {step['pos']}", "INFO")
    
    def execute_pickup(self):
        """픽업 실행"""
        if not self.motion_planner.current_plan:
            messagebox.showwarning("경고", "먼저 픽업 시퀀스를 계획해주세요.")
            return
        
        if not self.is_connected:
            messagebox.showwarning("경고", "먼저 Dobot에 연결해주세요.")
            return
        
        self.log_message("픽업 작업을 시작합니다...", "INFO")
        
        def execute_sequence():
            plan = self.motion_planner.current_plan
            total_steps = len(plan)
            
            for i, step in enumerate(plan):
                if self.emergency_stop_active:
                    break
                
                # 진행률 업데이트
                progress = (i / total_steps) * 100
                self.progress_var.set(progress)
                self.progress_label.config(text=f"{step['name']} 중...")
                
                # 이동
                x, y, z, r = step['pos']
                self.simulate_movement(x, y, z, r)
                
                # 그리퍼 제어
                if step['gripper'] and not self.gripper_state:
                    time.sleep(1)
                    self.grip_object()
                elif not step['gripper'] and self.gripper_state:
                    time.sleep(1)
                    self.release_object()
                
                time.sleep(2)  # 단계별 대기
                
                self.log_message(f"단계 {i+1} 완료: {step['name']}", "INFO")
            
            self.progress_var.set(100)
            self.progress_label.config(text="픽업 완료!")
            self.log_message("픽업 작업이 완료되었습니다.", "SUCCESS")
        
        threading.Thread(target=execute_sequence, daemon=True).start()
    
    def stop_operation(self):
        """작업 중지"""
        self.emergency_stop()
        self.progress_var.set(0)
        self.progress_label.config(text="작업 중지됨")
    
    # 추가 GUI 메서드들
    def manual_position_input(self):
        """수동 위치 입력 대화상자"""
        dialog = tk.Toplevel(self.root)
        dialog.title("수동 위치 입력")
        dialog.geometry("300x200")
        dialog.transient(self.root)
        dialog.grab_set()
        
        ttk.Label(dialog, text="객체 위치를 입력하세요:").pack(pady=10)
        
        entries = {}
        for axis in ['X', 'Y', 'Z']:
            frame = ttk.Frame(dialog)
            frame.pack(fill=tk.X, padx=20, pady=5)
            ttk.Label(frame, text=f"{axis}:").pack(side=tk.LEFT)
            entry = ttk.Entry(frame)
            entry.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(10, 0))
            entries[axis] = entry
        
        def add_manual_object():
            try:
                x = float(entries['X'].get())
                y = float(entries['Y'].get())
                z = float(entries['Z'].get())
                
                # 트리뷰에 추가
                pos_str = f"({x:.1f}, {y:.1f}, {z:.1f})"
                self.objects_tree.insert("", tk.END, values=(
                    "수동입력", "1.00", pos_str, "(수동)"
                ))
                
                dialog.destroy()
                self.log_message(f"수동 객체 추가: {pos_str}", "INFO")
                
            except ValueError:
                messagebox.showerror("오류", "올바른 숫자를 입력해주세요.")
        
        button_frame = ttk.Frame(dialog)
        button_frame.pack(fill=tk.X, pady=20)
        ttk.Button(button_frame, text="추가", command=add_manual_object).pack(side=tk.RIGHT, padx=(5, 20))
        ttk.Button(button_frame, text="취소", command=dialog.destroy).pack(side=tk.RIGHT)
    
    def add_job_dialog(self):
        """작업 추가 대화상자"""
        messagebox.showinfo("정보", "작업 추가 기능은 구현 예정입니다.")
    
    def edit_job_dialog(self):
        """작업 편집 대화상자"""
        messagebox.showinfo("정보", "작업 편집 기능은 구현 예정입니다.")
    
    def delete_job(self):
        """작업 삭제"""
        messagebox.showinfo("정보", "작업 삭제 기능은 구현 예정입니다.")
    
    def execute_queue(self):
        """큐 실행"""
        messagebox.showinfo("정보", "큐 실행 기능은 구현 예정입니다.")
    
    def stop_queue(self):
        """큐 정지"""
        messagebox.showinfo("정보", "큐 정지 기능은 구현 예정입니다.")
    
    def clear_logs(self):
        """로그 지우기"""
        if hasattr(self, 'log_text'):
            self.log_text.delete(1.0, tk.END)
    
    def save_logs(self):
        """로그 저장"""
        if hasattr(self, 'log_text'):
            filename = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("텍스트 파일", "*.txt"), ("모든 파일", "*.*")]
            )
            if filename:
                try:
                    with open(filename, 'w', encoding='utf-8') as f:
                        f.write(self.log_text.get(1.0, tk.END))
                    self.log_message(f"로그를 저장했습니다: {filename}", "SUCCESS")
                except Exception as e:
                    self.log_message(f"로그 저장 실패: {str(e)}", "ERROR")
    
    def export_logs(self):
        """로그 내보내기"""
        self.save_logs()
    
    def export_furniture_data(self):
        """가구 데이터 내보내기"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV 파일", "*.csv"), ("모든 파일", "*.*")]
        )
        if filename:
            try:
                furniture_list = self.db_manager.get_furniture_list()
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(['ID', 'Name', 'Type', 'Width', 'Height', 'Depth', 'Weight', 'X', 'Y', 'Z', 'Created'])
                    writer.writerows(furniture_list)
                self.log_message(f"가구 데이터를 내보냈습니다: {filename}", "SUCCESS")
            except Exception as e:
                self.log_message(f"데이터 내보내기 실패: {str(e)}", "ERROR")
    
    def save_settings(self):
        """설정 저장"""
        self.settings["speed"] = self.speed_var.get()
        self.settings["acceleration"] = self.accel_var.get()
        
        try:
            with open("settings.json", 'w') as f:
                json.dump(self.settings, f, indent=2)
            self.log_message("설정이 저장되었습니다.", "SUCCESS")
        except Exception as e:
            self.log_message(f"설정 저장 실패: {str(e)}", "ERROR")
    
    def load_settings(self):
        """설정 불러오기"""
        try:
            if os.path.exists("settings.json"):
                with open("settings.json", 'r') as f:
                    loaded_settings = json.load(f)
                self.settings.update(loaded_settings)
                
                # GUI 업데이트
                if hasattr(self, 'speed_var'):
                    self.speed_var.set(self.settings["speed"])
                    self.accel_var.set(self.settings["acceleration"])
                
                self.log_message("설정을 불러왔습니다.", "SUCCESS")
        except Exception as e:
            self.log_message(f"설정 불러오기 실패: {str(e)}", "ERROR")
    
    def calibrate_robot(self):
        """로봇 캘리브레이션"""
        messagebox.showinfo("정보", "캘리브레이션 기능은 구현 예정입니다.")
    
    def show_camera_view(self):
        """카메라 뷰 표시"""
        messagebox.showinfo("정보", "카메라 뷰 기능은 구현 예정입니다.")
    
    def open_furniture_manager(self):
        """가구 관리자 열기"""
        messagebox.showinfo("정보", "가구 관리자 기능은 구현 예정입니다.")
    
    def open_work_history(self):
        """작업 기록 열기"""
        messagebox.showinfo("정보", "작업 기록 기능은 구현 예정입니다.")
    
    def open_settings(self):
        """시스템 설정 열기"""
        dialog = tk.Toplevel(self.root)
        dialog.title("시스템 설정")
        dialog.geometry("400x300")
        dialog.transient(self.root)
        dialog.grab_set()
        
        notebook = ttk.Notebook(dialog)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 일반 설정 탭
        general_frame = ttk.Frame(notebook)
        notebook.add(general_frame, text="일반")
        
        # 안전 설정 탭
        safety_frame = ttk.Frame(notebook)
        notebook.add(safety_frame, text="안전")
        
        # 기본값으로 설정 추가
        ttk.Label(general_frame, text="설정 페이지는 구현 예정입니다.").pack(pady=50)
        ttk.Label(safety_frame, text="안전 설정은 구현 예정입니다.").pack(pady=50)
    
    def open_3d_viewer(self):
        """3D 뷰어 열기"""
        messagebox.showinfo("정보", "3D 뷰어 기능은 구현 예정입니다.")
    
    def show_help(self):
        """도움말 표시"""
        help_text = """
Dobot 가구 픽업 시스템 사용법

1. 연결
   - '로봇 연결' 버튼을 클릭하여 Dobot을 연결하세요.
   - DLL이 없으면 시뮬레이션 모드로 실행됩니다.

2. 수동 제어
   - 좌표를 입력하고 '이동' 버튼으로 위치를 제어하세요.
   - 조그 버튼으로 미세 조정이 가능합니다.

3. 가구 픽업
   - '비전 스캔'으로 객체를 감지하세요.
   - 감지된 객체를 선택하고 픽업 시퀀스를 계획하세요.
   - '픽업 실행'으로 자동 픽업을 시작하세요.

4. 안전
   - 비상 상황 시 '비상정지' 버튼을 사용하세요.
   - 안전 제한이 활성화되어 있습니다.

5. 로그
   - 모든 작업이 로그에 기록됩니다.
   - 로그는 저장하거나 내보낼 수 있습니다.
        """
        
        help_dialog = tk.Toplevel(self.root)
        help_dialog.title("도움말")
        help_dialog.geometry("500x400")
        help_dialog.transient(self.root)
        
        text_widget = tk.Text(help_dialog, wrap=tk.WORD, padx=10, pady=10)
        text_widget.pack(fill=tk.BOTH, expand=True)
        text_widget.insert(1.0, help_text)
        text_widget.config(state=tk.DISABLED)
    
    def show_about(self):
        """정보 표시"""
        about_text = f"""
Dobot 가구 픽업 시스템
Professional Edition v1.0

개발: AI Assistant
날짜: {datetime.now().strftime('%Y-%m-%d')}

기능:
- Dobot Magician 제어
- 비전 기반 객체 감지
- 자동 픽업 시퀀스
- 실시간 모니터링
- 작업 로그 관리

시스템 정보:
- Python {sys.version.split()[0]}
- Dobot DLL: {'사용가능' if DOBOT_AVAILABLE else '사용불가'}
- OpenCV: {'사용가능' if OPENCV_AVAILABLE else '사용불가'}
- Matplotlib: {'사용가능' if MATPLOTLIB_AVAILABLE else '사용불가'}

문의: support@example.com
        """
        messagebox.showinfo("프로그램 정보", about_text)
    
    def on_closing(self):
        """프로그램 종료 처리"""
        if self.is_connected:
            if messagebox.askokcancel("종료", "로봇이 연결되어 있습니다. 연결을 해제하고 종료하시겠습니까?"):
                self.disconnect_dobot()
                self.save_settings()
                self.root.destroy()
        else:
            self.save_settings()
            self.root.destroy()
    
    def run(self):
        """프로그램 실행"""
        self.log_message("=== Dobot 가구 픽업 시스템 시작 ===", "SUCCESS")
        self.log_message(f"Python 버전: {sys.version}", "INFO")
        self.log_message(f"시스템 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", "INFO")
        
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.log_message("프로그램이 중단되었습니다.", "WARNING")
        except Exception as e:
            self.log_message(f"예상치 못한 오류: {str(e)}", "ERROR")
        finally:
            if self.is_connected:
                self.disconnect_dobot()
            self.log_message("=== 프로그램 종료 ===", "INFO")

if __name__ == "__main__":
    app = DobotFurnitureSystem()
    app.run()
