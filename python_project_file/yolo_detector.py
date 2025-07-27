#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 객체 인식 시스템
실시간 카메라를 통한 가구 객체 감지 및 좌표 추출
"""

import cv2
import numpy as np
import threading
import time
from typing import List, Dict, Tuple, Optional, Callable
import logging
from dataclasses import dataclass

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️  YOLOv8가 설치되지 않았습니다. 'pip install ultralytics' 로 설치하세요.")

from config import AppConfig


@dataclass
class DetectedObject:
    """감지된 객체 정보"""
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]  # (x, y)
    area: float


class YOLODetector:
    """YOLOv8 기반 객체 감지 클래스"""
    
    def __init__(self, model_path: str = "yolov8n.pt", logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(__name__)
        self.model_path = model_path
        self.model = None
        self.is_initialized = False
        
        # 가구 클래스 매핑 (COCO 데이터셋 기준)
        self.furniture_classes = {
            'chair': 56,      # 의자
            'couch': 57,      # 소파  
            'bed': 59,        # 침대
            'diningtable': 60, # 식탁
            'toilet': 61,     # 변기
            'tvmonitor': 62,  # TV
            'laptop': 63,     # 노트북
            'mouse': 64,      # 마우스
            'remote': 65,     # 리모컨
            'keyboard': 66,   # 키보드
            'cell phone': 67, # 휴대폰
            'book': 73,       # 책
            'clock': 74,      # 시계
            'vase': 75,       # 화병
            'teddy bear': 77, # 곰인형
        }
        
        # 한글 클래스 이름 매핑
        self.class_names_korean = {
            'chair': '의자',
            'couch': '소파',
            'bed': '침대',
            'diningtable': '책상',
            'toilet': '변기',
            'tvmonitor': 'TV',
            'laptop': '노트북',
            'mouse': '마우스',
            'remote': '리모컨',
            'keyboard': '키보드',
            'cell phone': '휴대폰',
            'book': '책',
            'clock': '시계',
            'vase': '화병',
            'teddy bear': '곰인형'
        }
        
        # 감지 설정
        self.confidence_threshold = 0.5
        self.iou_threshold = 0.45
        
        self.initialize_model()
    
    def initialize_model(self) -> bool:
        """YOLO 모델 초기화"""
        if not YOLO_AVAILABLE:
            self.logger.warning("YOLOv8가 설치되지 않아 객체 감지를 사용할 수 없습니다.")
            return False
        
        try:
            self.logger.info(f"YOLO 모델 로딩 중: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.is_initialized = True
            self.logger.info("YOLO 모델 초기화 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"YOLO 모델 초기화 실패: {e}")
            self.is_initialized = False
            return False
    
    def detect_objects(self, frame: np.ndarray) -> List[DetectedObject]:
        """프레임에서 객체 감지"""
        if not self.is_initialized or self.model is None:
            return []
        
        try:
            # YOLO 추론 실행
            results = self.model(frame, conf=self.confidence_threshold, iou=self.iou_threshold)
            
            detected_objects = []
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 바운딩 박스 좌표
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        
                        # 신뢰도
                        confidence = float(box.conf[0].cpu().numpy())
                        
                        # 클래스 ID 및 이름
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id]
                        
                        # 가구 객체만 필터링
                        if class_name in self.furniture_classes:
                            # 중심점 계산
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            # 면적 계산
                            area = (x2 - x1) * (y2 - y1)
                            
                            detected_obj = DetectedObject(
                                class_name=class_name,
                                confidence=confidence,
                                bbox=(int(x1), int(y1), int(x2), int(y2)),
                                center=(center_x, center_y),
                                area=area
                            )
                            
                            detected_objects.append(detected_obj)
            
            return detected_objects
            
        except Exception as e:
            self.logger.error(f"객체 감지 실패: {e}")
            return []
    
    def draw_detections(self, frame: np.ndarray, detections: List[DetectedObject]) -> np.ndarray:
        """감지된 객체를 프레임에 그리기"""
        annotated_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            
            # 바운딩 박스 그리기
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 한글 클래스 이름 가져오기
            korean_name = self.class_names_korean.get(detection.class_name, detection.class_name)
            
            # 레이블 텍스트
            label = f"{korean_name}: {detection.confidence:.2f}"
            
            # 레이블 배경
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), (0, 255, 0), -1)
            
            # 레이블 텍스트
            cv2.putText(annotated_frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # 중심점 표시
            cv2.circle(annotated_frame, detection.center, 5, (255, 0, 0), -1)
        
        return annotated_frame
    
    def pixel_to_robot_coordinates(self, pixel_x: int, pixel_y: int, 
                                 frame_width: int, frame_height: int) -> Tuple[float, float]:
        """픽셀 좌표를 로봇 좌표로 변환"""
        # 카메라 캘리브레이션 매개변수 (실제 환경에 맞게 조정 필요)
        # 이 값들은 카메라와 로봇의 상대 위치에 따라 달라집니다
        
        # 카메라 시야각 내에서 로봇 작업 영역 매핑
        robot_workspace_width = AppConfig.X_MAX - AppConfig.X_MIN  # 800mm
        robot_workspace_height = AppConfig.Y_MAX - AppConfig.Y_MIN  # 800mm
        
        # 픽셀 좌표를 정규화 (0~1)
        normalized_x = pixel_x / frame_width
        normalized_y = pixel_y / frame_height
        
        # 로봇 좌표로 변환
        robot_x = AppConfig.X_MIN + (normalized_x * robot_workspace_width)
        robot_y = AppConfig.Y_MAX - (normalized_y * robot_workspace_height)  # Y축 뒤집기
        
        return robot_x, robot_y
    
    def get_pickup_coordinates(self, detection: DetectedObject, 
                             frame_width: int, frame_height: int) -> Dict[str, float]:
        """감지된 객체의 픽업 좌표 계산"""
        # 픽셀 좌표를 로봇 좌표로 변환
        robot_x, robot_y = self.pixel_to_robot_coordinates(
            detection.center[0], detection.center[1], frame_width, frame_height
        )
        
        # 객체 타입에 따른 Z 좌표 설정
        z_heights = {
            'chair': 25.0,    # 의자 시트 높이
            'couch': 20.0,    # 소파 시트 높이
            'bed': 15.0,      # 침대 매트리스 높이
            'diningtable': 30.0, # 책상 표면 높이
            'book': 5.0,      # 책 두께
            'cell phone': 2.0, # 휴대폰 두께
        }
        
        robot_z = z_heights.get(detection.class_name, 10.0)  # 기본값 10mm
        
        return {
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'r': 0.0  # 기본 회전각
        }


class CameraManager:
    """카메라 관리 클래스"""
    
    def __init__(self, camera_index: int = 0, logger: logging.Logger = None):
        self.camera_index = camera_index
        self.logger = logger or logging.getLogger(__name__)
        self.cap = None
        self.is_running = False
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
    def initialize_camera(self) -> bool:
        """카메라 초기화"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                self.logger.error(f"카메라 {self.camera_index}를 열 수 없습니다.")
                return False
            
            # 카메라 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            self.logger.info(f"카메라 {self.camera_index} 초기화 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"카메라 초기화 실패: {e}")
            return False
    
    def start_capture(self):
        """카메라 캡처 시작"""
        if not self.cap or not self.cap.isOpened():
            if not self.initialize_camera():
                return False
        
        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.logger.info("카메라 캡처 시작")
        return True
    
    def stop_capture(self):
        """카메라 캡처 중지"""
        self.is_running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.logger.info("카메라 캡처 중지")
    
    def _capture_loop(self):
        """카메라 캡처 루프"""
        while self.is_running:
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                else:
                    self.logger.warning("프레임 읽기 실패")
                    time.sleep(0.1)
                    
            except Exception as e:
                self.logger.error(f"프레임 캡처 오류: {e}")
                time.sleep(0.1)
    
    def get_current_frame(self) -> Optional[np.ndarray]:
        """현재 프레임 반환"""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None


class YOLOSystem:
    """YOLOv8 통합 시스템"""
    
    def __init__(self, camera_index: int = 0, model_path: str = "yolov8n.pt", 
                 logger: logging.Logger = None):
        self.logger = logger or logging.getLogger(__name__)
        
        # 구성 요소 초기화
        self.detector = YOLODetector(model_path, logger)
        self.camera = CameraManager(camera_index, logger)
        
        # 콜백 함수들
        self.detection_callbacks: List[Callable] = []
        
        # 실행 상태
        self.is_active = False
        self.detection_thread = None
        
    def add_detection_callback(self, callback: Callable[[List[DetectedObject]], None]):
        """감지 결과 콜백 함수 추가"""
        self.detection_callbacks.append(callback)
    
    def start_detection(self) -> bool:
        """객체 감지 시작"""
        if not self.detector.is_initialized:
            self.logger.error("YOLO 모델이 초기화되지 않았습니다.")
            return False
        
        if not self.camera.start_capture():
            self.logger.error("카메라 시작 실패")
            return False
        
        self.is_active = True
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        
        self.logger.info("YOLOv8 객체 감지 시작")
        return True
    
    def stop_detection(self):
        """객체 감지 중지"""
        self.is_active = False
        
        if self.detection_thread:
            self.detection_thread.join(timeout=2.0)
        
        self.camera.stop_capture()
        self.logger.info("YOLOv8 객체 감지 중지")
    
    def _detection_loop(self):
        """객체 감지 루프"""
        while self.is_active:
            try:
                frame = self.camera.get_current_frame()
                if frame is not None:
                    # 객체 감지 실행
                    detections = self.detector.detect_objects(frame)
                    
                    # 콜백 함수 호출
                    for callback in self.detection_callbacks:
                        try:
                            callback(detections)
                        except Exception as e:
                            self.logger.error(f"감지 콜백 오류: {e}")
                
                time.sleep(0.1)  # 10 FPS
                
            except Exception as e:
                self.logger.error(f"감지 루프 오류: {e}")
                time.sleep(1.0)
    
    def get_annotated_frame(self) -> Optional[np.ndarray]:
        """주석이 달린 프레임 반환"""
        frame = self.camera.get_current_frame()
        if frame is None:
            return None
        
        detections = self.detector.detect_objects(frame)
        return self.detector.draw_detections(frame, detections)
    
    def get_furniture_positions(self) -> Dict[str, Dict[str, float]]:
        """감지된 가구들의 위치 정보 반환"""
        frame = self.camera.get_current_frame()
        if frame is None:
            return {}
        
        detections = self.detector.detect_objects(frame)
        furniture_positions = {}
        
        height, width = frame.shape[:2]
        
        for i, detection in enumerate(detections):
            # 동일한 클래스가 여러 개 감지된 경우 인덱스 추가
            key = detection.class_name
            if key in furniture_positions:
                key = f"{detection.class_name}_{i}"
            
            coordinates = self.detector.get_pickup_coordinates(detection, width, height)
            furniture_positions[key] = coordinates
        
        return furniture_positions
    
    def is_available(self) -> bool:
        """YOLOv8 사용 가능 여부"""
        return YOLO_AVAILABLE and self.detector.is_initialized


# 편의 함수들
def create_yolo_system(camera_index: int = 0, model_path: str = "yolov8n.pt") -> YOLOSystem:
    """YOLOv8 시스템 생성"""
    return YOLOSystem(camera_index, model_path)

def test_yolo_detection():
    """YOLOv8 감지 테스트"""
    if not YOLO_AVAILABLE:
        print("YOLOv8가 설치되지 않았습니다.")
        return False
    
    try:
        detector = YOLODetector()
        if detector.is_initialized:
            print("✅ YOLOv8 모델 로딩 성공")
            return True
        else:
            print("❌ YOLOv8 모델 로딩 실패")
            return False
    except Exception as e:
        print(f"❌ YOLOv8 테스트 실패: {e}")
        return False


if __name__ == "__main__":
    # 테스트 실행
    test_yolo_detection()
