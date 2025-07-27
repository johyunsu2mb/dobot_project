"""
yolo_detector.py - 개선된 YOLO 객체 인식 시스템

주요 개선사항:
- 메모리 누수 방지 (중요!)
- GPU/CPU 자동 선택
- 성능 최적화
- 에러 처리 강화
- 리소스 안전 관리
- 객체 추적 기능
- 좌표 변환 개선
"""

import cv2
import numpy as np
import threading
import time
import logging
import atexit
from typing import List, Dict, Optional, Tuple, Any, Callable
from pathlib import Path
import queue
from contextlib import contextmanager
import gc

# YOLOv8 import (설치되어 있는 경우)
try:
    from ultralytics import YOLO
    import torch
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ YOLO 라이브러리가 설치되지 않음. 시뮬레이션 모드로 동작합니다.")

logger = logging.getLogger(__name__)

# ========== 데이터 클래스 ==========

class DetectedObject:
    """감지된 객체 정보"""
    
    def __init__(self, class_name: str, confidence: float, bbox: Tuple[int, int, int, int],
                 center: Tuple[int, int], area: float, timestamp: float = None):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2)
        self.center = center  # (x, y)
        self.area = area
        self.timestamp = timestamp or time.time()
        self.id: Optional[int] = None  # 객체 추적용 ID
    
    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            'class_name': self.class_name,
            'confidence': self.confidence,
            'bbox': self.bbox,
            'center': self.center,
            'area': self.area,
            'timestamp': self.timestamp,
            'id': self.id
        }
    
    def get_robot_coordinates(self, calibration_matrix: np.ndarray = None) -> Tuple[float, float]:
        """로봇 좌표계로 변환"""
        if calibration_matrix is not None:
            # 실제 캘리브레이션 매트릭스를 사용한 변환
            cam_point = np.array([self.center[0], self.center[1], 1])
            robot_point = calibration_matrix @ cam_point
            return float(robot_point[0]), float(robot_point[1])
        else:
            # 간단한 스케일링 변환 (예시)
            # 실제 프로젝트에서는 카메라 캘리브레이션 결과 사용
            x_scale = 800 / 640  # 예시 값
            y_scale = 600 / 480  # 예시 값
            x_offset = 200       # 예시 값
            y_offset = 0         # 예시 값
            
            robot_x = (self.center[0] - 320) * x_scale + x_offset
            robot_y = (240 - self.center[1]) * y_scale + y_offset
            
            return robot_x, robot_y

# ========== YOLO 감지기 클래스 ==========

class YOLODetector:
    """개선된 YOLO 객체 감지기"""
    
    def __init__(self, model_name: str = 'yolov8n.pt', 
                 device: str = 'auto',
                 confidence_threshold: float = 0.5,
                 target_classes: List[str] = None):
        
        self.model_name = model_name
        self.confidence_threshold = confidence_threshold
        self.target_classes = target_classes or ['chair', 'couch', 'bed', 'dining table']
        
        # 모델 및 상태 관리
        self.model: Optional[Any] = None
        self.device = self._determine_device(device)
        self.is_initialized = False
        
        # 성능 모니터링
        self.detection_count = 0
        self.last_detection_time = 0
        self.avg_inference_time = 0
        
        # 객체 추적
        self.tracked_objects: Dict[int, DetectedObject] = {}
        self.next_object_id = 1
        
        # 리소스 관리
        self._cleanup_registered = False
        
        logger.info(f"YOLODetector 초기화: 모델={model_name}, 디바이스={self.device}")
    
    def _determine_device(self, device: str) -> str:
        """디바이스 자동 선택"""
        if device == 'auto':
            if YOLO_AVAILABLE and torch.cuda.is_available():
                logger.info("CUDA GPU 감지됨. GPU 사용")
                return 'cuda'
            else:
                logger.info("GPU 미사용 또는 미감지. CPU 사용")
                return 'cpu'
        return device
    
    def initialize(self) -> bool:
        """모델 초기화"""
        if self.is_initialized:
            return True
        
        if not YOLO_AVAILABLE:
            logger.warning("YOLO 라이브러리 없음. 시뮬레이션 모드로 동작")
            self.is_initialized = True
            return True
        
        try:
            logger.info(f"YOLO 모델 로딩 중: {self.model_name}")
            
            # 모델 로드
            self.model = YOLO(self.model_name)
            
            # 디바이스 설정
            if hasattr(self.model, 'to'):
                self.model.to(self.device)
            
            # 모델 최적화 (추론 속도 향상)
            if hasattr(self.model, 'fuse'):
                self.model.fuse()
            
            # 워밍업 추론 (첫 번째 추론은 느림)
            dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
            self._run_inference(dummy_image)
            
            self.is_initialized = True
            
            # 🔥 자동 정리 등록 (중요!)
            if not self._cleanup_registered:
                atexit.register(self.cleanup)
                self._cleanup_registered = True
            
            logger.info("YOLO 모델 초기화 완료")
            return True
            
        except Exception as e:
            logger.error(f"YOLO 모델 초기화 실패: {e}")
            return False
    
    def _run_inference(self, image: np.ndarray) -> List[DetectedObject]:
        """추론 실행"""
        if not YOLO_AVAILABLE or self.model is None:
            # 시뮬레이션 모드
            return self._simulate_detection(image)
        
        start_time = time.time()
        
        try:
            # YOLO 추론 실행
            results = self.model(image, conf=self.confidence_threshold, verbose=False)
            
            # 결과 파싱
            detected_objects = []
            
            for result in results:
                if hasattr(result, 'boxes') and result.boxes is not None:
                    boxes = result.boxes
                    
                    for i in range(len(boxes)):
                        # 바운딩 박스 좌표
                        x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                        
                        # 신뢰도
                        confidence = float(boxes.conf[i].cpu().numpy())
                        
                        # 클래스 ID 및 이름
                        class_id = int(boxes.cls[i].cpu().numpy())
                        class_name = self.model.names[class_id]
                        
                        # 타겟 클래스 필터링
                        if class_name not in self.target_classes:
                            continue
                        
                        # 중심점 및 면적 계산
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        area = (x2 - x1) * (y2 - y1)
                        
                        # DetectedObject 생성
                        obj = DetectedObject(
                            class_name=class_name,
                            confidence=confidence,
                            bbox=(int(x1), int(y1), int(x2), int(y2)),
                            center=(center_x, center_y),
                            area=area
                        )
                        
                        detected_objects.append(obj)
            
            # 성능 통계 업데이트
            inference_time = time.time() - start_time
            self._update_performance_stats(inference_time)
            
            return detected_objects
            
        except Exception as e:
            logger.error(f"YOLO 추론 실패: {e}")
            return []
    
    def _simulate_detection(self, image: np.ndarray) -> List[DetectedObject]:
        """시뮬레이션 모드 감지"""
        height, width = image.shape[:2]
        
        # 가짜 객체 생성 (테스트용)
        simulated_objects = []
        
        if self.detection_count % 30 == 0:  # 1초에 한 번 정도 감지 (30fps 기준)
            # 가운데 근처에 의자 감지 시뮬레이션
            center_x = width // 2 + np.random.randint(-50, 50)
            center_y = height // 2 + np.random.randint(-50, 50)
            
            bbox_width = 80
            bbox_height = 100
            
            obj = DetectedObject(
                class_name='chair',
                confidence=0.85,
                bbox=(center_x - bbox_width//2, center_y - bbox_height//2,
                     center_x + bbox_width//2, center_y + bbox_height//2),
                center=(center_x, center_y),
                area=bbox_width * bbox_height
            )
            
            simulated_objects.append(obj)
        
        return simulated_objects
    
    def _update_performance_stats(self, inference_time: float):
        """성능 통계 업데이트"""
        self.detection_count += 1
        self.last_detection_time = time.time()
        
        # 이동 평균으로 추론 시간 계산
        alpha = 0.1
        self.avg_inference_time = (alpha * inference_time + 
                                 (1 - alpha) * self.avg_inference_time)
    
    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """객체 감지 메인 함수"""
        if not self.is_initialized:
            if not self.initialize():
                return []
        
        detected_objects = self._run_inference(image)
        
        # 객체 추적 업데이트
        if detected_objects:
            self._update_object_tracking(detected_objects)
        
        return detected_objects
    
    def _update_object_tracking(self, detected_objects: List[DetectedObject]):
        """객체 추적 업데이트"""
        # 간단한 추적 알고리즘 (중심점 기반)
        tracking_threshold = 100  # 픽셀 거리
        
        for obj in detected_objects:
            best_match_id = None
            best_distance = float('inf')
            
            # 기존 추적 객체와 매칭
            for track_id, tracked_obj in self.tracked_objects.items():
                distance = np.sqrt((obj.center[0] - tracked_obj.center[0])**2 + 
                                 (obj.center[1] - tracked_obj.center[1])**2)
                
                if distance < tracking_threshold and distance < best_distance:
                    best_distance = distance
                    best_match_id = track_id
            
            if best_match_id is not None:
                # 기존 객체 업데이트
                obj.id = best_match_id
                self.tracked_objects[best_match_id] = obj
            else:
                # 새 객체 추가
                obj.id = self.next_object_id
                self.tracked_objects[self.next_object_id] = obj
                self.next_object_id += 1
        
        # 오래된 추적 객체 제거
        current_time = time.time()
        ids_to_remove = []
        
        for track_id, tracked_obj in self.tracked_objects.items():
            if current_time - tracked_obj.timestamp > 5.0:  # 5초 후 제거
                ids_to_remove.append(track_id)
        
        for track_id in ids_to_remove:
            del self.tracked_objects[track_id]
    
    def get_best_target(self, preferred_class: str = None) -> Optional[DetectedObject]:
        """가장 적합한 타겟 객체 반환"""
        if not self.tracked_objects:
            return None
        
        # 선호 클래스가 있으면 해당 클래스 우선
        candidates = list(self.tracked_objects.values())
        
        if preferred_class:
            preferred_candidates = [obj for obj in candidates 
                                  if obj.class_name == preferred_class]
            if preferred_candidates:
                candidates = preferred_candidates
        
        # 신뢰도가 가장 높은 객체 선택
        best_object = max(candidates, key=lambda obj: obj.confidence)
        return best_object
    
    def get_performance_info(self) -> Dict[str, Any]:
        """성능 정보 반환"""
        return {
            'detection_count': self.detection_count,
            'avg_inference_time': self.avg_inference_time,
            'last_detection_time': self.last_detection_time,
            'tracked_objects_count': len(self.tracked_objects),
            'device': self.device,
            'is_initialized': self.is_initialized
        }
    
    def cleanup(self):
        """리소스 정리 - 메모리 누수 방지"""
        logger.info("YOLO 감지기 정리 시작...")
        
        try:
            # 추적 객체 정리
            self.tracked_objects.clear()
            
            # 모델 정리
            if self.model is not None:
                del self.model
                self.model = None
            
            # GPU 메모리 정리
            if YOLO_AVAILABLE and torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            # 가비지 컬렉션
            gc.collect()
            
            self.is_initialized = False
            logger.info("YOLO 감지기 정리 완료")
            
        except Exception as e:
            logger.error(f"YOLO 정리 중 오류: {e}")
    
    # Context Manager 지원
    def __enter__(self):
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

# ========== 카메라 관리 클래스 ==========

class CameraManager:
    """카메라 관리 클래스 - 메모리 누수 방지"""
    
    def __init__(self, camera_index: int = 0, 
                 resolution: Tuple[int, int] = (640, 480),
                 fps: int = 30):
        self.camera_index = camera_index
        self.resolution = resolution
        self.fps = fps
        
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_opened = False
        self.frame_count = 0
        self.last_frame_time = 0
        
        # 자동 정리 등록
        atexit.register(self.release)
    
    def initialize(self) -> bool:
        """카메라 초기화"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                logger.error(f"카메라 {self.camera_index} 열기 실패")
                return False
            
            # 카메라 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 버퍼 크기 최소화 (지연 감소)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            self.is_opened = True
            logger.info(f"카메라 초기화 완료: {self.resolution}@{self.fps}fps")
            return True
            
        except Exception as e:
            logger.error(f"카메라 초기화 실패: {e}")
            return False
    
    def read_frame(self) -> Optional[np.ndarray]:
        """프레임 읽기"""
        if not self.is_opened or self.cap is None:
            return None
        
        try:
            ret, frame = self.cap.read()
            
            if ret:
                self.frame_count += 1
                self.last_frame_time = time.time()
                return frame
            else:
                logger.warning("프레임 읽기 실패")
                return None
                
        except Exception as e:
            logger.error(f"프레임 읽기 오류: {e}")
            return None
    
    def get_fps(self) -> float:
        """실제 FPS 계산"""
        if self.frame_count < 2:
            return 0.0
        
        current_time = time.time()
        elapsed_time = current_time - (self.last_frame_time - self.frame_count / self.fps)
        
        if elapsed_time > 0:
            return self.frame_count / elapsed_time
        return 0.0
    
    def release(self):
        """카메라 리소스 해제"""
        try:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            
            self.is_opened = False
            logger.info("카메라 리소스 해제 완료")
            
        except Exception as e:
            logger.error(f"카메라 해제 오류: {e}")
    
    # Context Manager 지원
    def __enter__(self):
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

# ========== 통합 비전 시스템 ==========

class VisionSystem:
    """통합 비전 시스템"""
    
    def __init__(self, camera_index: int = 0,
                 model_name: str = 'yolov8n.pt',
                 confidence_threshold: float = 0.5):
        
        self.camera_manager = CameraManager(camera_index)
        self.yolo_detector = YOLODetector(model_name, confidence_threshold=confidence_threshold)
        
        # 상태 관리
        self.is_running = False
        self.detection_thread: Optional[threading.Thread] = None
        self.detection_queue = queue.Queue(maxsize=10)
        
        # 콜백 함수들
        self.detection_callbacks: List[Callable[[List[DetectedObject]], None]] = []
        
        # 성능 모니터링
        self.total_frames = 0
        self.start_time = time.time()
        
        logger.info("VisionSystem 초기화 완료")
    
    def add_detection_callback(self, callback: Callable[[List[DetectedObject]], None]):
        """감지 결과 콜백 추가"""
        self.detection_callbacks.append(callback)
    
    def start(self) -> bool:
        """비전 시스템 시작"""
        if self.is_running:
            return True
        
        # 카메라 초기화
        if not self.camera_manager.initialize():
            return False
        
        # YOLO 초기화
        if not self.yolo_detector.initialize():
            return False
        
        # 감지 스레드 시작
        self.is_running = True
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        
        logger.info("비전 시스템 시작 완료")
        return True
    
    def stop(self):
        """비전 시스템 중지"""
        self.is_running = False
        
        if self.detection_thread:
            self.detection_thread.join(timeout=2.0)
        
        self.camera_manager.release()
        self.yolo_detector.cleanup()
        
        logger.info("비전 시스템 중지 완료")
    
    def _detection_loop(self):
        """감지 루프 (별도 스레드에서 실행)"""
        logger.info("감지 루프 시작")
        
        while self.is_running:
            try:
                # 프레임 읽기
                frame = self.camera_manager.read_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                
                # 객체 감지
                detected_objects = self.yolo_detector.detect_objects(frame)
                
                # 결과를 큐에 추가
                try:
                    self.detection_queue.put_nowait({
                        'frame': frame.copy(),
                        'objects': detected_objects,
                        'timestamp': time.time()
                    })
                except queue.Full:
                    # 큐가 가득 찬 경우 오래된 결과 제거
                    try:
                        self.detection_queue.get_nowait()
                        self.detection_queue.put_nowait({
                            'frame': frame.copy(),
                            'objects': detected_objects,
                            'timestamp': time.time()
                        })
                    except queue.Empty:
                        pass
                
                # 콜백 호출
                for callback in self.detection_callbacks:
                    try:
                        callback(detected_objects)
                    except Exception as e:
                        logger.error(f"감지 콜백 실행 오류: {e}")
                
                self.total_frames += 1
                
                # CPU 사용률 조절
                time.sleep(0.033)  # ~30fps
                
            except Exception as e:
                logger.error(f"감지 루프 오류: {e}")
                time.sleep(1.0)
        
        logger.info("감지 루프 종료")
    
    def get_latest_detection(self) -> Optional[Dict[str, Any]]:
        """최신 감지 결과 반환"""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_performance_info(self) -> Dict[str, Any]:
        """성능 정보 반환"""
        elapsed_time = time.time() - self.start_time
        overall_fps = self.total_frames / elapsed_time if elapsed_time > 0 else 0
        
        info = {
            'overall_fps': overall_fps,
            'total_frames': self.total_frames,
            'elapsed_time': elapsed_time,
            'camera_fps': self.camera_manager.get_fps(),
            'is_running': self.is_running
        }
        
        # YOLO 성능 정보 추가
        info.update(self.yolo_detector.get_performance_info())
        
        return info
    
    # Context Manager 지원
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

# ========== 헬퍼 함수들 ==========

def draw_detections(image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
    """감지 결과를 이미지에 그리기"""
    result_image = image.copy()
    
    for obj in objects:
        # 바운딩 박스
        x1, y1, x2, y2 = obj.bbox
        cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 중심점
        cv2.circle(result_image, obj.center, 5, (0, 0, 255), -1)
        
        # 라벨
        label = f"{obj.class_name}: {obj.confidence:.2f}"
        if obj.id is not None:
            label += f" (ID: {obj.id})"
        
        cv2.putText(result_image, label, (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return result_image

def save_detection_results(objects: List[DetectedObject], filename: str):
    """감지 결과를 파일로 저장"""
    try:
        import json
        data = [obj.to_dict() for obj in objects]
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        
        logger.info(f"감지 결과 저장 완료: {filename}")
        
    except Exception as e:
        logger.error(f"감지 결과 저장 실패: {e}")

# ========== 테스트 함수 ==========

def test_yolo_system():
    """YOLO 시스템 테스트"""
    print("🧪 YOLO 시스템 테스트 시작")
    
    try:
        with VisionSystem() as vision:
            print("✅ 비전 시스템 초기화 성공")
            
            # 몇 초간 실행
            time.sleep(3.0)
            
            # 성능 정보 확인
            perf_info = vision.get_performance_info()
            print(f"✅ 성능 정보: FPS={perf_info['overall_fps']:.1f}, "
                  f"프레임={perf_info['total_frames']}")
            
            # 최신 감지 결과 확인
            latest = vision.get_latest_detection()
            if latest:
                print(f"✅ 감지된 객체 수: {len(latest['objects'])}")
            
        print("✅ YOLO 시스템 테스트 완료")
        
    except Exception as e:
        print(f"❌ YOLO 시스템 테스트 실패: {e}")

if __name__ == "__main__":
    # 로깅 설정
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 테스트 실행
    test_yolo_system()
