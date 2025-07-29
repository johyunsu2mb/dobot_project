"""
yolo_detector.py - YOLO 객체 인식 클래스
Enhanced Dobot Robot & YOLO Object Detection System
"""

import logging
from config import DEPENDENCIES

# 조건부 임포트
if DEPENDENCIES['YOLO_AVAILABLE'] and DEPENDENCIES['CV2_AVAILABLE']:
    try:
        from ultralytics import YOLO
        import cv2
    except ImportError:
        DEPENDENCIES['YOLO_AVAILABLE'] = False
        DEPENDENCIES['CV2_AVAILABLE'] = False

class YOLODetector:
    """향상된 YOLO 객체 인식 클래스 (안전한 로깅)"""
    def __init__(self):
        self.model = None
        self.custom_labels = None
        self.model_loaded = False
        self.logger = logging.getLogger('robot_system.yolo')
       
        if DEPENDENCIES['YOLO_AVAILABLE']:
            try:
                self.model = YOLO("best.pt")
                self.model_loaded = True
                try:
                    self.logger.info("YOLOv8 model loaded successfully")
                except:
                    print("YOLOv8 model loaded successfully")
            except Exception as e:
                try:
                    self.logger.error(f"YOLO model loading failed: {e}")
                except:
                    print(f"YOLO model loading failed: {e}")
                self.model_loaded = False
   
    def load_custom_labels(self, label_file_path: str) -> bool:
        """커스텀 라벨 파일 로드 (안전한 로깅)"""
        try:
            with open(label_file_path, 'r', encoding='utf-8') as f:
                self.custom_labels = [line.strip() for line in f.readlines()]
            try:
                self.logger.info(f"Custom labels loaded successfully: {len(self.custom_labels)} classes")
            except:
                print(f"Custom labels loaded successfully: {len(self.custom_labels)} classes")
            return True
        except Exception as e:
            try:
                self.logger.error(f"Label file loading failed: {e}")
            except:
                print(f"Label file loading failed: {e}")
            return False
   
    def detect_objects(self, frame, confidence: float = 0.5):
        """객체 인식 수행 (안전한 로깅)"""
        if not self.model_loaded or self.model is None:
            return frame, []
       
        try:
            results = self.model.predict(frame, conf=confidence, verbose=False)
           
            detections = []
            if results and len(results) > 0:
                result = results[0]
                if result.boxes is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    confidences = result.boxes.conf.cpu().numpy()
                    class_ids = result.boxes.cls.cpu().numpy()
                   
                    for i, (box, conf, cls_id) in enumerate(zip(boxes, confidences, class_ids)):
                        x1, y1, x2, y2 = map(int, box)
                       
                        if self.custom_labels and int(cls_id) < len(self.custom_labels):
                            class_name = self.custom_labels[int(cls_id)]
                        else:
                            class_name = self.model.names[int(cls_id)]
                       
                        detections.append({
                            'bbox': (x1, y1, x2, y2),
                            'confidence': float(conf),
                            'class': class_name,
                            'class_id': int(cls_id)
                        })
           
            annotated_frame = self.draw_detections(frame.copy(), detections)
            return annotated_frame, detections
           
        except Exception as e:
            try:
                self.logger.error(f"Object detection error: {e}")
            except:
                print(f"Object detection error: {e}")
            return frame, []
   
    def draw_detections(self, frame, detections):
        """인식 결과를 프레임에 그리기"""
        if not DEPENDENCIES['CV2_AVAILABLE']:
            return frame
            
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class']
           
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
           
            label = f"{class_name}: {confidence:.2f}"
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
       
        return frame
