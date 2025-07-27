"""
utils.py - 개선된 유틸리티 함수들

주요 개선사항:
- 안전한 에러 처리 클래스들
- 좌표 변환 유틸리티
- 연결 상태 모니터링
- 성능 측정 도구
- 파일 및 경로 관리
"""

import os
import sys
import time
import json
import yaml
import logging
import threading
import queue
import functools
import traceback
from typing import Any, Dict, List, Optional, Tuple, Callable, Union
from pathlib import Path
from dataclasses import dataclass
from contextlib import contextmanager
import socket

# psutil 선택적 import
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

logger = logging.getLogger(__name__)

# ========== 에러 클래스들 ==========

class DobotError(Exception):
    """Dobot 관련 기본 에러 클래스"""
    pass

class DobotConnectionError(DobotError):
    """연결 관련 에러"""
    def __init__(self, message: str, retry_possible: bool = True):
        super().__init__(message)
        self.retry_possible = retry_possible

class DobotTimeoutError(DobotError):
    """타임아웃 에러"""
    def __init__(self, message: str, timeout_duration: float):
        super().__init__(message)
        self.timeout_duration = timeout_duration

class DobotMovementError(DobotError):
    """움직임 관련 에러"""
    def __init__(self, message: str, position: Optional[Tuple[float, float, float, float]] = None):
        super().__init__(message)
        self.position = position

class DobotWorkspaceError(DobotError):
    """작업공간 제한 에러"""
    def __init__(self, message: str, coordinate: Tuple[float, float, float, float]):
        super().__init__(message)
        self.coordinate = coordinate

# ========== 데코레이터들 ==========

def retry_on_failure(max_retries: int = 3, delay: float = 1.0, 
                    exceptions: Tuple = (Exception,)):
    """
    실패시 재시도하는 데코레이터
    
    Args:
        max_retries: 최대 재시도 횟수
        delay: 재시도 간 대기 시간
        exceptions: 재시도할 예외 타입들
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None
            
            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt < max_retries:
                        logger.warning(f"{func.__name__} 실패 (시도 {attempt + 1}/{max_retries + 1}): {e}")
                        time.sleep(delay)
                    else:
                        logger.error(f"{func.__name__} 모든 재시도 실패: {e}")
            
            raise last_exception
        return wrapper
    return decorator

def timeout_handler(timeout_seconds: float):
    """
    타임아웃 처리 데코레이터
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            result = [None]
            exception = [None]
            
            def target():
                try:
                    result[0] = func(*args, **kwargs)
                except Exception as e:
                    exception[0] = e
            
            thread = threading.Thread(target=target)
            thread.daemon = True
            thread.start()
            thread.join(timeout_seconds)
            
            if thread.is_alive():
                # 타임아웃 발생
                raise DobotTimeoutError(f"{func.__name__} 타임아웃 ({timeout_seconds}초)", timeout_seconds)
            
            if exception[0]:
                raise exception[0]
            
            return result[0]
        return wrapper
    return decorator

def log_execution_time(func):
    """
    함수 실행 시간을 로깅하는 데코레이터
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = func(*args, **kwargs)
            execution_time = time.time() - start_time
            logger.debug(f"{func.__name__} 실행 시간: {execution_time:.3f}초")
            return result
        except Exception as e:
            execution_time = time.time() - start_time
            logger.debug(f"{func.__name__} 실행 시간 (실패): {execution_time:.3f}초")
            raise
    return wrapper

def safe_execution(default_return=None, log_error=True):
    """
    안전한 실행을 위한 데코레이터 - 예외 발생시 기본값 반환
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                if log_error:
                    logger.error(f"{func.__name__} 실행 중 오류: {e}")
                return default_return
        return wrapper
    return decorator

# ========== 좌표 및 계산 유틸리티 ==========

@dataclass
class Position:
    """위치 정보 클래스"""
    x: float
    y: float
    z: float
    r: float = 0.0
    
    def __iter__(self):
        return iter((self.x, self.y, self.z, self.r))
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.r)
    
    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "r": self.r}
    
    def distance_to(self, other: 'Position') -> float:
        """다른 위치까지의 거리 계산"""
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)**0.5
    
    def is_within_bounds(self, bounds: Dict[str, Tuple[float, float]]) -> bool:
        """주어진 범위 내에 있는지 확인"""
        return (bounds['x'][0] <= self.x <= bounds['x'][1] and
                bounds['y'][0] <= self.y <= bounds['y'][1] and
                bounds['z'][0] <= self.z <= bounds['z'][1] and
                bounds['r'][0] <= self.r <= bounds['r'][1])

class CoordinateValidator:
    """좌표 검증 클래스"""
    
    def __init__(self, workspace_limits: Dict[str, Tuple[float, float]]):
        self.limits = workspace_limits
    
    def validate_position(self, position: Union[Position, Tuple[float, float, float, float]]) -> bool:
        """위치 유효성 검사"""
        if isinstance(position, tuple):
            position = Position(*position)
        
        return position.is_within_bounds(self.limits)
    
    def clamp_position(self, position: Union[Position, Tuple[float, float, float, float]]) -> Position:
        """위치를 유효 범위 내로 제한"""
        if isinstance(position, tuple):
            position = Position(*position)
        
        clamped = Position(
            x=max(self.limits['x'][0], min(position.x, self.limits['x'][1])),
            y=max(self.limits['y'][0], min(position.y, self.limits['y'][1])),
            z=max(self.limits['z'][0], min(position.z, self.limits['z'][1])),
            r=max(self.limits['r'][0], min(position.r, self.limits['r'][1]))
        )
        
        return clamped
    
    def get_safe_approach_position(self, target: Position, offset_z: float = 50.0) -> Position:
        """안전한 접근 위치 계산"""
        return Position(target.x, target.y, target.z + offset_z, target.r)

def interpolate_positions(start: Position, end: Position, steps: int = 10) -> List[Position]:
    """두 위치 사이를 보간하여 중간 위치들 생성"""
    positions = []
    
    for i in range(steps + 1):
        t = i / steps
        
        interpolated = Position(
            x=start.x + (end.x - start.x) * t,
            y=start.y + (end.y - start.y) * t,
            z=start.z + (end.z - start.z) * t,
            r=start.r + (end.r - start.r) * t
        )
        
        positions.append(interpolated)
    
    return positions

# ========== 연결 상태 모니터링 ==========

class ConnectionMonitor:
    """연결 상태 모니터링 클래스"""
    
    def __init__(self, check_interval: float = 5.0):
        self.check_interval = check_interval
        self.is_monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.status_callbacks: List[Callable] = []
        self.last_status = None
    
    def add_status_callback(self, callback: Callable[[bool], None]):
        """상태 변경 콜백 추가"""
        self.status_callbacks.append(callback)
    
    def start_monitoring(self, check_function: Callable[[], bool]):
        """모니터링 시작"""
        if self.is_monitoring:
            return
        
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(check_function,),
            daemon=True
        )
        self.monitor_thread.start()
        logger.info("연결 모니터링 시작")
    
    def stop_monitoring(self):
        """모니터링 중지"""
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        logger.info("연결 모니터링 중지")
    
    def _monitor_loop(self, check_function: Callable[[], bool]):
        """모니터링 루프"""
        while self.is_monitoring:
            try:
                current_status = check_function()
                
                if current_status != self.last_status:
                    logger.info(f"연결 상태 변경: {self.last_status} -> {current_status}")
                    
                    # 모든 콜백 호출
                    for callback in self.status_callbacks:
                        try:
                            callback(current_status)
                        except Exception as e:
                            logger.error(f"상태 콜백 실행 중 오류: {e}")
                    
                    self.last_status = current_status
                
            except Exception as e:
                logger.error(f"연결 상태 확인 중 오류: {e}")
            
            time.sleep(self.check_interval)

# ========== 성능 측정 도구 ==========

class PerformanceMonitor:
    """성능 측정 클래스"""
    
    def __init__(self):
        self.metrics = {}
        self.start_times = {}
    
    @contextmanager
    def measure_time(self, metric_name: str):
        """시간 측정 컨텍스트 매니저"""
        start_time = time.time()
        try:
            yield
        finally:
            elapsed = time.time() - start_time
            self.record_metric(metric_name, elapsed)
    
    def record_metric(self, name: str, value: float):
        """메트릭 기록"""
        if name not in self.metrics:
            self.metrics[name] = []
        
        self.metrics[name].append({
            'value': value,
            'timestamp': time.time()
        })
        
        # 최근 100개만 유지
        if len(self.metrics[name]) > 100:
            self.metrics[name] = self.metrics[name][-100:]
    
    def get_average(self, metric_name: str, recent_count: int = 10) -> Optional[float]:
        """평균값 계산"""
        if metric_name not in self.metrics:
            return None
        
        recent_values = [m['value'] for m in self.metrics[metric_name][-recent_count:]]
        if not recent_values:
            return None
        
        return sum(recent_values) / len(recent_values)
    
    def get_system_info(self) -> Dict[str, Any]:
        """시스템 정보 조회"""
        try:
            if PSUTIL_AVAILABLE:
                cpu_percent = psutil.cpu_percent(interval=1)
                memory = psutil.virtual_memory()
                disk = psutil.disk_usage('/')
                
                return {
                    'cpu_percent': cpu_percent,
                    'memory_percent': memory.percent,
                    'memory_available_mb': memory.available / 1024 / 1024,
                    'disk_percent': disk.percent,
                    'disk_free_gb': disk.free / 1024 / 1024 / 1024
                }
            else:
                return {
                    'cpu_percent': 0,
                    'memory_percent': 0,
                    'memory_available_mb': 0,
                    'disk_percent': 0,
                    'disk_free_gb': 0,
                    'note': 'psutil not available'
                }
        except Exception as e:
            logger.error(f"시스템 정보 조회 실패: {e}")
            return {}

# ========== 파일 및 경로 관리 ==========

class FileManager:
    """파일 관리 클래스"""
    
    def __init__(self, base_dir: str = "."):
        self.base_dir = Path(base_dir)
        self.ensure_directories()
    
    def ensure_directories(self):
        """필요한 디렉토리 생성"""
        directories = ['logs', 'data', 'models', 'config', 'temp', 'fonts']
        
        for dir_name in directories:
            dir_path = self.base_dir / dir_name
            dir_path.mkdir(exist_ok=True)
            logger.debug(f"디렉토리 확인: {dir_path}")
    
    def get_log_path(self, log_name: str) -> Path:
        """로그 파일 경로 반환"""
        return self.base_dir / 'logs' / f"{log_name}.log"
    
    def get_config_path(self, config_name: str) -> Path:
        """설정 파일 경로 반환"""
        return self.base_dir / 'config' / f"{config_name}.yaml"
    
    def save_json(self, data: Any, filename: str, subdir: str = 'data') -> bool:
        """JSON 파일 저장"""
        try:
            file_path = self.base_dir / subdir / f"{filename}.json"
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            logger.debug(f"JSON 저장 완료: {file_path}")
            return True
        except Exception as e:
            logger.error(f"JSON 저장 실패: {e}")
            return False
    
    def load_json(self, filename: str, subdir: str = 'data') -> Optional[Any]:
        """JSON 파일 로드"""
        try:
            file_path = self.base_dir / subdir / f"{filename}.json"
            if not file_path.exists():
                return None
            
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            logger.debug(f"JSON 로드 완료: {file_path}")
            return data
        except Exception as e:
            logger.error(f"JSON 로드 실패: {e}")
            return None
    
    def save_yaml(self, data: Any, filename: str, subdir: str = 'config') -> bool:
        """YAML 파일 저장"""
        try:
            file_path = self.base_dir / subdir / f"{filename}.yaml"
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            logger.debug(f"YAML 저장 완료: {file_path}")
            return True
        except Exception as e:
            logger.error(f"YAML 저장 실패: {e}")
            return False
    
    def load_yaml(self, filename: str, subdir: str = 'config') -> Optional[Any]:
        """YAML 파일 로드"""
        try:
            file_path = self.base_dir / subdir / f"{filename}.yaml"
            if not file_path.exists():
                return None
            
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            logger.debug(f"YAML 로드 완료: {file_path}")
            return data
        except Exception as e:
            logger.error(f"YAML 로드 실패: {e}")
            return None
    
    def cleanup_old_files(self, subdir: str = 'logs', max_age_days: int = 7):
        """오래된 파일 정리"""
        try:
            dir_path = self.base_dir / subdir
            if not dir_path.exists():
                return
            
            cutoff_time = time.time() - (max_age_days * 24 * 60 * 60)
            
            for file_path in dir_path.iterdir():
                if file_path.is_file() and file_path.stat().st_mtime < cutoff_time:
                    file_path.unlink()
                    logger.debug(f"오래된 파일 삭제: {file_path}")
                    
        except Exception as e:
            logger.error(f"파일 정리 실패: {e}")

# ========== 스레드 안전 큐 ==========

class SafeQueue:
    """스레드 안전 큐 클래스"""
    
    def __init__(self, maxsize: int = 0):
        self.queue = queue.Queue(maxsize=maxsize)
    
    def put(self, item: Any, timeout: Optional[float] = None) -> bool:
        """아이템 추가"""
        try:
            self.queue.put(item, timeout=timeout)
            return True
        except queue.Full:
            logger.warning("큐가 가득 참")
            return False
    
    def get(self, timeout: Optional[float] = None) -> Optional[Any]:
        """아이템 가져오기"""
        try:
            return self.queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def size(self) -> int:
        """큐 크기 반환"""
        return self.queue.qsize()
    
    def clear(self):
        """큐 비우기"""
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break

# ========== 네트워크 유틸리티 ==========

def check_network_connection(host: str, port: int, timeout: float = 5.0) -> bool:
    """네트워크 연결 확인"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception:
        return False

def get_local_ip() -> str:
    """로컬 IP 주소 조회"""
    try:
        # 구글 DNS에 연결을 시도하여 로컬 IP 확인
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        return local_ip
    except Exception:
        return "127.0.0.1"

def find_available_port(start_port: int = 8000, end_port: int = 9000) -> Optional[int]:
    """사용 가능한 포트 찾기"""
    for port in range(start_port, end_port + 1):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', port))
                return port
        except OSError:
            continue
    return None

# ========== 데이터 변환 유틸리티 ==========

def safe_float_conversion(value: Any, default: float = 0.0) -> float:
    """안전한 float 변환"""
    try:
        return float(value)
    except (ValueError, TypeError):
        return default

def safe_int_conversion(value: Any, default: int = 0) -> int:
    """안전한 int 변환"""
    try:
        return int(float(value))  # float을 거쳐서 "123.45"도 처리
    except (ValueError, TypeError):
        return default

def parse_robot_response(response: str) -> Dict[str, Any]:
    """로봇 응답 파싱"""
    try:
        if not response:
            return {'status': 'error', 'message': '빈 응답'}
        
        response = response.strip()
        
        if response.startswith('OK'):
            parts = response.split(',')
            if len(parts) > 1:
                # 데이터가 포함된 응답
                data = [safe_float_conversion(part.strip('{}')) for part in parts[1:]]
                return {'status': 'success', 'data': data}
            else:
                # 단순 성공 응답
                return {'status': 'success'}
        
        elif response.startswith('ERROR'):
            return {'status': 'error', 'message': response}
        
        else:
            return {'status': 'unknown', 'raw': response}
    
    except Exception as e:
        return {'status': 'error', 'message': f'응답 파싱 실패: {e}'}

# ========== 전역 인스턴스들 ==========

# 전역 파일 매니저
file_manager = FileManager()

# 전역 성능 모니터
performance_monitor = PerformanceMonitor()

# 전역 연결 모니터
connection_monitor = ConnectionMonitor()

# ========== 헬퍼 함수들 ==========

def get_timestamp_string() -> str:
    """타임스탬프 문자열 반환"""
    return time.strftime('%Y%m%d_%H%M%S')

def format_duration(seconds: float) -> str:
    """초를 사람이 읽기 쉬운 형태로 변환"""
    if seconds < 60:
        return f"{seconds:.1f}초"
    elif seconds < 3600:
        minutes = seconds / 60
        return f"{minutes:.1f}분"
    else:
        hours = seconds / 3600
        return f"{hours:.1f}시간"

def format_bytes(bytes_value: int) -> str:
    """바이트를 사람이 읽기 쉬운 형태로 변환"""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_value < 1024:
            return f"{bytes_value:.1f} {unit}"
        bytes_value /= 1024
    return f"{bytes_value:.1f} PB"

@safe_execution(default_return="알 수 없음", log_error=False)
def get_system_user() -> str:
    """시스템 사용자명 반환"""
    return os.getenv('USERNAME') or os.getenv('USER') or "unknown"

def create_error_report(exception: Exception, context: str = "") -> Dict[str, Any]:
    """에러 리포트 생성"""
    return {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'exception_type': type(exception).__name__,
        'exception_message': str(exception),
        'context': context,
        'traceback': traceback.format_exc(),
        'system_info': performance_monitor.get_system_info(),
        'user': get_system_user()
    }

# ========== 초기화 및 정리 ==========

def initialize_utils():
    """유틸리티 초기화"""
    logger.info("유틸리티 시스템 초기화 완료")
    
    # 필요한 디렉토리 생성
    file_manager.ensure_directories()
    
    # 오래된 로그 파일 정리
    file_manager.cleanup_old_files('logs', max_age_days=7)
    
    logger.info(f"로컬 IP: {get_local_ip()}")
    logger.info(f"시스템 사용자: {get_system_user()}")

def cleanup_utils():
    """유틸리티 정리"""
    logger.info("유틸리티 시스템 정리 중...")
    
    # 연결 모니터 중지
    connection_monitor.stop_monitoring()
    
    logger.info("유틸리티 시스템 정리 완료")

# 프로그램 시작시 자동 초기화
if __name__ != "__main__":
    initialize_utils()

# ========== 테스트 함수 ==========

def test_utils():
    """유틸리티 함수들 테스트"""
    print("🧪 유틸리티 테스트 시작")
    
    # Position 클래스 테스트
    pos1 = Position(100, 200, 300, 45)
    pos2 = Position(150, 250, 350, 90)
    distance = pos1.distance_to(pos2)
    print(f"✅ Position 테스트: 거리 = {distance:.2f}")
    
    # 좌표 검증 테스트
    limits = {
        'x': (-400, 400),
        'y': (-400, 400),
        'z': (-200, 200),
        'r': (-180, 180)
    }
    validator = CoordinateValidator(limits)
    is_valid = validator.validate_position(pos1)
    print(f"✅ 좌표 검증 테스트: {is_valid}")
    
    # 성능 모니터 테스트
    with performance_monitor.measure_time("test_operation"):
        time.sleep(0.1)
    avg_time = performance_monitor.get_average("test_operation")
    print(f"✅ 성능 모니터 테스트: 평균 시간 = {avg_time:.3f}초")
    
    # 파일 매니저 테스트
    test_data = {"test": "data", "timestamp": time.time()}
    save_success = file_manager.save_json(test_data, "test")
    loaded_data = file_manager.load_json("test")
    print(f"✅ 파일 매니저 테스트: 저장={save_success}, 로드={'성공' if loaded_data else '실패'}")
    
    # 네트워크 테스트
    is_connected = check_network_connection("8.8.8.8", 53, 2.0)
    local_ip = get_local_ip()
    print(f"✅ 네트워크 테스트: 연결={is_connected}, IP={local_ip}")
    
    print("🎉 모든 유틸리티 테스트 완료!")

if __name__ == "__main__":
    # 로깅 설정
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 테스트 실행
    test_utils()
