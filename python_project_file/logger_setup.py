"""
logger_setup.py - 개선된 로깅 시스템 설정

주요 개선사항:
- Windows 환경에서 한글/이모지 안전 처리
- 회전 로그 파일 지원
- 컬러 로깅 지원
- 성능 최적화
- 에러 트래킹
- 로그 필터링
- 구조화된 로깅
"""

import os
import sys
import logging
import logging.handlers
import time
import threading
import queue
import json
from typing import Any, Dict, Optional, List
from pathlib import Path
from datetime import datetime
import traceback
import functools

# Windows 환경에서 한글 처리를 위한 설정
if sys.platform.startswith('win'):
    import locale
    try:
        # Windows에서 UTF-8 인코딩 설정
        locale.setlocale(locale.LC_ALL, 'Korean_Korea.UTF-8')
    except:
        try:
            locale.setlocale(locale.LC_ALL, 'ko_KR.UTF-8')
        except:
            pass

# ========== 안전한 문자열 처리 ==========

def safe_string(text: Any) -> str:
    """
    Windows 환경에서 안전한 문자열 변환
    한글, 이모지, 특수문자 등을 안전하게 처리
    """
    if text is None:
        return "None"
    
    try:
        # 이미 문자열인 경우
        if isinstance(text, str):
            # 이모지 및 특수 문자 필터링 (필요한 경우)
            if sys.platform.startswith('win'):
                # Windows에서 문제가 될 수 있는 문자들 처리
                safe_text = text.encode('utf-8', errors='replace').decode('utf-8', errors='replace')
                return safe_text
            return text
        
        # 다른 타입을 문자열로 변환
        return str(text)
        
    except Exception:
        # 모든 변환이 실패한 경우 안전한 표현 반환
        try:
            return repr(text)
        except:
            return f"<{type(text).__name__} object>"

def safe_format_message(msg: str, *args, **kwargs) -> str:
    """안전한 메시지 포맷팅"""
    try:
        if args or kwargs:
            # 모든 인자를 안전한 문자열로 변환
            safe_args = [safe_string(arg) for arg in args]
            safe_kwargs = {k: safe_string(v) for k, v in kwargs.items()}
            return msg.format(*safe_args, **safe_kwargs)
        return safe_string(msg)
    except Exception:
        # 포맷팅 실패시 안전한 문자열들 결합
        safe_msg = safe_string(msg)
        if args:
            safe_args_str = ", ".join(safe_string(arg) for arg in args)
            return f"{safe_msg} [args: {safe_args_str}]"
        return safe_msg

# ========== 커스텀 포매터 ==========

class SafeFormatter(logging.Formatter):
    """안전한 로그 포매터 - 한글/이모지 처리"""
    
    def __init__(self, fmt=None, datefmt=None, style='%'):
        super().__init__(fmt, datefmt, style)
    
    def format(self, record):
        # 모든 문자열 필드를 안전하게 변환
        record.getMessage = lambda: safe_format_message(record.msg, *record.args)
        record.name = safe_string(record.name)
        record.funcName = safe_string(record.funcName)
        record.pathname = safe_string(record.pathname)
        
        # 예외 정보도 안전하게 처리
        if record.exc_info:
            record.exc_text = safe_string(self.formatException(record.exc_info))
        
        return super().format(record)

class ColoredFormatter(SafeFormatter):
    """컬러 로그 포매터"""
    
    # ANSI 컬러 코드
    COLORS = {
        'DEBUG': '\033[36m',    # 청록색
        'INFO': '\033[32m',     # 녹색
        'WARNING': '\033[33m',  # 노란색
        'ERROR': '\033[31m',    # 빨간색
        'CRITICAL': '\033[35m', # 마젠타
        'RESET': '\033[0m'      # 리셋
    }
    
    def __init__(self, fmt=None, datefmt=None, use_colors=True):
        super().__init__(fmt, datefmt)
        self.use_colors = use_colors and self._supports_color()
    
    def _supports_color(self) -> bool:
        """컬러 지원 여부 확인"""
        # Windows에서는 colorama 필요, 없으면 컬러 비활성화
        if sys.platform.startswith('win'):
            try:
                import colorama
                return True
            except ImportError:
                return False
        return True
    
    def format(self, record):
        if self.use_colors:
            level_color = self.COLORS.get(record.levelname, '')
            reset_color = self.COLORS['RESET']
            
            # 레벨명에 컬러 적용
            colored_level = f"{level_color}{record.levelname}{reset_color}"
            record.levelname = colored_level
        
        return super().format(record)

class StructuredFormatter(SafeFormatter):
    """구조화된 JSON 로그 포매터"""
    
    def format(self, record):
        log_data = {
            'timestamp': datetime.fromtimestamp(record.created).isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': safe_string(record.getMessage()),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'thread': record.thread,
            'thread_name': record.threadName
        }
        
        # 예외 정보 추가
        if record.exc_info:
            log_data['exception'] = {
                'type': record.exc_info[0].__name__,
                'message': safe_string(record.exc_info[1]),
                'traceback': safe_string(self.formatException(record.exc_info))
            }
        
        # 추가 컨텍스트 정보
        if hasattr(record, 'extra_data'):
            log_data['extra'] = record.extra_data
        
        return json.dumps(log_data, ensure_ascii=False, default=str)

# ========== 커스텀 핸들러 ==========

class SafeFileHandler(logging.handlers.RotatingFileHandler):
    """안전한 파일 핸들러 - UTF-8 인코딩 보장"""
    
    def __init__(self, filename, mode='a', maxBytes=0, backupCount=0, encoding='utf-8', delay=False):
        # UTF-8 인코딩 강제 설정
        super().__init__(filename, mode, maxBytes, backupCount, encoding, delay)
    
    def emit(self, record):
        try:
            super().emit(record)
        except UnicodeEncodeError:
            # 인코딩 오류시 안전한 버전으로 재시도
            try:
                safe_record = logging.LogRecord(
                    record.name, record.levelno, record.pathname,
                    record.lineno, safe_string(record.msg), 
                    tuple(safe_string(arg) for arg in record.args),
                    record.exc_info, record.funcName
                )
                super().emit(safe_record)
            except Exception as e:
                # 최후의 수단: 간단한 에러 메시지 출력
                fallback_msg = f"로그 출력 실패: {safe_string(e)}\n"
                if self.stream:
                    self.stream.write(fallback_msg)

class AsyncHandler(logging.Handler):
    """비동기 로그 핸들러 - 성능 최적화"""
    
    def __init__(self, target_handler):
        super().__init__()
        self.target_handler = target_handler
        self.queue = queue.Queue(maxsize=1000)
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()
        self._shutdown = False
    
    def emit(self, record):
        if not self._shutdown:
            try:
                self.queue.put_nowait(record)
            except queue.Full:
                # 큐가 가득 찬 경우 오래된 레코드 제거
                try:
                    self.queue.get_nowait()
                    self.queue.put_nowait(record)
                except queue.Empty:
                    pass
    
    def _worker(self):
        """백그라운드 워커 스레드"""
        while not self._shutdown:
            try:
                record = self.queue.get(timeout=1.0)
                self.target_handler.emit(record)
                self.queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"AsyncHandler 오류: {e}")
    
    def close(self):
        self._shutdown = True
        self.worker_thread.join(timeout=2.0)
        super().close()

# ========== 로그 필터 ==========

class LevelFilter(logging.Filter):
    """레벨 기반 필터"""
    
    def __init__(self, min_level=logging.DEBUG, max_level=logging.CRITICAL):
        super().__init__()
        self.min_level = min_level
        self.max_level = max_level
    
    def filter(self, record):
        return self.min_level <= record.levelno <= self.max_level

class PerformanceFilter(logging.Filter):
    """성능 기반 필터 - 같은 메시지 반복 제한"""
    
    def __init__(self, max_duplicates=5, time_window=60):
        super().__init__()
        self.max_duplicates = max_duplicates
        self.time_window = time_window
        self.message_counts = {}
        self.last_cleanup = time.time()
    
    def filter(self, record):
        current_time = time.time()
        message_key = f"{record.name}:{record.levelno}:{record.getMessage()}"
        
        # 주기적으로 오래된 메시지 정리
        if current_time - self.last_cleanup > self.time_window:
            self._cleanup_old_messages(current_time)
            self.last_cleanup = current_time
        
        # 메시지 카운트 확인
        if message_key in self.message_counts:
            first_time, count = self.message_counts[message_key]
            
            if current_time - first_time < self.time_window:
                if count >= self.max_duplicates:
                    return False  # 중복 메시지 차단
                else:
                    self.message_counts[message_key] = (first_time, count + 1)
            else:
                # 시간 윈도우 만료, 새로 시작
                self.message_counts[message_key] = (current_time, 1)
        else:
            # 새 메시지
            self.message_counts[message_key] = (current_time, 1)
        
        return True
    
    def _cleanup_old_messages(self, current_time):
        """오래된 메시지 기록 정리"""
        keys_to_remove = []
        for key, (first_time, _) in self.message_counts.items():
            if current_time - first_time > self.time_window:
                keys_to_remove.append(key)
        
        for key in keys_to_remove:
            del self.message_counts[key]

# ========== 로거 설정 클래스 ==========

class LoggerSetup:
    """로거 설정 관리 클래스"""
    
    def __init__(self, base_dir: str = ".", log_level: str = "INFO"):
        self.base_dir = Path(base_dir)
        self.log_dir = self.base_dir / "logs"
        self.log_level = getattr(logging, log_level.upper())
        
        # 로그 디렉토리 생성
        self.log_dir.mkdir(exist_ok=True)
        
        # 기본 설정
        self.loggers = {}
        self.handlers = {}
        
        # Windows colorama 초기화
        if sys.platform.startswith('win'):
            try:
                import colorama
                colorama.init()
            except ImportError:
                pass
    
    def setup_root_logger(self):
        """루트 로거 설정"""
        root_logger = logging.getLogger()
        root_logger.setLevel(self.log_level)
        
        # 기존 핸들러 제거
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        
        # 콘솔 핸들러 추가
        console_handler = self._create_console_handler()
        root_logger.addHandler(console_handler)
        
        # 파일 핸들러 추가
        file_handler = self._create_file_handler("root")
        root_logger.addHandler(file_handler)
        
        return root_logger
    
    def setup_module_logger(self, module_name: str, 
                          level: Optional[str] = None,
                          file_logging: bool = True,
                          console_logging: bool = True) -> logging.Logger:
        """모듈별 로거 설정"""
        
        logger = logging.getLogger(module_name)
        logger.setLevel(level and getattr(logging, level.upper()) or self.log_level)
        
        # 기존 핸들러 제거
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
        
        # 콘솔 핸들러
        if console_logging:
            console_handler = self._create_console_handler()
            logger.addHandler(console_handler)
        
        # 파일 핸들러
        if file_logging:
            file_handler = self._create_file_handler(module_name)
            logger.addHandler(file_handler)
        
        # 상위 로거로 전파 방지 (중복 방지)
        logger.propagate = False
        
        self.loggers[module_name] = logger
        return logger
    
    def _create_console_handler(self) -> logging.Handler:
        """콘솔 핸들러 생성"""
        handler = logging.StreamHandler(sys.stdout)
        
        # 컬러 포매터 사용
        formatter = ColoredFormatter(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%H:%M:%S'
        )
        
        handler.setFormatter(formatter)
        handler.setLevel(self.log_level)
        
        # 성능 필터 추가
        handler.addFilter(PerformanceFilter())
        
        return handler
    
    def _create_file_handler(self, name: str) -> logging.Handler:
        """파일 핸들러 생성"""
        log_file = self.log_dir / f"{name}.log"
        
        # 회전 파일 핸들러
        file_handler = SafeFileHandler(
            str(log_file),
            maxBytes=10*1024*1024,  # 10MB
            backupCount=5,
            encoding='utf-8'
        )
        
        # 파일용 포매터 (컬러 없음)
        formatter = SafeFormatter(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler.setFormatter(formatter)
        file_handler.setLevel(self.log_level)
        
        # 비동기 핸들러로 감싸기 (성능 향상)
        async_handler = AsyncHandler(file_handler)
        
        return async_handler
    
    def create_error_logger(self) -> logging.Logger:
        """에러 전용 로거 생성"""
        error_logger = logging.getLogger("errors")
        error_logger.setLevel(logging.ERROR)
        
        # 에러 파일 핸들러
        error_file = self.log_dir / "errors.log"
        error_handler = SafeFileHandler(
            str(error_file),
            maxBytes=5*1024*1024,  # 5MB
            backupCount=10,
            encoding='utf-8'
        )
        
        # 구조화된 포매터 사용
        error_formatter = StructuredFormatter()
        error_handler.setFormatter(error_formatter)
        
        # ERROR 이상만 로깅
        error_handler.addFilter(LevelFilter(min_level=logging.ERROR))
        
        error_logger.addHandler(error_handler)
        error_logger.propagate = False
        
        return error_logger
    
    def shutdown_all_loggers(self):
        """모든 로거 종료"""
        # 모든 핸들러 종료
        for logger in self.loggers.values():
            for handler in logger.handlers:
                handler.close()
        
        # 루트 로거 핸들러 종료
        root_logger = logging.getLogger()
        for handler in root_logger.handlers:
            handler.close()

# ========== 로깅 데코레이터 ==========

def log_function_call(logger: Optional[logging.Logger] = None, level: int = logging.DEBUG):
    """함수 호출을 로깅하는 데코레이터"""
    def decorator(func):
        nonlocal logger
        if logger is None:
            logger = logging.getLogger(func.__module__)
        
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            func_name = func.__name__
            
            # 함수 시작 로깅
            logger.log(level, f"{func_name} 시작")
            
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time
                
                # 성공 로깅
                logger.log(level, f"{func_name} 완료 ({execution_time:.3f}초)")
                return result
                
            except Exception as e:
                execution_time = time.time() - start_time
                
                # 실패 로깅
                logger.error(f"{func_name} 실패 ({execution_time:.3f}초): {safe_string(e)}")
                raise
        
        return wrapper
    return decorator

def log_exceptions(logger: Optional[logging.Logger] = None):
    """예외를 로깅하는 데코레이터"""
    def decorator(func):
        nonlocal logger
        if logger is None:
            logger = logging.getLogger(func.__module__)
        
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                logger.exception(f"{func.__name__}에서 예외 발생: {safe_string(e)}")
                raise
        
        return wrapper
    return decorator

# ========== 전역 설정 함수 ==========

def setup_logging(base_dir: str = ".", 
                 log_level: str = "INFO",
                 console_logging: bool = True,
                 file_logging: bool = True) -> LoggerSetup:
    """전역 로깅 설정"""
    
    # 로거 설정 인스턴스 생성
    logger_setup = LoggerSetup(base_dir, log_level)
    
    # 루트 로거 설정
    if console_logging or file_logging:
        logger_setup.setup_root_logger()
    
    # 모듈별 로거들 설정
    loggers_to_setup = [
        "robot_controller",
        "yolo_detector",
        "ui_components",
        "main"
    ]
    
    for module_name in loggers_to_setup:
        logger_setup.setup_module_logger(
            module_name,
            file_logging=file_logging,
            console_logging=console_logging
        )
    
    # 특수 로거들 생성
    error_logger = logger_setup.create_error_logger()
    
    # 초기 로그 메시지
    main_logger = logging.getLogger("main")
    main_logger.info("=" * 50)
    main_logger.info("로깅 시스템 초기화 완료")
    main_logger.info(f"로그 레벨: {log_level}")
    main_logger.info(f"로그 디렉토리: {logger_setup.log_dir}")
    main_logger.info("=" * 50)
    
    return logger_setup

def get_logger(name: str) -> logging.Logger:
    """로거 인스턴스 반환"""
    return logging.getLogger(name)

# ========== 초기화 ==========

# 전역 로거 설정 인스턴스 (필요시 사용)
_global_logger_setup: Optional[LoggerSetup] = None

def initialize_logging(base_dir: str = ".", log_level: str = "INFO") -> LoggerSetup:
    """로깅 시스템 초기화"""
    global _global_logger_setup
    
    if _global_logger_setup is None:
        _global_logger_setup = setup_logging(base_dir, log_level)
    
    return _global_logger_setup

def shutdown_logging():
    """로깅 시스템 종료"""
    global _global_logger_setup
    
    if _global_logger_setup:
        _global_logger_setup.shutdown_all_loggers()
        _global_logger_setup = None

# ========== 테스트 함수 ==========

def test_logging():
    """로깅 시스템 테스트"""
    print("🧪 로깅 시스템 테스트 시작")
    
    # 로깅 초기화
    logger_setup = initialize_logging(".", "DEBUG")
    
    # 각종 로거 테스트
    main_logger = get_logger("main")
    robot_logger = get_logger("robot_controller")
    error_logger = get_logger("errors")
    
    # 다양한 레벨 테스트
    main_logger.debug("디버그 메시지 테스트 🐛")
    main_logger.info("정보 메시지 테스트 ℹ️")
    main_logger.warning("경고 메시지 테스트 ⚠️")
    main_logger.error("에러 메시지 테스트 ❌")
    
    # 한글 메시지 테스트
    robot_logger.info("로봇 연결 테스트 🤖")
    robot_logger.info("한글 메시지가 제대로 출력되는지 확인")
    
    # 예외 테스트
    try:
        raise ValueError("테스트 예외입니다")
    except Exception as e:
        error_logger.exception("예외 테스트")
    
    print("✅ 로깅 시스템 테스트 완료")
    print(f"로그 파일 위치: {logger_setup.log_dir}")

if __name__ == "__main__":
    test_logging()
