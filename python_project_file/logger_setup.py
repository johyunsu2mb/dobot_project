#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로깅 시스템 설정
파일 로깅, 콘솔 로깅, GUI 로깅을 통합 관리
"""

import logging
import logging.handlers
import os
import sys
from typing import Optional
from config import AppConfig


def setup_logger(name: str = __name__, level: str = None) -> logging.Logger:
    """
    통합 로거 설정
    
    Args:
        name: 로거 이름
        level: 로그 레벨 ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL')
    
    Returns:
        설정된 로거 인스턴스
    """
    # 로그 레벨 설정
    if level is None:
        level = AppConfig.LOG_LEVEL
    
    log_level = getattr(logging, level.upper(), logging.INFO)
    
    # 로거 생성
    logger = logging.getLogger(name)
    logger.setLevel(log_level)
    
    # 이미 핸들러가 있으면 중복 생성 방지
    if logger.handlers:
        return logger
    
    # 포매터 생성
    formatter = logging.Formatter(
        fmt=AppConfig.LOG_FORMAT,
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 콘솔 핸들러 설정
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 파일 핸들러 설정
    try:
        # 로그 디렉토리 생성
        os.makedirs(AppConfig.LOG_DIR, exist_ok=True)
        
        # 회전 파일 핸들러 (크기 기반)
        file_handler = logging.handlers.RotatingFileHandler(
            filename=AppConfig.get_log_file_path(),
            maxBytes=AppConfig.MAX_LOG_SIZE,
            backupCount=AppConfig.LOG_BACKUP_COUNT,
            encoding='utf-8'
        )
        file_handler.setLevel(log_level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
    except (OSError, PermissionError) as e:
        logger.warning(f"파일 로깅을 설정할 수 없습니다: {e}")
    
    # 루트 로거 설정 (한 번만)
    if name == __name__ or name == 'DobotApp':
        _setup_root_logger(log_level, formatter)
    
    logger.info(f"로거 '{name}' 초기화 완료 (레벨: {level})")
    return logger


def _setup_root_logger(log_level: int, formatter: logging.Formatter):
    """루트 로거 설정"""
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    
    # 기존 핸들러 제거 (중복 방지)
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)


class SafeFileHandler(logging.handlers.RotatingFileHandler):
    """안전한 파일 핸들러 (한글/특수문자 지원)"""
    
    def __init__(self, *args, **kwargs):
        # 인코딩을 UTF-8로 강제 설정
        kwargs['encoding'] = 'utf-8'
        super().__init__(*args, **kwargs)
    
    def emit(self, record):
        """안전한 로그 레코드 출력"""
        try:
            # 메시지에서 문제가 될 수 있는 문자 처리
            if hasattr(record, 'getMessage'):
                msg = record.getMessage()
                # 특수 문자나 이모지 등을 안전하게 처리
                safe_msg = msg.encode('utf-8', errors='replace').decode('utf-8')
                record.msg = safe_msg
            
            super().emit(record)
            
        except Exception as e:
            # 로깅 자체가 실패할 경우 콘솔에 출력
            print(f"로그 출력 실패: {e}", file=sys.stderr)


class ColoredConsoleHandler(logging.StreamHandler):
    """컬러 콘솔 핸들러"""
    
    # ANSI 색상 코드
    COLORS = {
        logging.DEBUG: '\033[36m',    # 청록색
        logging.INFO: '\033[32m',     # 녹색
        logging.WARNING: '\033[33m',  # 노란색
        logging.ERROR: '\033[31m',    # 빨간색
        logging.CRITICAL: '\033[35m'  # 자홍색
    }
    RESET = '\033[0m'
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Windows 환경에서 ANSI 색상 지원 확인
        self.use_colors = self._supports_color()
    
    def _supports_color(self) -> bool:
        """색상 지원 여부 확인"""
        # Windows 10 이상에서 ANSI 지원
        if sys.platform == "win32":
            try:
                import os
                # Windows Terminal이나 PowerShell Core에서는 색상 지원
                return os.environ.get('WT_SESSION') is not None or \
                       os.environ.get('TERM_PROGRAM') == 'vscode'
            except Exception:
                return False
        return True
    
    def format(self, record):
        """색상이 적용된 로그 포맷"""
        if self.use_colors and record.levelno in self.COLORS:
            # 레벨 이름에만 색상 적용
            record.levelname = (
                f"{self.COLORS[record.levelno]}"
                f"{record.levelname}"
                f"{self.RESET}"
            )
        
        return super().format(record)


class MemoryHandler(logging.Handler):
    """메모리 기반 로그 핸들러 (GUI용)"""
    
    def __init__(self, max_records: int = 1000):
        super().__init__()
        self.max_records = max_records
        self.records = []
        self.callbacks = []
    
    def emit(self, record):
        """로그 레코드를 메모리에 저장"""
        try:
            # 포맷된 메시지 생성
            formatted_message = self.format(record)
            
            # 레코드 저장
            self.records.append({
                'timestamp': record.created,
                'level': record.levelname,
                'message': formatted_message,
                'name': record.name
            })
            
            # 최대 레코드 수 제한
            if len(self.records) > self.max_records:
                self.records.pop(0)
            
            # 콜백 함수 호출 (GUI 업데이트 등)
            for callback in self.callbacks:
                try:
                    callback(record.levelname, formatted_message)
                except Exception:
                    pass  # 콜백 실패 시 무시
                    
        except Exception:
            pass  # 메모리 핸들러 실패 시 무시
    
    def add_callback(self, callback):
        """GUI 업데이트 콜백 추가"""
        self.callbacks.append(callback)
    
    def get_records(self, level: str = None, limit: int = None) -> list:
        """저장된 로그 레코드 반환"""
        records = self.records
        
        # 레벨 필터링
        if level:
            records = [r for r in records if r['level'] == level]
        
        # 개수 제한
        if limit:
            records = records[-limit:]
        
        return records
    
    def clear_records(self):
        """저장된 레코드 초기화"""
        self.records.clear()


def setup_gui_logging(gui_callback=None) -> MemoryHandler:
    """GUI용 로깅 설정"""
    memory_handler = MemoryHandler(max_records=AppConfig.MAX_LOG_ENTRIES_GUI)
    
    if gui_callback:
        memory_handler.add_callback(gui_callback)
    
    # 포매터 설정
    formatter = logging.Formatter('%(name)s: %(message)s')
    memory_handler.setFormatter(formatter)
    
    # 루트 로거에 추가
    logging.getLogger().addHandler(memory_handler)
    
    return memory_handler


def setup_debug_logging():
    """디버그용 상세 로깅 설정"""
    if not AppConfig.DEBUG_MODE:
        return
    
    # 모든 모듈의 로그 레벨을 DEBUG로 설정
    logging.getLogger().setLevel(logging.DEBUG)
    
    # 상세 포매터
    debug_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - '
        '%(filename)s:%(lineno)d - %(funcName)s - %(message)s'
    )
    
    # 디버그 파일 핸들러
    try:
        debug_handler = SafeFileHandler(
            filename=os.path.join(AppConfig.LOG_DIR, 'debug.log'),
            maxBytes=50 * 1024 * 1024,  # 50MB
            backupCount=3,
            encoding='utf-8'
        )
        debug_handler.setLevel(logging.DEBUG)
        debug_handler.setFormatter(debug_formatter)
        
        logging.getLogger().addHandler(debug_handler)
        
    except Exception as e:
        print(f"디버그 로깅 설정 실패: {e}", file=sys.stderr)


def create_operation_logger(operation_name: str) -> logging.Logger:
    """특정 작업용 로거 생성"""
    logger_name = f"dobot.{operation_name}"
    return setup_logger(logger_name)


def log_function_call(func):
    """함수 호출 로깅 데코레이터"""
    def wrapper(*args, **kwargs):
        logger = logging.getLogger(func.__module__)
        
        # 함수 호출 로그
        if AppConfig.VERBOSE_LOGGING:
            logger.debug(f"함수 호출: {func.__name__}")
        
        try:
            result = func(*args, **kwargs)
            
            # 성공 로그
            if AppConfig.VERBOSE_LOGGING:
                logger.debug(f"함수 완료: {func.__name__}")
            
            return result
            
        except Exception as e:
            # 에러 로그
            logger.error(f"함수 {func.__name__} 실행 중 오류: {e}")
            raise
    
    return wrapper


def log_performance(operation: str):
    """성능 로깅 데코레이터"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if not AppConfig.ENABLE_PROFILING:
                return func(*args, **kwargs)
            
            import time
            logger = logging.getLogger(func.__module__)
            
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time
                
                logger.info(f"{operation} 실행 시간: {execution_time:.3f}초")
                return result
                
            except Exception as e:
                execution_time = time.time() - start_time
                logger.error(f"{operation} 실행 실패 (시간: {execution_time:.3f}초): {e}")
                raise
        
        return wrapper
    return decorator


# 시스템 시작 시 기본 로깅 설정
def initialize_logging():
    """시스템 시작 시 로깅 초기화"""
    try:
        # 기본 로거 설정
        main_logger = setup_logger('DobotSystem')
        
        # 디버그 모드 시 상세 로깅
        if AppConfig.DEBUG_MODE:
            setup_debug_logging()
        
        main_logger.info("로깅 시스템 초기화 완료")
        main_logger.info(f"로그 파일: {AppConfig.get_log_file_path()}")
        
        return main_logger
        
    except Exception as e:
        print(f"로깅 초기화 실패: {e}", file=sys.stderr)
        # 기본 로거라도 반환
        return logging.getLogger('DobotSystem')


# 모듈 로드 시 자동 초기화
if __name__ != "__main__":
    # 라이브러리로 import될 때는 자동 초기화하지 않음
    pass
else:
    # 직접 실행될 때 테스트
    test_logger = initialize_logging()
    test_logger.info("로깅 시스템 테스트")
    test_logger.debug("디버그 메시지")
    test_logger.warning("경고 메시지")
    test_logger.error("에러 메시지")
