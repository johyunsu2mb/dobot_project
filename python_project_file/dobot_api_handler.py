# improved_dobot_api_handler.py
import socket
import time
import threading
import logging
import atexit
import signal
import sys
from contextlib import contextmanager
from typing import Optional, Tuple, Dict, Any

logger = logging.getLogger(__name__)

class ImprovedDobotAPIHandler:
    """개선된 Dobot API 핸들러 - 통신 안정성 문제 해결"""
    
    def __init__(self, ip_address: str = "192.168.1.6", 
                 dashboard_port: int = 29999, 
                 move_port: int = 30003,
                 feed_port: int = 30004):
        self.ip_address = ip_address
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.feed_port = feed_port
        
        # 소켓 연결 관리
        self.dashboard_socket: Optional[socket.socket] = None
        self.move_socket: Optional[socket.socket] = None
        self.feed_socket: Optional[socket.socket] = None
        
        # 연결 상태 관리
        self.is_connected = False
        self.connection_lock = threading.Lock()
        
        # 자동 정리 등록
        atexit.register(self.cleanup_all_connections)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """프로그램 종료시 신호 처리"""
        logger.info(f"신호 {signum} 받음. 연결 정리 중...")
        self.cleanup_all_connections()
        sys.exit(0)
    
    def _create_socket_with_options(self) -> socket.socket:
        """소켓 생성 및 옵션 설정"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # 중요: 소켓 재사용 옵션 설정
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Windows에서 SO_REUSEPORT 지원하는 경우
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass  # Windows에서는 지원하지 않을 수 있음
        
        # 타임아웃 설정
        sock.settimeout(10.0)
        
        # Keep-alive 설정
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        
        return sock
    
    def _safe_socket_close(self, sock: Optional[socket.socket], name: str):
        """안전한 소켓 종료"""
        if sock is not None:
            try:
                # 우아한 종료 시도
                sock.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error) as e:
                logger.debug(f"{name} 소켓 shutdown 실패: {e}")
            
            try:
                sock.close()
                logger.info(f"{name} 소켓 정상 종료")
            except (OSError, socket.error) as e:
                logger.warning(f"{name} 소켓 종료 중 오류: {e}")
    
    def cleanup_all_connections(self):
        """모든 연결 정리"""
        with self.connection_lock:
            logger.info("모든 Dobot 연결 정리 시작...")
            
            # 각 소켓 정리
            self._safe_socket_close(self.dashboard_socket, "Dashboard")
            self._safe_socket_close(self.move_socket, "Move")
            self._safe_socket_close(self.feed_socket, "Feed")
            
            # 소켓 참조 제거
            self.dashboard_socket = None
            self.move_socket = None
            self.feed_socket = None
            
            self.is_connected = False
            
            # 소켓이 완전히 정리될 때까지 대기
            time.sleep(0.5)
            logger.info("모든 연결 정리 완료")
    
    def connect_with_retry(self, max_retries: int = 3, retry_delay: float = 2.0) -> bool:
        """재시도 로직이 있는 연결"""
        for attempt in range(max_retries):
            try:
                logger.info(f"Dobot 연결 시도 {attempt + 1}/{max_retries}")
                
                # 기존 연결이 있다면 정리
                if self.is_connected:
                    self.cleanup_all_connections()
                    time.sleep(1.0)  # 정리 후 대기
                
                success = self._connect_all_sockets()
                if success:
                    self.is_connected = True
                    logger.info("Dobot 연결 성공!")
                    return True
                    
            except Exception as e:
                logger.warning(f"연결 시도 {attempt + 1} 실패: {e}")
                
                # 실패한 연결 정리
                self.cleanup_all_connections()
                
                if attempt < max_retries - 1:
                    logger.info(f"{retry_delay}초 후 재시도...")
                    time.sleep(retry_delay)
        
        logger.error("모든 연결 시도 실패")
        return False
    
    def _connect_all_sockets(self) -> bool:
        """모든 소켓 연결"""
        try:
            # Dashboard 소켓 연결
            self.dashboard_socket = self._create_socket_with_options()
            self.dashboard_socket.connect((self.ip_address, self.dashboard_port))
            logger.info(f"Dashboard 연결 성공: {self.ip_address}:{self.dashboard_port}")
            
            # Move 소켓 연결
            self.move_socket = self._create_socket_with_options()
            self.move_socket.connect((self.ip_address, self.move_port))
            logger.info(f"Move 연결 성공: {self.ip_address}:{self.move_port}")
            
            # Feed 소켓 연결 (선택적)
            try:
                self.feed_socket = self._create_socket_with_options()
                self.feed_socket.connect((self.ip_address, self.feed_port))
                logger.info(f"Feed 연결 성공: {self.ip_address}:{self.feed_port}")
            except Exception as e:
                logger.warning(f"Feed 소켓 연결 실패 (계속 진행): {e}")
                self.feed_socket = None
            
            return True
            
        except Exception as e:
            logger.error(f"소켓 연결 실패: {e}")
            self.cleanup_all_connections()
            return False
    
    @contextmanager
    def connection_context(self):
        """Context manager로 연결 관리"""
        try:
            if not self.connect_with_retry():
                raise ConnectionError("Dobot 연결 실패")
            yield self
        finally:
            self.cleanup_all_connections()
    
    def send_command(self, command: str, socket_type: str = "dashboard") -> Optional[str]:
        """안전한 명령 전송"""
        with self.connection_lock:
            if not self.is_connected:
                logger.error("로봇이 연결되지 않음")
                return None
            
            # 소켓 선택
            sock = None
            if socket_type == "dashboard":
                sock = self.dashboard_socket
            elif socket_type == "move":
                sock = self.move_socket
            elif socket_type == "feed":
                sock = self.feed_socket
            
            if sock is None:
                logger.error(f"{socket_type} 소켓이 없음")
                return None
            
            try:
                # 명령 전송
                sock.send(command.encode('utf-8'))
                
                # 응답 수신
                response = sock.recv(1024).decode('utf-8').strip()
                logger.debug(f"명령: {command} | 응답: {response}")
                return response
                
            except socket.timeout:
                logger.error(f"명령 타임아웃: {command}")
                return None
            except (socket.error, ConnectionError) as e:
                logger.error(f"명령 전송 실패: {command}, 오류: {e}")
                self.is_connected = False
                return None
    
    def check_connection_health(self) -> bool:
        """연결 상태 확인"""
        try:
            response = self.send_command("GetPose()", "dashboard")
            return response is not None and "ERROR" not in response.upper()
        except:
            return False
    
    def __enter__(self):
        """Context manager 진입"""
        if not self.connect_with_retry():
            raise ConnectionError("Dobot 연결 실패")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 종료"""
        self.cleanup_all_connections()
