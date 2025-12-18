"""
GNSS data receiver module.
Receives GNSS position data via ZMQ from the TCP server.
"""

from __future__ import annotations

import threading
import queue
from dataclasses import dataclass
from typing import Optional, Callable, Dict

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False
    print("Warning: pyzmq not installed. GNSS receiver will not work.")
    print("Install with: pip install pyzmq")


@dataclass
class GNSSData:
    """GNSS data received from devices."""
    client_id: int  # 0, 1, 2 for anchors; other IDs for tags
    lat: float      # Latitude in degrees
    lng: float      # Longitude in degrees
    alt: float = 0.0  # Altitude in meters (optional)
    timestamp: float = 0.0


class GNSSReceiver:
    """
    Receives GNSS data via ZMQ from the TCP server.
    
    Expected data format from tcp_server.py:
    {
        "client_id": 0,
        "lat": 22.31930,
        "lng": 114.16940,
        "alt": 10.0  # optional
    }
    """
    
    ZMQ_CONNECT_ADDR = "tcp://127.0.0.1:5555"
    
    def __init__(self, zmq_addr: str = None):
        self.zmq_addr = zmq_addr or self.ZMQ_CONNECT_ADDR
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._data_queue: queue.Queue[GNSSData] = queue.Queue(maxsize=100)
        self._callback: Optional[Callable[[GNSSData], None]] = None
        self._is_running = False
        
        # Store latest data for each client_id
        self._latest_data: Dict[int, GNSSData] = {}
        self._data_lock = threading.Lock()
    
    @property
    def is_running(self) -> bool:
        return self._is_running
    
    def set_callback(self, callback: Callable[[GNSSData], None]) -> None:
        """Set callback function to be called when data is received."""
        self._callback = callback
    
    def start(self) -> bool:
        """Start the GNSS receiver."""
        if not HAS_ZMQ:
            print("[GNSS] pyzmq not available")
            return False
        
        if self._is_running:
            return True
        
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.PULL)
            self._socket.connect(self.zmq_addr)
            self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._thread.start()
            self._is_running = True
            print(f"[GNSS] Connected to ZMQ at {self.zmq_addr}")
            return True
        except Exception as e:
            print(f"[GNSS] Failed to start: {e}")
            return False
    
    def stop(self) -> None:
        """Stop the GNSS receiver."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
        if self._context:
            try:
                self._context.term()
            except:
                pass
        self._is_running = False
        print("[GNSS] Stopped")
    
    def _receive_loop(self) -> None:
        """Main receive loop running in background thread."""
        import time
        
        while not self._stop_event.is_set():
            try:
                msg = self._socket.recv_json()
                self._process_data(msg, time.time())
            except zmq.Again:
                # Timeout, continue
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[GNSS] Receive error: {e}")
    
    def _process_data(self, msg: dict, timestamp: float) -> None:
        """Process received GNSS data."""
        try:
            # Validate required fields
            if not all(k in msg for k in ['client_id', 'lat', 'lng']):
                print(f"[GNSS] Invalid data: {msg}")
                return
            
            gnss_data = GNSSData(
                client_id=int(msg['client_id']),
                lat=float(msg['lat']),
                lng=float(msg['lng']),
                alt=float(msg.get('alt', 0.0)),
                timestamp=timestamp
            )
            
            # Store latest data for this client_id
            with self._data_lock:
                self._latest_data[gnss_data.client_id] = gnss_data
            
            # Put in queue (non-blocking)
            try:
                self._data_queue.put_nowait(gnss_data)
            except queue.Full:
                try:
                    self._data_queue.get_nowait()
                    self._data_queue.put_nowait(gnss_data)
                except:
                    pass
            
            # Call callback if set
            if self._callback:
                self._callback(gnss_data)
                
        except Exception as e:
            print(f"[GNSS] Process error: {e}")
    
    def get_latest_data(self, client_id: int = None) -> Optional[GNSSData]:
        """
        Get the latest received data.
        If client_id is specified, get data for that client only.
        """
        with self._data_lock:
            if client_id is not None:
                return self._latest_data.get(client_id)
            # Return latest from queue
            latest = None
            while not self._data_queue.empty():
                try:
                    latest = self._data_queue.get_nowait()
                except queue.Empty:
                    break
            return latest
    
    def get_anchor_data(self, anchor_id: int) -> Optional[GNSSData]:
        """Get GNSS data for a specific anchor (0, 1, or 2)."""
        return self.get_latest_data(anchor_id)
    
    def get_all_anchors(self) -> Dict[int, GNSSData]:
        """Get GNSS data for all anchors (0, 1, 2)."""
        with self._data_lock:
            return {
                i: self._latest_data[i]
                for i in [0, 1, 2]
                if i in self._latest_data
            }
    
    def has_all_anchors(self) -> bool:
        """Check if we have GNSS data for all 3 anchors."""
        with self._data_lock:
            return all(i in self._latest_data for i in [0, 1, 2])


