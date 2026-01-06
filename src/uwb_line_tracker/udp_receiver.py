"""
UDP receiver for UWB position data from the host software.
"""

from __future__ import annotations

import json
import socket
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Callable


@dataclass
class UWBTagData:
    """Data received from UWB system."""
    tag_id: int
    x: float
    y: float
    z: float
    timestamp: float = 0.0


@dataclass
class AnchorRangeData:
    """Inter-anchor range measurement data."""
    anchor_i: int
    anchor_j: int
    distance_mm: int  # Distance in millimeters
    timestamp: float = 0.0


class UDPReceiver:
    """
    UDP receiver for UWB position data and anchor range measurements.
    
    Expected data formats:
    1. Tag position:
    {
        "Command": "UpLink",
        "TagID": 0,
        "X": 1.089,
        "Y": 1.056,
        "Z": 1.539
    }
    
    2. Anchor range (inter-anchor distance):
    {
        "Command": "AnchorRange",
        "AnchorI": 0,
        "AnchorJ": 1,
        "Distance": 25000  // in millimeters
    }
    
    3. Range data with multiple measurements:
    {
        "Command": "RangeData",
        "Ranges": [
            {"I": 0, "J": 1, "Distance": 25000},
            {"I": 0, "J": 2, "Distance": 30000},
            {"I": 1, "J": 2, "Distance": 28000}
        ]
    }
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 5000):
        self.host = host
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._data_queue: queue.Queue[UWBTagData] = queue.Queue(maxsize=100)
        self._range_queue: queue.Queue[AnchorRangeData] = queue.Queue(maxsize=100)
        self._callback: Optional[Callable[[UWBTagData], None]] = None
        self._range_callback: Optional[Callable[[AnchorRangeData], None]] = None
        self._is_running = False
    
    @property
    def is_running(self) -> bool:
        return self._is_running
    
    def set_callback(self, callback: Callable[[UWBTagData], None]) -> None:
        """Set callback function to be called when tag data is received."""
        self._callback = callback
    
    def set_range_callback(self, callback: Callable[[AnchorRangeData], None]) -> None:
        """Set callback function to be called when anchor range data is received."""
        self._range_callback = callback
    
    def start(self) -> bool:
        """Start the UDP receiver."""
        if self._is_running:
            return True
        
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((self.host, self.port))
            self._socket.settimeout(1.0)
            
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._thread.start()
            self._is_running = True
            print(f"[UDP] Listening on {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[UDP] Failed to start: {e}")
            return False
    
    def stop(self) -> None:
        """Stop the UDP receiver."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
        self._is_running = False
        print("[UDP] Stopped")
    
    def _receive_loop(self) -> None:
        """Main receive loop running in background thread."""
        import time
        
        while not self._stop_event.is_set():
            try:
                data, addr = self._socket.recvfrom(4096)
                self._process_data(data, time.time())
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[UDP] Receive error: {e}")
    
    def _process_data(self, data: bytes, timestamp: float) -> None:
        """Process received UDP data."""
        try:
            json_str = data.decode('utf-8').strip()
            payload = json.loads(json_str)
            
            command = payload.get("Command", "")
            
            # Tag position data
            if command == "UpLink":
                tag_data = UWBTagData(
                    tag_id=payload.get("TagID", 0),
                    x=float(payload.get("X", 0.0)),
                    y=float(payload.get("Y", 0.0)),
                    z=float(payload.get("Z", 0.0)),
                    timestamp=timestamp
                )
                
                # Put in queue (non-blocking)
                try:
                    self._data_queue.put_nowait(tag_data)
                except queue.Full:
                    # Drop oldest data
                    try:
                        self._data_queue.get_nowait()
                        self._data_queue.put_nowait(tag_data)
                    except:
                        pass
                
                # Call callback if set
                if self._callback:
                    self._callback(tag_data)
            
            # Single anchor range measurement
            elif command == "AnchorRange":
                range_data = AnchorRangeData(
                    anchor_i=int(payload.get("AnchorI", 0)),
                    anchor_j=int(payload.get("AnchorJ", 0)),
                    distance_mm=int(payload.get("Distance", 0)),
                    timestamp=timestamp
                )
                
                # Put in range queue
                try:
                    self._range_queue.put_nowait(range_data)
                except queue.Full:
                    try:
                        self._range_queue.get_nowait()
                        self._range_queue.put_nowait(range_data)
                    except:
                        pass
                
                # Call range callback if set
                if self._range_callback:
                    self._range_callback(range_data)
            
            # Multiple range measurements
            elif command == "RangeData":
                ranges = payload.get("Ranges", [])
                for r in ranges:
                    range_data = AnchorRangeData(
                        anchor_i=int(r.get("I", 0)),
                        anchor_j=int(r.get("J", 0)),
                        distance_mm=int(r.get("Distance", 0)),
                        timestamp=timestamp
                    )
                    
                    # Put in range queue
                    try:
                        self._range_queue.put_nowait(range_data)
                    except queue.Full:
                        try:
                            self._range_queue.get_nowait()
                            self._range_queue.put_nowait(range_data)
                        except:
                            pass
                    
                    # Call range callback if set
                    if self._range_callback:
                        self._range_callback(range_data)
                    
        except json.JSONDecodeError as e:
            print(f"[UDP] JSON decode error: {e}")
        except Exception as e:
            print(f"[UDP] Process error: {e}")
    
    def get_latest_data(self) -> Optional[UWBTagData]:
        """Get the latest received data (non-blocking)."""
        latest = None
        while not self._data_queue.empty():
            try:
                latest = self._data_queue.get_nowait()
            except queue.Empty:
                break
        return latest
    
    def get_all_data(self) -> list[UWBTagData]:
        """Get all queued tag data."""
        data_list = []
        while not self._data_queue.empty():
            try:
                data_list.append(self._data_queue.get_nowait())
            except queue.Empty:
                break
        return data_list
    
    def get_latest_range(self) -> Optional[AnchorRangeData]:
        """Get the latest anchor range data (non-blocking)."""
        latest = None
        while not self._range_queue.empty():
            try:
                latest = self._range_queue.get_nowait()
            except queue.Empty:
                break
        return latest
    
    def get_all_ranges(self) -> list[AnchorRangeData]:
        """Get all queued anchor range data."""
        range_list = []
        while not self._range_queue.empty():
            try:
                range_list.append(self._range_queue.get_nowait())
            except queue.Empty:
                break
        return range_list




