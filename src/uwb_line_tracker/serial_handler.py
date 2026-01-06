"""
UWB Serial Communication Handler.
Handles serial port connection, command sending, and data parsing for UWB devices.
Supports both anchor self-calibration and tag positioning modes.
"""

from __future__ import annotations

import threading
import time
import queue
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict, Any
import serial
import serial.tools.list_ports


@dataclass
class RangeData:
    """Parsed range data from UWB device."""
    message_type: str  # 'ma', 'mc', 'mi'
    user_byte: str
    ranges: List[int]  # Distance values in mm (-1 for invalid)
    timestamp: str
    role: str  # 'a' for anchor, 't' for tag
    base_station_id: str
    tag_id: Optional[str] = None
    raw_data: str = ""


@dataclass
class AnchorRangeData:
    """Anchor-to-anchor range data for self-calibration."""
    anchor_id: int
    ranges: Dict[int, int]  # {other_anchor_id: distance_mm}
    timestamp: float


class UWBSerialHandler:
    """
    UWB Serial Communication Handler.
    
    Supports:
    - Serial port connection/disconnection
    - Sending calibration commands ($ancrangestart, $ancrangestop)
    - Parsing range data (ma/mc/mi messages)
    - Callback-based data notification
    """
    
    def __init__(self, port: str = "", baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._data_queue = queue.Queue(maxsize=1000)
        
        # Callbacks
        self._range_callback: Optional[Callable[[RangeData], None]] = None
        self._anchor_range_callback: Optional[Callable[[AnchorRangeData], None]] = None
        
        # State
        self._is_calibrating = False
        self._buffer = ""
    
    @staticmethod
    def list_available_ports() -> List[str]:
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def set_range_callback(self, callback: Callable[[RangeData], None]):
        """Set callback for range data (for positioning)."""
        self._range_callback = callback
    
    def set_anchor_range_callback(self, callback: Callable[[AnchorRangeData], None]):
        """Set callback for anchor range data (for calibration)."""
        self._anchor_range_callback = callback
    
    def connect(self, port: Optional[str] = None) -> bool:
        """Connect to the serial port."""
        if port:
            self.port = port
        
        if not self.port:
            print("[Serial] No port specified")
            return False
        
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"[Serial] Connected to {self.port}")
            return True
        except Exception as e:
            print(f"[Serial] Failed to connect: {e}")
            self._serial = None
            return False
    
    def disconnect(self):
        """Disconnect from the serial port."""
        self.stop()
        if self._serial:
            try:
                self._serial.close()
            except:
                pass
            self._serial = None
        print("[Serial] Disconnected")
    
    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self._serial is not None and self._serial.is_open
    
    def start(self) -> bool:
        """Start the receive thread."""
        if not self.is_connected():
            return False
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print("[Serial] Receive thread started")
        return True
    
    def stop(self):
        """Stop the receive thread."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        print("[Serial] Receive thread stopped")
    
    def send_command(self, command: str) -> bool:
        """Send a command to the UWB device."""
        if not self.is_connected():
            return False
        
        try:
            if not command.endswith('\r\n'):
                command += '\r\n'
            self._serial.write(command.encode('utf-8'))
            print(f"[Serial] Sent: {command.strip()}")
            return True
        except Exception as e:
            print(f"[Serial] Failed to send command: {e}")
            return False
    
    def start_calibration(self) -> bool:
        """Start anchor self-calibration mode."""
        if self.send_command('$ancrangestart'):
            self._is_calibrating = True
            self._debug_count = 0  # Reset debug counter
            print("[Serial] Calibration mode started")
            return True
        return False
    
    def stop_calibration(self) -> bool:
        """Stop anchor self-calibration mode."""
        if self.send_command('$ancrangestop'):
            self._is_calibrating = False
            print("[Serial] Calibration mode stopped")
            return True
        return False
    
    @property
    def is_calibrating(self) -> bool:
        return self._is_calibrating
    
    def _receive_loop(self):
        """Main receive loop running in a separate thread."""
        while not self._stop_event.is_set():
            try:
                if self._serial and self._serial.in_waiting > 0:
                    data = self._serial.read_until(b'\r\n')
                    self._buffer += data.decode('utf-8', errors='ignore')
                    
                    # Process complete frames
                    if '\r\n' in self._buffer:
                        frames = self._buffer.split('\r\n')
                        for frame in frames[:-1]:  # Process all complete frames
                            if frame.strip():
                                self._process_frame(frame.strip())
                        self._buffer = frames[-1]  # Keep incomplete data
                else:
                    time.sleep(0.01)  # Small sleep to prevent CPU spinning
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[Serial] Receive error: {e}")
                time.sleep(0.1)
    
    def _process_frame(self, frame: str):
        """Process a single data frame."""
        try:
            parsed = self._parse_data(frame)
            if parsed:
                # Debug: Print first few parsed messages during calibration
                if self._is_calibrating and hasattr(self, '_debug_count'):
                    self._debug_count += 1
                    if self._debug_count <= 3:
                        print(f"[Serial Debug] Parsed: role={parsed.role}, id={parsed.base_station_id}, ranges={parsed.ranges[:4]}")
                
                # Notify callbacks
                if self._range_callback:
                    self._range_callback(parsed)
                
                # For calibration mode, extract anchor-anchor ranges
                if self._is_calibrating and self._anchor_range_callback:
                    anchor_data = self._extract_anchor_ranges(parsed)
                    if anchor_data:
                        self._anchor_range_callback(anchor_data)
        except Exception as e:
            print(f"[Serial] Parse error: {e}")
    
    def _parse_data(self, data_str: str) -> Optional[RangeData]:
        """Parse received data string into RangeData."""
        if not data_str:
            return None
        
        # Check for valid message types
        if not (data_str.startswith('mc') or data_str.startswith('mi') or data_str.startswith('ma')):
            return None
        
        parts = data_str.split(' ')
        if len(parts) < 5:
            return None
        
        message_type = parts[0]
        
        try:
            if message_type == 'ma':
                length = len(parts)
                
                if length == 11:  # 4 distance values
                    role_info = parts[9]
                    ranges = [int(parts[i], 16) if parts[i] != '-1' else -1 for i in range(2, 6)]
                    timestamp = parts[-1]
                elif length == 15:  # 8 distance values
                    role_info = parts[13]
                    ranges = [int(parts[i], 16) if parts[i] != '-1' else -1 for i in range(2, 10)]
                    timestamp = parts[-1]
                else:
                    return None
                
                # Parse role info: first char is role, rest is base_station_id:tag_id
                role = role_info[0] if role_info else ''
                id_parts = role_info[1:].split(':') if len(role_info) > 1 else ['', '']
                base_station_id = id_parts[0] if len(id_parts) > 0 else ''
                tag_id = id_parts[1] if len(id_parts) > 1 else None
                
                return RangeData(
                    message_type=message_type,
                    user_byte=parts[1] if len(parts) > 1 else '',
                    ranges=ranges,
                    timestamp=timestamp,
                    role=role,
                    base_station_id=base_station_id,
                    tag_id=tag_id,
                    raw_data=data_str
                )
            
            elif message_type in ['mc', 'mi']:
                # Similar parsing for mc/mi messages
                length = len(parts)
                
                if length >= 11:
                    # Try to parse as 4-anchor format
                    try:
                        ranges = [int(parts[i], 16) if parts[i] != '-1' else -1 for i in range(2, 6)]
                        role_info = parts[9] if length > 9 else ''
                        timestamp = parts[-1]
                    except:
                        return None
                elif length >= 7:
                    # Shorter format
                    ranges = [int(parts[i], 16) if parts[i] != '-1' else -1 for i in range(2, min(6, length-1))]
                    role_info = parts[-2] if length > 2 else ''
                    timestamp = parts[-1]
                else:
                    return None
                
                role = role_info[0] if role_info else ''
                id_parts = role_info[1:].split(':') if len(role_info) > 1 else ['', '']
                base_station_id = id_parts[0] if len(id_parts) > 0 else ''
                tag_id = id_parts[1] if len(id_parts) > 1 else None
                
                return RangeData(
                    message_type=message_type,
                    user_byte=parts[1] if len(parts) > 1 else '',
                    ranges=ranges,
                    timestamp=timestamp,
                    role=role,
                    base_station_id=base_station_id,
                    tag_id=tag_id,
                    raw_data=data_str
                )
        
        except Exception as e:
            print(f"[Serial] Parse error for '{data_str}': {e}")
            return None
        
        return None
    
    def _extract_anchor_ranges(self, data: RangeData) -> Optional[AnchorRangeData]:
        """Extract anchor-to-anchor ranges for calibration."""
        if data.role != 'a':  # Only process anchor data
            return None
        
        try:
            anchor_id = int(data.base_station_id)
        except:
            return None
        
        # Build range dictionary (only valid distances)
        # Valid distance: > 0 and < 100m (100000mm)
        # This filters out invalid/corrupted values
        MAX_VALID_DISTANCE_MM = 100000  # 100 meters max
        MIN_VALID_DISTANCE_MM = 10      # 1 cm min (filter noise)
        
        ranges = {}
        for i, dist in enumerate(data.ranges):
            if MIN_VALID_DISTANCE_MM < dist < MAX_VALID_DISTANCE_MM:
                ranges[i] = dist
        
        if not ranges:
            return None
        
        return AnchorRangeData(
            anchor_id=anchor_id,
            ranges=ranges,
            timestamp=time.time()
        )


# Test code
if __name__ == "__main__":
    print("Available ports:", UWBSerialHandler.list_available_ports())
    
    handler = UWBSerialHandler()
    
    def on_range(data: RangeData):
        print(f"Range: {data}")
    
    def on_anchor_range(data: AnchorRangeData):
        print(f"Anchor range: {data}")
    
    handler.set_range_callback(on_range)
    handler.set_anchor_range_callback(on_anchor_range)
    
    # Example: connect to COM14
    # handler.connect("COM14")
    # handler.start()
    # handler.start_calibration()
    # time.sleep(10)
    # handler.stop_calibration()
    # handler.disconnect()

