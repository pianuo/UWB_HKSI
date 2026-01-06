"""
Serial receiver for UWB data.
Connects to UWB device via serial port and parses ranging data.
"""

from __future__ import annotations

import threading
import time
import queue
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict, Any

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("Warning: pyserial not installed. Install with: pip install pyserial")


@dataclass
class UWBRangingData:
    """UWB ranging data from serial port."""
    message_type: str  # 'mc' or 'mi'
    user_byte: str
    ranges: List[int]  # Distance values in mm, -1 for invalid
    timestamp: str
    role: str  # 'a' for anchor, 't' for tag
    base_station_id: str
    tag_id: str
    raw_data: str


class SerialReceiver:
    """
    Receives and parses UWB data from serial port.
    
    Data format example:
    mc ff 00001d09 00000f41 00001f8e ffffffff ... 095f c1 00024c24 a0:0
    
    Fields:
    - mc/mi: message type
    - ff: user byte
    - 8 hex values: distances (ffffffff = -1 invalid)
    - role_info: role + base_station_id:tag_id
    - timestamp
    """
    
    def __init__(self, port: str = "COM14", baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._data_queue: queue.Queue[UWBRangingData] = queue.Queue(maxsize=100)
        self._callback: Optional[Callable[[UWBRangingData], None]] = None
        self._is_connected = False
        
        # Calibration mode flag
        self._calibration_mode = False
        self._calibration_data: List[Dict[str, Any]] = []
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected and self._serial is not None and self._serial.is_open
    
    def set_callback(self, callback: Callable[[UWBRangingData], None]):
        """Set callback function for received data."""
        self._callback = callback
    
    def set_port(self, port: str):
        """Set serial port."""
        self.port = port
    
    def connect(self) -> bool:
        """Connect to serial port."""
        if not HAS_SERIAL:
            print("[Serial] pyserial not installed")
            return False
        
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
            
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self._is_connected = True
            print(f"[Serial] Connected to {self.port}")
            return True
        except Exception as e:
            print(f"[Serial] Failed to connect to {self.port}: {e}")
            self._is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._is_connected = False
        print("[Serial] Disconnected")
    
    def start(self) -> bool:
        """Start receiving data in background thread."""
        if not self.connect():
            return False
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print("[Serial] Started receiving")
        return True
    
    def stop(self):
        """Stop receiving data."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.disconnect()
        print("[Serial] Stopped")
    
    def _receive_loop(self):
        """Main receive loop running in background thread."""
        buffer = ""
        
        while not self._stop_event.is_set():
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(0.1)
                    continue
                
                if self._serial.in_waiting > 0:
                    data = self._serial.read_until(b'\r\n')
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # Process complete frames
                    if '\r\n' in buffer:
                        frames = buffer.split('\r\n')
                        for frame in frames[:-1]:  # Last one might be incomplete
                            if frame.strip():
                                parsed = self._parse_data(frame.strip())
                                if parsed:
                                    self._handle_parsed_data(parsed)
                        buffer = frames[-1]  # Keep incomplete frame
                else:
                    time.sleep(0.01)
                    
            except Exception as e:
                print(f"[Serial] Error: {e}")
                time.sleep(0.1)
    
    def _parse_data(self, data_str: str) -> Optional[UWBRangingData]:
        """Parse received data string."""
        try:
            if not (data_str.startswith('mc') or data_str.startswith('mi')):
                return None
            
            parts = data_str.split(' ')
            length = len(parts)
            
            if parts[0] == 'mc':
                if length == 11:
                    # 4 distance values format
                    role_info = parts[9]
                    ranges = []
                    for i in range(2, 6):
                        val = int(parts[i], 16)
                        # ffffffff or 0xffffffff means invalid (-1)
                        if val == 0xFFFFFFFF or val > 0x7FFFFFFF:
                            ranges.append(-1)
                        else:
                            ranges.append(val)
                    # Pad to 8 values
                    ranges.extend([-1] * 4)
                    timestamp = parts[10]
                    
                elif length == 15:
                    # 8 distance values format
                    role_info = parts[13]
                    ranges = []
                    for i in range(2, 10):
                        val = int(parts[i], 16)
                        if val == 0xFFFFFFFF or val > 0x7FFFFFFF:
                            ranges.append(-1)
                        else:
                            ranges.append(val)
                    timestamp = parts[14]
                else:
                    return None
                
                # Parse role info (e.g., "a0:0" or "c1")
                role = role_info[0] if role_info else ''
                
                if ':' in role_info:
                    base_station_id, tag_id = role_info[1:].split(':')
                else:
                    base_station_id = role_info[1:] if len(role_info) > 1 else ''
                    tag_id = ''
                
                return UWBRangingData(
                    message_type=parts[0],
                    user_byte=parts[1],
                    ranges=ranges,
                    timestamp=timestamp,
                    role=role,
                    base_station_id=base_station_id,
                    tag_id=tag_id,
                    raw_data=data_str
                )
            
            return None
            
        except Exception as e:
            # print(f"[Serial] Parse error: {e}")
            return None
    
    def _handle_parsed_data(self, data: UWBRangingData):
        """Handle parsed data."""
        # Add to queue
        try:
            self._data_queue.put_nowait(data)
        except queue.Full:
            # Remove oldest and add new
            try:
                self._data_queue.get_nowait()
                self._data_queue.put_nowait(data)
            except:
                pass
        
        # Call callback
        if self._callback:
            try:
                self._callback(data)
            except Exception as e:
                print(f"[Serial] Callback error: {e}")
        
        # Store calibration data if in calibration mode
        if self._calibration_mode:
            self._calibration_data.append({
                'ranges': data.ranges.copy(),
                'timestamp': time.time(),
                'base_station_id': data.base_station_id
            })
    
    def get_latest_data(self) -> Optional[UWBRangingData]:
        """Get latest data from queue."""
        try:
            return self._data_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_all_data(self) -> List[UWBRangingData]:
        """Get all data from queue."""
        data_list = []
        while True:
            try:
                data_list.append(self._data_queue.get_nowait())
            except queue.Empty:
                break
        return data_list
    
    def start_calibration(self):
        """Start calibration mode to collect data."""
        self._calibration_mode = True
        self._calibration_data = []
        print("[Serial] Calibration mode started")
    
    def stop_calibration(self) -> List[Dict[str, Any]]:
        """Stop calibration mode and return collected data."""
        self._calibration_mode = False
        data = self._calibration_data.copy()
        self._calibration_data = []
        print(f"[Serial] Calibration mode stopped, collected {len(data)} samples")
        return data
    
    def send_command(self, command: str) -> bool:
        """Send command to UWB device."""
        if not self._serial or not self._serial.is_open:
            print("[Serial] Not connected")
            return False
        
        try:
            # Add newline if not present
            if not command.endswith('\r\n'):
                command += '\r\n'
            self._serial.write(command.encode('utf-8'))
            print(f"[Serial] Sent: {command.strip()}")
            return True
        except Exception as e:
            print(f"[Serial] Send error: {e}")
            return False
    
    def enter_calibration_mode(self) -> bool:
        """Send command to enter anchor calibration mode."""
        # This command may vary depending on UWB device firmware
        # Common calibration commands
        commands = [
            "cal",      # Generic calibration command
            "la",       # Enter calibration mode (some devices)
            "lec",      # Enable calibration (some devices)
        ]
        
        # Try sending calibration command
        # You may need to adjust this based on actual device protocol
        return self.send_command("la")  # Adjust as needed
    
    def exit_calibration_mode(self) -> bool:
        """Send command to exit anchor calibration mode."""
        return self.send_command("les")  # Adjust as needed


def get_available_ports() -> List[str]:
    """Get list of available serial ports."""
    if not HAS_SERIAL:
        return []
    
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    except Exception as e:
        print(f"Error listing ports: {e}")
        return []

