"""
Serial port communication module for UWB anchor self-calibration.

This module handles:
1. Serial port connection (COM16)
2. Sending commands to enter calibration mode
3. Receiving anchor range measurement data
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
from typing import Optional, Callable, List
from dataclasses import dataclass


@dataclass
class AnchorRangeMeasurement:
    """Single anchor-to-anchor range measurement."""
    anchor_i: int
    anchor_j: int
    distance_mm: int  # Distance in millimeters
    timestamp: float = 0.0


class SerialCalibrationComm:
    """
    Serial communication for anchor self-calibration.
    
    Handles:
    - Connection to COM port (default: COM16)
    - Sending calibration mode commands
    - Receiving and parsing anchor range data
    """
    
    def __init__(self, port: str = "COM16", baudrate: int = 115200):
        """
        Initialize serial communication.
        
        Args:
            port: Serial port name (e.g., "COM16")
            baudrate: Baud rate (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._is_running = False
        
        # Data queue
        self._range_queue: queue.Queue[AnchorRangeMeasurement] = queue.Queue(maxsize=100)
        
        # Callback for received data
        self._range_callback: Optional[Callable[[AnchorRangeMeasurement], None]] = None
        
        # Calibration mode state
        self._in_calibration_mode = False
    
    @property
    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self.serial_conn is not None and self.serial_conn.is_open
    
    @property
    def is_running(self) -> bool:
        """Check if communication thread is running."""
        return self._is_running
    
    @property
    def in_calibration_mode(self) -> bool:
        """Check if in calibration mode."""
        return self._in_calibration_mode
    
    def set_range_callback(self, callback: Callable[[AnchorRangeMeasurement], None]):
        """Set callback for received range measurements."""
        self._range_callback = callback
    
    def connect(self) -> bool:
        """
        Connect to serial port.
        
        Returns:
            True if connected successfully, False otherwise
        """
        if self.is_connected:
            return True
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            
            print(f"[Serial] Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"[Serial] Failed to connect: {e}")
            return False
        except Exception as e:
            print(f"[Serial] Unexpected error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        self.stop()
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except:
                pass
            self.serial_conn = None
        print(f"[Serial] Disconnected from {self.port}")
    
    def start(self) -> bool:
        """
        Start receiving data in background thread.
        
        Returns:
            True if started successfully
        """
        if not self.is_connected:
            if not self.connect():
                return False
        
        if self._is_running:
            return True
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        self._is_running = True
        print("[Serial] Started receiving thread")
        return True
    
    def stop(self):
        """Stop receiving data."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._is_running = False
        print("[Serial] Stopped receiving thread")
    
    def send_command(self, command: bytes) -> bool:
        """
        Send command to serial port.
        
        Args:
            command: Command bytes to send
            
        Returns:
            True if sent successfully
        """
        if not self.is_connected:
            print("[Serial] Not connected, cannot send command")
            return False
        
        try:
            self.serial_conn.write(command)
            self.serial_conn.flush()
            print(f"[Serial] Sent command: {command.hex()}")
            return True
        except Exception as e:
            print(f"[Serial] Failed to send command: {e}")
            return False
    
    def enter_calibration_mode(self) -> bool:
        """
        Send command to enter calibration mode.
        
        Note: The actual command format needs to be provided by the hardware vendor.
        Common formats:
        - ASCII: "CALIBRATE\r\n" or "CAL\r\n"
        - Hex: b'\x01\x02\x03' (example)
        
        Returns:
            True if command sent successfully
        """
        # TODO: Replace with actual command from hardware documentation
        # Common formats to try:
        # command = b"CALIBRATE\r\n"  # ASCII format
        # command = b"CAL\r\n"
        # command = bytes([0x01, 0x02, 0x03])  # Binary format
        
        # Default: Try ASCII format
        command = b"CALIBRATE\r\n"
        
        if self.send_command(command):
            self._in_calibration_mode = True
            print("[Serial] Entered calibration mode")
            return True
        return False
    
    def exit_calibration_mode(self) -> bool:
        """
        Send command to exit calibration mode.
        
        Returns:
            True if command sent successfully
        """
        # TODO: Replace with actual command
        command = b"EXIT_CAL\r\n"
        
        if self.send_command(command):
            self._in_calibration_mode = False
            print("[Serial] Exited calibration mode")
            return True
        return False
    
    def _receive_loop(self):
        """Main receive loop running in background thread."""
        buffer = b""
        
        while not self._stop_event.is_set():
            try:
                if not self.is_connected:
                    time.sleep(0.1)
                    continue
                
                # Read available data
                data = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                if not data:
                    continue
                
                buffer += data
                
                # Process complete lines/messages
                while b"\n" in buffer or b"\r" in buffer:
                    # Try to find line ending
                    if b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                    elif b"\r" in buffer:
                        line, buffer = buffer.split(b"\r", 1)
                    else:
                        break
                    
                    # Process line
                    if line:
                        self._process_received_data(line)
                
            except serial.SerialException as e:
                if not self._stop_event.is_set():
                    print(f"[Serial] Receive error: {e}")
                break
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[Serial] Unexpected error: {e}")
                time.sleep(0.1)
    
    def _process_received_data(self, data: bytes):
        """
        Process received data and extract anchor range measurements.
        
        This function needs to parse the actual data format from the hardware.
        Common formats:
        1. JSON: {"AnchorI": 0, "AnchorJ": 1, "Distance": 25000}
        2. CSV: "0,1,25000"
        3. Binary: [0x00, 0x01, 0x61, 0xA8] (example)
        4. Custom text: "A0-A1: 25000mm"
        
        Args:
            data: Raw bytes received from serial port
        """
        try:
            # Try JSON format first
            import json
            text = data.decode('utf-8', errors='ignore').strip()
            
            # Try parsing as JSON
            try:
                payload = json.loads(text)
                if "AnchorI" in payload and "AnchorJ" in payload and "Distance" in payload:
                    range_data = AnchorRangeMeasurement(
                        anchor_i=int(payload["AnchorI"]),
                        anchor_j=int(payload["AnchorJ"]),
                        distance_mm=int(payload["Distance"]),
                        timestamp=time.time()
                    )
                    self._handle_range_data(range_data)
                    return
            except json.JSONDecodeError:
                pass
            
            # Try CSV format: "I,J,Distance"
            parts = text.split(',')
            if len(parts) >= 3:
                try:
                    range_data = AnchorRangeMeasurement(
                        anchor_i=int(parts[0]),
                        anchor_j=int(parts[1]),
                        distance_mm=int(parts[2]),
                        timestamp=time.time()
                    )
                    self._handle_range_data(range_data)
                    return
                except ValueError:
                    pass
            
            # Try custom text format: "A0-A1: 25000mm" or "0-1: 25000"
            import re
            pattern = r'[A]?(\d+)[\s-]+[A]?(\d+)[\s:]+(\d+)'
            match = re.search(pattern, text)
            if match:
                try:
                    range_data = AnchorRangeMeasurement(
                        anchor_i=int(match.group(1)),
                        anchor_j=int(match.group(2)),
                        distance_mm=int(match.group(3)),
                        timestamp=time.time()
                    )
                    self._handle_range_data(range_data)
                    return
                except ValueError:
                    pass
            
            # If no format matches, print for debugging
            if text and len(text) > 0:
                print(f"[Serial] Unparsed data: {text}")
                
        except Exception as e:
            print(f"[Serial] Error processing data: {e}")
    
    def _handle_range_data(self, range_data: AnchorRangeMeasurement):
        """Handle received range measurement data."""
        # Add to queue
        try:
            self._range_queue.put_nowait(range_data)
        except queue.Full:
            # Drop oldest
            try:
                self._range_queue.get_nowait()
                self._range_queue.put_nowait(range_data)
            except:
                pass
        
        # Call callback
        if self._range_callback:
            self._range_callback(range_data)
    
    def get_latest_range(self) -> Optional[AnchorRangeMeasurement]:
        """Get latest range measurement (non-blocking)."""
        latest = None
        while not self._range_queue.empty():
            try:
                latest = self._range_queue.get_nowait()
            except queue.Empty:
                break
        return latest
    
    def get_all_ranges(self) -> List[AnchorRangeMeasurement]:
        """Get all queued range measurements."""
        ranges = []
        while not self._range_queue.empty():
            try:
                ranges.append(self._range_queue.get_nowait())
            except queue.Empty:
                break
        return ranges
    
    @staticmethod
    def list_available_ports() -> List[str]:
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]


def test_serial_comm():
    """Test serial communication."""
    comm = SerialCalibrationComm(port="COM16")
    
    print("Available ports:", SerialCalibrationComm.list_available_ports())
    
    if comm.connect():
        print("Connected successfully!")
        
        # Try to enter calibration mode
        comm.enter_calibration_mode()
        
        # Start receiving
        comm.start()
        
        # Wait for data
        time.sleep(5)
        
        # Get received data
        ranges = comm.get_all_ranges()
        print(f"Received {len(ranges)} range measurements")
        for r in ranges:
            print(f"  A{r.anchor_i}-A{r.anchor_j}: {r.distance_mm}mm")
        
        # Exit calibration mode
        comm.exit_calibration_mode()
        
        comm.disconnect()
    else:
        print("Failed to connect")


if __name__ == "__main__":
    test_serial_comm()

