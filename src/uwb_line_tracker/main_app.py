"""
Main application for UWB and GNSS visualization.
Features:
- Dual map display (UWB and GNSS on OpenStreetMap)
- Start line crossing detection
- Speed, distance, and time calculations
- Timing statistics
- GNSS anchor data reception via TCP/ZMQ
- Serial port UWB data with trilateration positioning
- Anchor self-calibration using MDS algorithm
"""

from __future__ import annotations

import math
import time
import threading
import subprocess
import sys
import os
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from typing import Optional, Tuple, List
from dataclasses import dataclass

try:
    import tkintermapview
    HAS_MAP = True
except ImportError:
    HAS_MAP = False
    print("Warning: tkintermapview not installed. Maps will not be displayed.")
    print("Install with: pip install tkintermapview")

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Warning: PIL/Pillow not installed. Image background will not be displayed.")
    print("Install with: pip install Pillow")

from .coordinate_transform import (
    UWBToGPSTransformer, 
    UWBPosition, 
    GNSSPosition,
    calculate_distance_to_line,
    check_line_crossing
)
from .udp_receiver import UDPReceiver, UWBTagData
from .gnss_receiver import GNSSReceiver, GNSSData
from .serial_receiver import SerialReceiver, UWBRangingData, get_available_ports, HAS_SERIAL
from .trilateration_wrapper import TrilaterationEngine, Position3D
from .anchor_calibration import AnchorCalibration, AnchorPosition


@dataclass
class TimingStats:
    """Timing statistics for start line crossing."""
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    crossing_time: Optional[float] = None
    
    def reset(self):
        self.start_time = None
        self.end_time = None
        self.crossing_time = None


class AnchorCalibrationDialog(tk.Toplevel):
    """
    Dialog for UWB anchor self-calibration.
    Features:
    - Serial port selection
    - "Calibrate" button: Start/stop calibration data collection
    - "Confirm" button: Apply calibrated anchor positions
    - Real-time display of calibration data
    """
    
    def __init__(self, parent, current_anchors: list, serial_receiver: SerialReceiver, on_confirm=None):
        super().__init__(parent)
        self.title("UWB Anchor Self-Calibration")
        self.geometry("600x550")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        
        self.result = None
        self.current_anchors = current_anchors
        self.serial_receiver = serial_receiver
        self.on_confirm = on_confirm
        
        self._calibration = AnchorCalibration(num_anchors=len(current_anchors))
        self._is_calibrating = False
        self._calibration_start_time = None
        self._calibration_samples = 0
        self._calibrated_positions: Optional[List[AnchorPosition]] = None
        
        # Distance measurements storage
        self._distance_buffer = {}  # {(from_id, to_id): [distances]}
        
        self._create_widgets()
        self._update_port_list()
        
        # Start update loop
        self._update_id = None
        self._start_update_loop()
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="UWB Anchor Self-Calibration", 
                                font=("Arial", 14, "bold"))
        title_label.pack(pady=(0, 10))
        
        # Instructions
        info_text = """Instructions:
1. Connect UWB device via serial port
2. Click "Start Calibration" to begin collecting inter-anchor distances
3. Wait for sufficient samples (at least 10-20 seconds)
4. Click "Stop Calibration" to process data
5. Review the calculated anchor positions
6. Click "Confirm" to apply the calibrated positions"""
        
        info_label = ttk.Label(main_frame, text=info_text, font=("Arial", 9), 
                               justify=tk.LEFT, foreground="gray")
        info_label.pack(pady=(0, 10), anchor=tk.W)
        
        # Serial port selection
        port_frame = ttk.LabelFrame(main_frame, text="Serial Port", padding=5)
        port_frame.pack(fill=tk.X, pady=5)
        
        port_row = ttk.Frame(port_frame)
        port_row.pack(fill=tk.X)
        
        ttk.Label(port_row, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_combo = ttk.Combobox(port_row, width=15, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(port_row, text="Refresh", command=self._update_port_list).pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(port_row, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.connection_status = ttk.Label(port_row, text="Disconnected", foreground="red")
        self.connection_status.pack(side=tk.LEFT, padx=10)
        
        # Calibration controls
        cal_frame = ttk.LabelFrame(main_frame, text="Calibration", padding=5)
        cal_frame.pack(fill=tk.X, pady=5)
        
        cal_row = ttk.Frame(cal_frame)
        cal_row.pack(fill=tk.X)
        
        self.calibrate_btn = ttk.Button(cal_row, text="Start Calibration", 
                                        command=self._toggle_calibration, width=20)
        self.calibrate_btn.pack(side=tk.LEFT, padx=5)
        
        self.cal_status = ttk.Label(cal_row, text="Not started", font=("Arial", 10))
        self.cal_status.pack(side=tk.LEFT, padx=10)
        
        # Progress
        self.progress_var = tk.DoubleVar(value=0)
        self.progress_bar = ttk.Progressbar(cal_frame, variable=self.progress_var, 
                                            maximum=100, length=400)
        self.progress_bar.pack(fill=tk.X, pady=5, padx=5)
        
        # Distance matrix display
        dist_frame = ttk.LabelFrame(main_frame, text="Inter-Anchor Distances (meters)", padding=5)
        dist_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.dist_text = tk.Text(dist_frame, height=6, width=50, font=("Consolas", 10))
        self.dist_text.pack(fill=tk.BOTH, expand=True)
        self.dist_text.insert(tk.END, "No calibration data yet...")
        self.dist_text.config(state=tk.DISABLED)
        
        # Result display
        result_frame = ttk.LabelFrame(main_frame, text="Calibrated Anchor Positions (meters)", padding=5)
        result_frame.pack(fill=tk.X, pady=5)
        
        self.result_text = tk.Text(result_frame, height=4, width=50, font=("Consolas", 10))
        self.result_text.pack(fill=tk.X)
        self._display_current_anchors()
        self.result_text.config(state=tk.DISABLED)
        
        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.confirm_btn = ttk.Button(btn_frame, text="Confirm", command=self._on_confirm, 
                                      state=tk.DISABLED, width=15)
        self.confirm_btn.pack(side=tk.RIGHT, padx=5)
        
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel, width=15).pack(side=tk.RIGHT, padx=5)
        
        # Manual input frame
        manual_frame = ttk.LabelFrame(main_frame, text="Manual Distance Input (for testing)", padding=5)
        manual_frame.pack(fill=tk.X, pady=5)
        
        manual_row = ttk.Frame(manual_frame)
        manual_row.pack(fill=tk.X)
        
        ttk.Label(manual_row, text="A0-A1 (mm):").grid(row=0, column=0, padx=2)
        self.d01_entry = ttk.Entry(manual_row, width=10)
        self.d01_entry.grid(row=0, column=1, padx=2)
        self.d01_entry.insert(0, "25000")
        
        ttk.Label(manual_row, text="A0-A2 (mm):").grid(row=0, column=2, padx=2)
        self.d02_entry = ttk.Entry(manual_row, width=10)
        self.d02_entry.grid(row=0, column=3, padx=2)
        self.d02_entry.insert(0, "15811")
        
        ttk.Label(manual_row, text="A1-A2 (mm):").grid(row=0, column=4, padx=2)
        self.d12_entry = ttk.Entry(manual_row, width=10)
        self.d12_entry.grid(row=0, column=5, padx=2)
        self.d12_entry.insert(0, "15811")
        
        ttk.Button(manual_row, text="Calibrate from Input", 
                   command=self._manual_calibrate).grid(row=0, column=6, padx=10)
    
    def _display_current_anchors(self):
        """Display current anchor positions."""
        self.result_text.config(state=tk.NORMAL)
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, "Current positions:\n")
        for i, anchor in enumerate(self.current_anchors):
            self.result_text.insert(tk.END, f"  Anchor {i}: X={anchor.get('x', 0):.3f}, Y={anchor.get('y', 0):.3f}, Z={anchor.get('z', 0):.3f}\n")
        self.result_text.config(state=tk.DISABLED)
    
    def _update_port_list(self):
        """Update available serial ports."""
        ports = get_available_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
        else:
            self.port_combo.set("No ports found")
    
    def _toggle_connection(self):
        """Toggle serial port connection."""
        if self.serial_receiver.is_connected:
            self.serial_receiver.stop()
            self.connect_btn.config(text="Connect")
            self.connection_status.config(text="Disconnected", foreground="red")
        else:
            port = self.port_combo.get()
            if port and port != "No ports found":
                self.serial_receiver.set_port(port)
                if self.serial_receiver.start():
                    self.connect_btn.config(text="Disconnect")
                    self.connection_status.config(text=f"Connected to {port}", foreground="green")
                else:
                    messagebox.showerror("Error", f"Failed to connect to {port}")
    
    def _toggle_calibration(self):
        """Toggle calibration mode."""
        if self._is_calibrating:
            self._stop_calibration()
        else:
            self._start_calibration()
    
    def _start_calibration(self):
        """Start calibration data collection."""
        if not self.serial_receiver.is_connected:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        self._is_calibrating = True
        self._calibration_start_time = time.time()
        self._calibration_samples = 0
        self._distance_buffer = {}
        self._calibration.reset()
        
        self.calibrate_btn.config(text="Stop Calibration")
        self.cal_status.config(text="Collecting data...", foreground="blue")
        self.confirm_btn.config(state=tk.DISABLED)
        
        # Enter calibration mode on device
        self.serial_receiver.enter_calibration_mode()
        self.serial_receiver.start_calibration()
    
    def _stop_calibration(self):
        """Stop calibration and process data."""
        self._is_calibrating = False
        self.calibrate_btn.config(text="Start Calibration")
        self.cal_status.config(text="Processing...", foreground="orange")
        
        # Get collected data
        cal_data = self.serial_receiver.stop_calibration()
        self.serial_receiver.exit_calibration_mode()
        
        # Process data into distance matrix
        self._process_calibration_data(cal_data)
        
        # Run calibration algorithm
        self._run_calibration()
    
    def _process_calibration_data(self, cal_data: list):
        """Process collected calibration data into distance measurements."""
        # For each sample, extract inter-anchor distances
        for sample in cal_data:
            ranges = sample.get('ranges', [])
            anchor_id = int(sample.get('base_station_id', '0'))
            
            for i, dist in enumerate(ranges):
                if dist > 0 and i != anchor_id:
                    key = (anchor_id, i)
                    if key not in self._distance_buffer:
                        self._distance_buffer[key] = []
                    self._distance_buffer[key].append(dist)
        
        # Update calibration object
        for (from_id, to_id), distances in self._distance_buffer.items():
            if distances:
                median_dist = sum(distances) / len(distances)
                self._calibration.add_distance_measurement(from_id, to_id, median_dist)
    
    def _run_calibration(self):
        """Run MDS calibration algorithm."""
        result = self._calibration.calibrate()
        
        if result:
            self._calibrated_positions = result
            self._display_calibration_result(result)
            self.cal_status.config(text="Calibration successful!", foreground="green")
            self.confirm_btn.config(state=tk.NORMAL)
        else:
            self.cal_status.config(text="Calibration failed - insufficient data", foreground="red")
            messagebox.showwarning("Calibration Failed", 
                                   "Insufficient distance data for calibration.\n"
                                   "Please ensure all anchors can measure distances to each other.")
    
    def _display_calibration_result(self, positions: List[AnchorPosition]):
        """Display calibration result."""
        self.result_text.config(state=tk.NORMAL)
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, "Calibrated positions:\n")
        for pos in positions:
            self.result_text.insert(tk.END, f"  Anchor {pos.id}: X={pos.x:.3f}, Y={pos.y:.3f}, Z={pos.z:.3f}\n")
        self.result_text.config(state=tk.DISABLED)
    
    def _display_distance_matrix(self):
        """Display current distance measurements."""
        self.dist_text.config(state=tk.NORMAL)
        self.dist_text.delete(1.0, tk.END)
        
        if not self._distance_buffer:
            self.dist_text.insert(tk.END, "No distance data collected yet...")
        else:
            self.dist_text.insert(tk.END, "Collected distances (mm):\n")
            for (from_id, to_id), distances in sorted(self._distance_buffer.items()):
                if from_id < to_id:  # Show each pair once
                    avg = sum(distances) / len(distances) if distances else 0
                    self.dist_text.insert(tk.END, 
                        f"  A{from_id} <-> A{to_id}: {avg:.0f} mm ({len(distances)} samples)\n")
        
        self.dist_text.config(state=tk.DISABLED)
    
    def _manual_calibrate(self):
        """Perform calibration from manually entered distances."""
        try:
            d01 = float(self.d01_entry.get())
            d02 = float(self.d02_entry.get())
            d12 = float(self.d12_entry.get())
            
            # Build distance matrix
            matrix = [
                [0, d01, d02],
                [d01, 0, d12],
                [d02, d12, 0]
            ]
            
            # Update distance buffer for display
            self._distance_buffer = {
                (0, 1): [d01],
                (0, 2): [d02],
                (1, 2): [d12],
            }
            self._display_distance_matrix()
            
            # Create new calibration with manual data
            import numpy as np
            self._calibration = AnchorCalibration(num_anchors=3)
            self._calibration.set_distance_matrix(np.array(matrix))
            
            # Run calibration
            result = self._calibration.calibrate()
            
            if result:
                self._calibrated_positions = result
                self._display_calibration_result(result)
                self.cal_status.config(text="Manual calibration successful!", foreground="green")
                self.confirm_btn.config(state=tk.NORMAL)
            else:
                self.cal_status.config(text="Manual calibration failed", foreground="red")
                
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")
    
    def _start_update_loop(self):
        """Start UI update loop."""
        self._update_ui()
    
    def _update_ui(self):
        """Update UI periodically."""
        if self._is_calibrating:
            elapsed = time.time() - self._calibration_start_time
            # Update progress (assume 30 seconds for full progress)
            progress = min(100, (elapsed / 30) * 100)
            self.progress_var.set(progress)
            
            # Count samples
            total_samples = sum(len(v) for v in self._distance_buffer.values())
            self.cal_status.config(text=f"Collecting... {elapsed:.1f}s, {total_samples} samples")
            
            # Update distance display
            self._display_distance_matrix()
        
        # Schedule next update
        self._update_id = self.after(500, self._update_ui)
    
    def _on_confirm(self):
        """Confirm and apply calibrated positions."""
        if self._calibrated_positions:
            self.result = []
            for pos in self._calibrated_positions:
                self.result.append({
                    "x": pos.x,
                    "y": pos.y,
                    "z": pos.z,
                })
            
            if self.on_confirm:
                self.on_confirm(self.result)
        
        self._cleanup()
        self.destroy()
    
    def _on_cancel(self):
        """Cancel and close dialog."""
        self.result = None
        self._cleanup()
        self.destroy()
    
    def _cleanup(self):
        """Clean up resources."""
        if self._update_id:
            self.after_cancel(self._update_id)
        if self._is_calibrating:
            self.serial_receiver.stop_calibration()


class SettingsDialog(tk.Toplevel):
    """Dialog for manually configuring UWB anchor positions."""
    
    def __init__(self, parent, current_anchors: list[dict]):
        super().__init__(parent)
        self.title("Manual Anchor Settings")
        self.geometry("400x350")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        
        self.result = None
        self.current_anchors = current_anchors
        
        self._create_widgets()
        self._load_values()
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        ttk.Label(main_frame, text="UWB Anchor Coordinates (meters)", 
                  font=("Arial", 11, "bold")).pack(pady=(0, 10))
        
        # Anchor entries
        self.entries = {}
        for i in range(3):
            frame = ttk.LabelFrame(main_frame, text=f"Anchor {i}", padding=5)
            frame.pack(fill=tk.X, pady=5)
            
            row = ttk.Frame(frame)
            row.pack(fill=tk.X)
            
            self.entries[f"anchor{i}"] = {}
            for j, axis in enumerate(["X", "Y", "Z"]):
                ttk.Label(row, text=f"{axis}:").grid(row=0, column=j*2, padx=(5, 2))
                entry = ttk.Entry(row, width=10)
                entry.grid(row=0, column=j*2+1, padx=(0, 10))
                self.entries[f"anchor{i}"][axis.lower()] = entry
        
        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(20, 0))
        
        ttk.Button(btn_frame, text="OK", command=self._on_ok).pack(side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side=tk.RIGHT)
    
    def _load_values(self):
        for i, anchor in enumerate(self.current_anchors):
            key = f"anchor{i}"
            if key in self.entries:
                self.entries[key]["x"].insert(0, str(anchor.get("x", 0.0)))
                self.entries[key]["y"].insert(0, str(anchor.get("y", 0.0)))
                self.entries[key]["z"].insert(0, str(anchor.get("z", 0.0)))
    
    def _on_ok(self):
        try:
            self.result = []
            for i in range(3):
                key = f"anchor{i}"
                self.result.append({
                    "x": float(self.entries[key]["x"].get() or 0),
                    "y": float(self.entries[key]["y"].get() or 0),
                    "z": float(self.entries[key]["z"].get() or 0),
                })
            self.destroy()
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid value: {e}")
    
    def _on_cancel(self):
        self.result = None
        self.destroy()


class TransformSettingsDialog(tk.Toplevel):
    """Dialog for configuring UWB anchor transformation (rotation, scale, offset)."""
    
    def __init__(self, parent, scale: float, rotation_deg: float, offset_x: float, offset_y: float, apply_callback=None):
        super().__init__(parent)
        self.title("UWB Transform Settings")
        self.geometry("450x420")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        
        self.result = None
        self.scale = scale
        self.rotation_deg = rotation_deg
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.apply_callback = apply_callback
        
        self._create_widgets()
        self._load_values()
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title and instructions
        title_label = ttk.Label(main_frame, text="UWB Coordinate Transformation", 
                  font=("Arial", 12, "bold"))
        title_label.pack(pady=(0, 5))
        
        info_label = ttk.Label(main_frame, 
                  text="Adjust these values to align UWB anchors with the background image.\nClick 'Apply' to update the map immediately.",
                  font=("Arial", 9), foreground="gray", justify=tk.LEFT)
        info_label.pack(pady=(0, 10))
        
        # Scale
        scale_frame = ttk.LabelFrame(main_frame, text="Scale Factor", padding=5)
        scale_frame.pack(fill=tk.X, pady=3)
        scale_row = ttk.Frame(scale_frame)
        scale_row.pack(fill=tk.X)
        ttk.Label(scale_row, text="Scale:").pack(side=tk.LEFT, padx=5)
        self.scale_entry = ttk.Entry(scale_row, width=20)
        self.scale_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(scale_row, text="(e.g., 1.0 = no scaling)", font=("Arial", 8), foreground="gray").pack(side=tk.LEFT, padx=5)
        
        # Rotation
        rot_frame = ttk.LabelFrame(main_frame, text="Rotation", padding=5)
        rot_frame.pack(fill=tk.X, pady=3)
        rot_row = ttk.Frame(rot_frame)
        rot_row.pack(fill=tk.X)
        ttk.Label(rot_row, text="Angle:").pack(side=tk.LEFT, padx=5)
        self.rotation_entry = ttk.Entry(rot_row, width=20)
        self.rotation_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(rot_row, text="degrees (e.g., 0 = no rotation)", font=("Arial", 8), foreground="gray").pack(side=tk.LEFT, padx=5)
        
        # Offset
        offset_frame = ttk.LabelFrame(main_frame, text="Offset (meters)", padding=5)
        offset_frame.pack(fill=tk.X, pady=3)
        
        offset_row1 = ttk.Frame(offset_frame)
        offset_row1.pack(fill=tk.X)
        ttk.Label(offset_row1, text="X Offset:").grid(row=0, column=0, padx=(5, 2), sticky=tk.W)
        self.offset_x_entry = ttk.Entry(offset_row1, width=15)
        self.offset_x_entry.grid(row=0, column=1, padx=(0, 15))
        ttk.Label(offset_row1, text="Y Offset:", font=("Arial", 9)).grid(row=0, column=2, padx=(5, 2), sticky=tk.W)
        self.offset_y_entry = ttk.Entry(offset_row1, width=15)
        self.offset_y_entry.grid(row=0, column=3, padx=(0, 5))
        
        # Status label for Apply feedback
        self.status_label = ttk.Label(main_frame, text="", font=("Arial", 9))
        self.status_label.pack(pady=(5, 0))
        
        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(15, 0))
        
        ttk.Button(btn_frame, text="Apply", command=self._on_apply, width=12).pack(side=tk.RIGHT, padx=5)
        ttk.Label(btn_frame, text="(Settings apply immediately, dialog stays open)", font=("Arial", 8), foreground="green").pack(side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="Close", command=self._on_close, width=12).pack(side=tk.LEFT, padx=5)
    
    def _load_values(self):
        self.scale_entry.insert(0, str(self.scale))
        self.rotation_entry.insert(0, str(self.rotation_deg))
        self.offset_x_entry.insert(0, str(self.offset_x))
        self.offset_y_entry.insert(0, str(self.offset_y))
    
    def _on_apply(self):
        """Apply settings without closing dialog."""
        try:
            new_settings = {
                "scale": float(self.scale_entry.get() or 1.0),
                "rotation_deg": float(self.rotation_entry.get() or 0.0),
                "offset_x": float(self.offset_x_entry.get() or 0.0),
                "offset_y": float(self.offset_y_entry.get() or 0.0),
            }
            
            self.scale = new_settings["scale"]
            self.rotation_deg = new_settings["rotation_deg"]
            self.offset_x = new_settings["offset_x"]
            self.offset_y = new_settings["offset_y"]
            
            if self.apply_callback:
                self.apply_callback(new_settings)
            
            self.status_label.config(text="Settings applied!", foreground="green")
            self.after(1000, lambda: self.status_label.config(text="", foreground="black"))
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid value: {e}")
    
    def _on_close(self):
        """Close the dialog."""
        try:
            self.result = {
                "scale": float(self.scale_entry.get() or 1.0),
                "rotation_deg": float(self.rotation_entry.get() or 0.0),
                "offset_x": float(self.offset_x_entry.get() or 0.0),
                "offset_y": float(self.offset_y_entry.get() or 0.0),
            }
        except ValueError:
            self.result = {
                "scale": self.scale,
                "rotation_deg": self.rotation_deg,
                "offset_x": self.offset_x,
                "offset_y": self.offset_y,
            }
        self.destroy()


class TCPServerManager:
    """Manages the TCP server for GNSS data in a subprocess."""
    
    def __init__(self):
        self._process: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
    
    def start(self) -> bool:
        """Start the TCP server."""
        if self._process is not None:
            return True
        
        try:
            self._process = subprocess.Popen(
                [sys.executable, "-m", "uwb_line_tracker.tcp_server"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._read_output, daemon=True)
            self._thread.start()
            
            print("[TCP Server] Started")
            return True
        except Exception as e:
            print(f"[TCP Server] Failed to start: {e}")
            return False
    
    def _read_output(self):
        """Read and print TCP server output."""
        while not self._stop_event.is_set() and self._process:
            try:
                line = self._process.stdout.readline()
                if line:
                    print(f"[TCP Server] {line.strip()}")
                elif self._process.poll() is not None:
                    break
            except:
                break
    
    def stop(self):
        """Stop the TCP server."""
        self._stop_event.set()
        if self._process:
            try:
                self._process.terminate()
                self._process.wait(timeout=2)
            except:
                try:
                    self._process.kill()
                except:
                    pass
            self._process = None
        print("[TCP Server] Stopped")
    
    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None


class MainApplication:
    """Main application window."""
    
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UWB & GNSS Start Line Tracker")
        self.root.geometry("1400x900")
        self.root.minsize(1200, 800)
        
        # State
        self.transformer = UWBToGPSTransformer()
        self.udp_receiver = UDPReceiver(host="0.0.0.0", port=8080)
        self.gnss_receiver = GNSSReceiver()
        self.tcp_server = TCPServerManager()
        
        # Serial receiver and trilateration for direct positioning
        self.serial_receiver = SerialReceiver()
        self.trilateration = TrilaterationEngine()
        
        # Data source mode: 'udp' or 'serial'
        self.data_source = 'udp'  # Default to UDP for backward compatibility
        
        # UWB anchor positions (to be configured or calibrated)
        self.uwb_anchors = [
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 25.0, "y": 0.0, "z": 0.0},
            {"x": 12.5, "y": 10.0, "z": 0.0},
        ]
        
        # UWB anchor transformation parameters (for image overlay)
        self.uwb_scale = 1.0
        self.uwb_rotation_deg = 0.0
        self.uwb_offset_x = 0.0
        self.uwb_offset_y = 0.0
        
        # UWB background image
        self.uwb_bg_image_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            "UWB底图.png"
        )
        self.uwb_bg_image = None
        self.uwb_bg_photo = None
        
        # UWB map canvas (for custom image background) - Square shape
        self.uwb_canvas = None
        self.uwb_canvas_width = 450
        self.uwb_canvas_height = 450
        
        # GNSS tag update throttling (1 second)
        self.last_gnss_tag_update_time = 0.0
        self.gnss_tag_update_interval = 1.0
        
        # GNSS anchor positions (from GNSS data)
        self.gnss_anchors: list[Optional[GNSSPosition]] = [None, None, None]
        self.gnss_calibrated = False
        self.gnss_anchor_markers_updated = False
        
        # GNSS Tag position (direct from GNSS, for right map)
        self.gnss_direct_tag_pos: Optional[GNSSPosition] = None
        
        # Tag tracking (UWB)
        self.uwb_tag_pos: Optional[UWBPosition] = None
        self.gnss_tag_pos: Optional[GNSSPosition] = None
        self.prev_uwb_tag_pos: Optional[UWBPosition] = None
        self.prev_update_time: Optional[float] = None
        self.last_uwb_data_time: Optional[float] = None
        
        # Speed calculation with smoothing
        self.speed_history = []
        self.current_speed = 0.0
        
        # Crossing detection
        self.has_crossed = False
        self.timing = TimingStats()
        
        # Map markers
        self.uwb_markers = {}
        self.gnss_markers = {}
        self.uwb_start_line = None
        self.gnss_start_line = None
        self.uwb_tag_marker = None
        self.gnss_tag_marker = None
        
        # Create UI
        self._create_ui()
        
        # Start services
        self._start_services()
        
        # Start update loop
        self.root.after(100, self._update_loop)
        
        # Handle close
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
    
    def _start_services(self):
        """Start all background services."""
        # Start TCP server for GNSS
        self.tcp_server.start()
        
        time.sleep(0.5)
        
        # Start GNSS receiver (ZMQ)
        self.gnss_receiver.set_callback(self._on_gnss_data)
        self.gnss_receiver.start()
        
        # Start UDP receiver for UWB (default mode)
        self.udp_receiver.start()
        
        # Update trilateration with anchor positions
        self._update_trilateration_anchors()
    
    def _update_trilateration_anchors(self):
        """Update trilateration engine with current anchor positions."""
        if self.trilateration.is_loaded:
            anchors = [(a["x"], a["y"], a["z"]) for a in self.uwb_anchors]
            self.trilateration.set_anchors(anchors)
    
    def _on_gnss_data(self, data: GNSSData):
        """Callback when GNSS data is received."""
        if data.client_id in [0, 1, 2]:
            self.gnss_anchors[data.client_id] = GNSSPosition(
                lat=data.lat, lon=data.lng, alt=data.alt
            )
            
            anchor_count = sum(1 for a in self.gnss_anchors if a is not None)
            self.root.after(0, lambda: self._update_gnss_status(anchor_count))
            
            if not self.gnss_calibrated and all(a is not None for a in self.gnss_anchors):
                self.gnss_calibrated = True
                self.root.after(100, self._auto_calibrate)
        else:
            self.gnss_direct_tag_pos = GNSSPosition(
                lat=data.lat, lon=data.lng, alt=data.alt
            )
    
    def _update_gnss_status(self, anchor_count: int):
        """Update GNSS status display."""
        if anchor_count >= 3:
            self.gnss_status_var.set(f"GNSS: {anchor_count}/3 Anchors ✓")
            self.gnss_label.config(foreground="green")
        else:
            self.gnss_status_var.set(f"GNSS: {anchor_count}/3 Anchors")
            self.gnss_label.config(foreground="orange")
    
    def _auto_calibrate(self):
        """Auto-calibrate when all GNSS anchors are received."""
        if not self.gnss_calibrated:
            return
        
        uwb_positions = [
            UWBPosition(a["x"], a["y"], a["z"]) for a in self.uwb_anchors
        ]
        gnss_positions = [a for a in self.gnss_anchors if a is not None]
        
        if len(gnss_positions) >= 3:
            if self.transformer.calibrate(uwb_positions, gnss_positions):
                print("[Calibration] Success!")
                self._update_anchor_markers()
                self.gnss_status_var.set("GNSS: Calibrated ✓")
            else:
                print("[Calibration] Failed")
                self.gnss_status_var.set("GNSS: Calibration failed")
    
    def _create_ui(self):
        """Create the main UI layout."""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Top section: Maps (smaller)
        maps_frame = ttk.Frame(main_frame)
        maps_frame.pack(fill=tk.BOTH, expand=True)
        
        # UWB Map (Left) - Canvas with image background (Square frame)
        uwb_frame = ttk.LabelFrame(maps_frame, text="UWB Positioning", padding=5)
        uwb_frame.pack(side=tk.LEFT, padx=(0, 2))
        
        # Create canvas with fixed square size
        self.uwb_canvas = tk.Canvas(
            uwb_frame, 
            width=self.uwb_canvas_width, 
            height=self.uwb_canvas_height, 
            bg="white", 
            highlightthickness=0
        )
        self.uwb_canvas.pack()
        self.uwb_map = None
        
        # Load background image
        self._load_uwb_background_image()
        
        # GNSS Map (Right)
        gnss_frame = ttk.LabelFrame(maps_frame, text="GNSS Positioning", padding=5)
        gnss_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(2, 0))
        
        if HAS_MAP:
            self.gnss_map = tkintermapview.TkinterMapView(gnss_frame, width=300, height=300)
            self.gnss_map.pack(fill=tk.BOTH, expand=True)
            self.gnss_map.set_position(22.3193, 114.1694)
            self.gnss_map.set_zoom(18)
        else:
            self.gnss_map = None
            ttk.Label(gnss_frame, text="Map not available\nInstall tkintermapview").pack(expand=True)
        
        # Bottom section: Status and Controls
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Row 1: Crossing status
        status_frame = ttk.Frame(bottom_frame)
        status_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(status_frame, text="Start Line Status:", font=("Arial", 11)).pack(side=tk.LEFT, padx=5)
        self.crossing_label = ttk.Label(status_frame, text="NOT CROSSED", 
                                         font=("Arial", 14, "bold"), foreground="red")
        self.crossing_label.pack(side=tk.LEFT, padx=10)
        
        # Data source selector
        ttk.Label(status_frame, text="Data Source:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(20, 5))
        self.data_source_var = tk.StringVar(value="udp")
        data_source_combo = ttk.Combobox(status_frame, textvariable=self.data_source_var, 
                                         values=["udp", "serial"], width=10, state="readonly")
        data_source_combo.pack(side=tk.LEFT, padx=5)
        data_source_combo.bind("<<ComboboxSelected>>", self._on_data_source_change)
        
        # Row 2: Metrics
        metrics_frame = ttk.Frame(bottom_frame)
        metrics_frame.pack(fill=tk.X, pady=5)
        
        # Speed (larger font)
        speed_frame = ttk.LabelFrame(metrics_frame, text="Speed", padding=8)
        speed_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.speed_var = tk.StringVar(value="0.00 m/s")
        ttk.Label(speed_frame, textvariable=self.speed_var, font=("Consolas", 36, "bold")).pack()
        
        # Distance to line
        dist_frame = ttk.LabelFrame(metrics_frame, text="Distance to Start Line", padding=8)
        dist_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.distance_var = tk.StringVar(value="0.00 m")
        ttk.Label(dist_frame, textvariable=self.distance_var, font=("Consolas", 36, "bold")).pack()
        
        # Time to line
        time_frame = ttk.LabelFrame(metrics_frame, text="Time to Start Line", padding=8)
        time_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.time_to_line_var = tk.StringVar(value="-- s")
        ttk.Label(time_frame, textvariable=self.time_to_line_var, font=("Consolas", 36, "bold")).pack()
        
        # Row 3: Timing statistics
        timing_frame = ttk.LabelFrame(bottom_frame, text="Timing Statistics", padding=5)
        timing_frame.pack(fill=tk.X, pady=5)
        
        timing_grid = ttk.Frame(timing_frame)
        timing_grid.pack(fill=tk.X)
        
        ttk.Label(timing_grid, text="Start Time:", font=("Arial", 14)).grid(row=0, column=0, padx=5, sticky=tk.W)
        self.start_time_var = tk.StringVar(value="--:--:--.---")
        ttk.Label(timing_grid, textvariable=self.start_time_var, font=("Consolas", 27, "bold")).grid(row=0, column=1, padx=5, sticky=tk.W)
        
        ttk.Label(timing_grid, text="End Time:", font=("Arial", 14)).grid(row=0, column=2, padx=5, sticky=tk.W)
        self.end_time_var = tk.StringVar(value="--:--:--.---")
        ttk.Label(timing_grid, textvariable=self.end_time_var, font=("Consolas", 27, "bold")).grid(row=0, column=3, padx=5, sticky=tk.W)
        
        ttk.Label(timing_grid, text="Crossing Duration:", font=("Arial", 14)).grid(row=0, column=4, padx=5, sticky=tk.W)
        self.duration_var = tk.StringVar(value="--.--- s")
        ttk.Label(timing_grid, textvariable=self.duration_var, font=("Consolas", 27, "bold")).grid(row=0, column=5, padx=5, sticky=tk.W)
        
        # Row 4: Controls
        ctrl_frame = ttk.Frame(bottom_frame)
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(ctrl_frame, text="Anchor Self-Calibration", 
                   command=self._show_calibration_dialog).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Manual Anchor Settings", 
                   command=self._show_anchor_settings).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="UWB Transform Settings", 
                   command=self._show_transform_settings).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Reset Timing", 
                   command=self._reset_timing).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Start Timing", 
                   command=self._start_timing).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Recalibrate", 
                   command=self._manual_calibrate).pack(side=tk.LEFT, padx=5)
        
        # Status indicators frame
        status_container = ttk.LabelFrame(ctrl_frame, text="Connection Status", padding=3)
        status_container.pack(side=tk.RIGHT, padx=5)
        
        # UWB status
        self.uwb_data_count = 0
        self.udp_status_var = tk.StringVar(value="UWB: Waiting (UDP 8080)")
        self.udp_label = ttk.Label(status_container, textvariable=self.udp_status_var, foreground="red")
        self.udp_label.pack(side=tk.LEFT, padx=5)
        
        # GNSS status
        self.gnss_status_var = tk.StringVar(value="GNSS: Waiting (TCP 7799)")
        self.gnss_label = ttk.Label(status_container, textvariable=self.gnss_status_var, foreground="orange")
        self.gnss_label.pack(side=tk.LEFT, padx=5)
    
    def _on_data_source_change(self, event):
        """Handle data source change."""
        new_source = self.data_source_var.get()
        if new_source != self.data_source:
            self.data_source = new_source
            if new_source == 'serial':
                self.udp_status_var.set("UWB: Serial Mode")
                # Serial will be started when calibration dialog is used
            else:
                self.udp_status_var.set("UWB: Waiting (UDP 8080)")
    
    def _load_uwb_background_image(self):
        """Load the UWB background image, zoom in 2x and fill the canvas."""
        if not HAS_PIL:
            return
        
        try:
            if os.path.exists(self.uwb_bg_image_path):
                original_image = Image.open(self.uwb_bg_image_path)
                orig_width, orig_height = original_image.size
                
                crop_size = min(orig_width, orig_height) // 2
                left = (orig_width - crop_size) // 2
                top = (orig_height - crop_size) // 2
                right = left + crop_size
                bottom = top + crop_size
                
                cropped_image = original_image.crop((left, top, right, bottom))
                
                self.uwb_bg_image = cropped_image.resize(
                    (self.uwb_canvas_width, self.uwb_canvas_height),
                    Image.Resampling.LANCZOS
                )
                
                self.uwb_bg_photo = ImageTk.PhotoImage(self.uwb_bg_image)
                print(f"[UWB] Loaded background image: {self.uwb_bg_image_path}")
            else:
                print(f"[UWB] Background image not found: {self.uwb_bg_image_path}")
        except Exception as e:
            print(f"[UWB] Failed to load background image: {e}")
    
    def _show_calibration_dialog(self):
        """Show anchor self-calibration dialog."""
        def on_confirm(result):
            if result:
                self.uwb_anchors = result
                self._update_trilateration_anchors()
                if self.gnss_calibrated:
                    self._auto_calibrate()
                self._draw_uwb_map()
                print(f"[Calibration] Applied new anchor positions: {result}")
        
        dialog = AnchorCalibrationDialog(
            self.root, 
            self.uwb_anchors, 
            self.serial_receiver,
            on_confirm=on_confirm
        )
        self.root.wait_window(dialog)
    
    def _show_anchor_settings(self):
        """Show manual anchor settings dialog."""
        dialog = SettingsDialog(self.root, self.uwb_anchors)
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.uwb_anchors = dialog.result
            self._update_trilateration_anchors()
            if self.gnss_calibrated:
                self._auto_calibrate()
            self._draw_uwb_map()
    
    def _show_transform_settings(self):
        """Show transform settings dialog."""
        def apply_settings(settings):
            self.uwb_scale = settings["scale"]
            self.uwb_rotation_deg = settings["rotation_deg"]
            self.uwb_offset_x = settings["offset_x"]
            self.uwb_offset_y = settings["offset_y"]
            self._draw_uwb_map()
        
        dialog = TransformSettingsDialog(
            self.root, 
            self.uwb_scale, 
            self.uwb_rotation_deg,
            self.uwb_offset_x,
            self.uwb_offset_y,
            apply_callback=apply_settings
        )
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.uwb_scale = dialog.result["scale"]
            self.uwb_rotation_deg = dialog.result["rotation_deg"]
            self.uwb_offset_x = dialog.result["offset_x"]
            self.uwb_offset_y = dialog.result["offset_y"]
            self._draw_uwb_map()
    
    def _transform_uwb_to_canvas(self, x: float, y: float) -> Tuple[int, int]:
        """Transform UWB coordinates to canvas coordinates."""
        angle_rad = math.radians(self.uwb_rotation_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        x_rot = x * cos_a - y * sin_a
        y_rot = x * sin_a + y * cos_a
        
        x_scaled = x_rot * self.uwb_scale
        y_scaled = y_rot * self.uwb_scale
        
        x_final = x_scaled + self.uwb_offset_x
        y_final = y_scaled + self.uwb_offset_y
        
        canvas_x = self.uwb_canvas_width / 2 + x_final
        canvas_y = self.uwb_canvas_height / 2 - y_final
        
        return int(canvas_x), int(canvas_y)
    
    def _draw_uwb_map(self):
        """Draw UWB map on canvas with background image and markers."""
        if not self.uwb_canvas:
            return
        
        self.uwb_canvas.delete("all")
        
        if self.uwb_bg_photo:
            self.uwb_canvas.create_image(
                self.uwb_canvas_width // 2,
                self.uwb_canvas_height // 2,
                image=self.uwb_bg_photo,
                anchor=tk.CENTER
            )
        
        # Draw anchors
        anchor_colors = ["red", "blue", "green"]
        for i, anchor in enumerate(self.uwb_anchors):
            x, y = self._transform_uwb_to_canvas(anchor["x"], anchor["y"])
            self.uwb_canvas.create_oval(
                x - 8, y - 8, x + 8, y + 8,
                fill=anchor_colors[i], outline="black", width=2
            )
            self.uwb_canvas.create_text(
                x, y - 15,
                text=f"A{i}",
                font=("Arial", 10, "bold"),
                fill="black"
            )
        
        # Draw start line (anchor0 to anchor1)
        if len(self.uwb_anchors) >= 2:
            x0, y0 = self._transform_uwb_to_canvas(
                self.uwb_anchors[0]["x"], 
                self.uwb_anchors[0]["y"]
            )
            x1, y1 = self._transform_uwb_to_canvas(
                self.uwb_anchors[1]["x"], 
                self.uwb_anchors[1]["y"]
            )
            self.uwb_canvas.create_line(
                x0, y0, x1, y1,
                fill="red", width=3
            )
        
        # Draw tag if available
        if self.uwb_tag_pos:
            tag_x, tag_y = self._transform_uwb_to_canvas(
                self.uwb_tag_pos.x,
                self.uwb_tag_pos.y
            )
            self.uwb_canvas.create_oval(
                tag_x - 6, tag_y - 6, tag_x + 6, tag_y + 6,
                fill="orange", outline="black", width=2
            )
            self.uwb_canvas.create_text(
                tag_x, tag_y - 12,
                text="Tag",
                font=("Arial", 9, "bold"),
                fill="black"
            )
    
    def _manual_calibrate(self):
        """Manually trigger recalibration."""
        if not all(a is not None for a in self.gnss_anchors):
            messagebox.showinfo("Info", "Waiting for all 3 GNSS anchor positions...")
            return
        
        self.gnss_calibrated = True
        self._auto_calibrate()
    
    def _reset_timing(self):
        """Reset timing statistics."""
        self.timing.reset()
        self.has_crossed = False
        self.crossing_label.config(text="NOT CROSSED", foreground="red")
        self.start_time_var.set("--:--:--.---")
        self.end_time_var.set("--:--:--.---")
        self.duration_var.set("--.--- s")
    
    def _start_timing(self):
        """Start the timing."""
        self._reset_timing()
        self.timing.start_time = time.time()
        self.start_time_var.set(self._format_time(self.timing.start_time))
    
    def _format_time(self, timestamp: float) -> str:
        """Format timestamp to HH:MM:SS.mmm"""
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%H:%M:%S") + f".{int((timestamp % 1) * 1000):03d}"
    
    def _update_loop(self):
        """Main update loop."""
        try:
            # Get data from appropriate source
            if self.data_source == 'udp':
                uwb_data = self.udp_receiver.get_latest_data()
                if uwb_data:
                    self._process_uwb_data(uwb_data)
            elif self.data_source == 'serial':
                # Process serial data with trilateration
                serial_data = self.serial_receiver.get_latest_data()
                if serial_data:
                    self._process_serial_data(serial_data)
            
            # Update UWB map (Canvas)
            self._draw_uwb_map()
            
            # Update GNSS map with UWB-converted tag position (throttled to 1Hz)
            current_time = time.time()
            if current_time - self.last_gnss_tag_update_time >= self.gnss_tag_update_interval:
                self._update_gnss_tag_marker()
                self.last_gnss_tag_update_time = current_time
            
            # Update metrics display
            self._update_metrics()
            
            # Update GNSS anchor markers
            if self.gnss_calibrated and not self.gnss_anchor_markers_updated:
                self._update_anchor_markers()
                self.gnss_anchor_markers_updated = True
            
        except Exception as e:
            print(f"Update error: {e}")
        
        self.root.after(50, self._update_loop)
    
    def _process_serial_data(self, data: UWBRangingData):
        """Process serial UWB data and calculate position using trilateration."""
        current_time = time.time()
        
        # Update status
        self.uwb_data_count += 1
        self.udp_status_var.set(f"UWB: Serial ({self.uwb_data_count} pkts)")
        self.udp_label.config(foreground="green")
        
        # Calculate position using trilateration DLL
        if self.trilateration.is_loaded:
            position = self.trilateration.calculate_position(data.ranges)
            if position:
                self.prev_uwb_tag_pos = self.uwb_tag_pos
                self.uwb_tag_pos = UWBPosition(position.x, position.y, position.z)
                
                # Calculate speed
                if self.prev_uwb_tag_pos and self.last_uwb_data_time is not None:
                    dt = current_time - self.last_uwb_data_time
                    if dt > 0 and dt < 1.0:
                        dx = self.uwb_tag_pos.x - self.prev_uwb_tag_pos.x
                        dy = self.uwb_tag_pos.y - self.prev_uwb_tag_pos.y
                        dist = math.hypot(dx, dy)
                        instant_speed = dist / dt
                        
                        self.speed_history.append(instant_speed)
                        if len(self.speed_history) > 10:
                            self.speed_history.pop(0)
                        if len(self.speed_history) > 0:
                            self.current_speed = sum(self.speed_history) / len(self.speed_history)
                
                self.last_uwb_data_time = current_time
                
                # Transform to GNSS coordinates
                if self.transformer.is_calibrated:
                    self.gnss_tag_pos = self.transformer.transform(self.uwb_tag_pos)
                
                # Check line crossing
                self._check_line_crossing(current_time)
    
    def _process_uwb_data(self, data: UWBTagData):
        """Process received UWB data from UDP."""
        current_time = time.time()
        
        self.uwb_data_count += 1
        self.udp_status_var.set(f"UWB: Active ({self.uwb_data_count} pkts)")
        self.udp_label.config(foreground="green")
        
        self.prev_uwb_tag_pos = self.uwb_tag_pos
        self.uwb_tag_pos = UWBPosition(data.x, data.y, data.z)
        
        if self.uwb_data_count <= 5 or self.uwb_data_count % 100 == 0:
            print(f"[UWB] Received: X={data.x:.3f}, Y={data.y:.3f}, Z={data.z:.3f}")
        
        # Calculate speed
        if self.prev_uwb_tag_pos and self.last_uwb_data_time is not None:
            dt = current_time - self.last_uwb_data_time
            if dt > 0 and dt < 1.0:
                dx = self.uwb_tag_pos.x - self.prev_uwb_tag_pos.x
                dy = self.uwb_tag_pos.y - self.prev_uwb_tag_pos.y
                dist = math.hypot(dx, dy)
                instant_speed = dist / dt
                
                self.speed_history.append(instant_speed)
                if len(self.speed_history) > 10:
                    self.speed_history.pop(0)
                if len(self.speed_history) > 0:
                    self.current_speed = sum(self.speed_history) / len(self.speed_history)
        elif self.prev_uwb_tag_pos is None:
            self.current_speed = 0.0
        
        self.last_uwb_data_time = current_time
        
        # Transform to GNSS coordinates
        if self.transformer.is_calibrated:
            self.gnss_tag_pos = self.transformer.transform(self.uwb_tag_pos)
        
        # Check line crossing
        self._check_line_crossing(current_time)
    
    def _check_line_crossing(self, current_time: float):
        """Check if tag has crossed the start line."""
        if self.prev_uwb_tag_pos and not self.has_crossed:
            anchor0 = (self.uwb_anchors[0]["x"], self.uwb_anchors[0]["y"])
            anchor1 = (self.uwb_anchors[1]["x"], self.uwb_anchors[1]["y"])
            prev_pos = (self.prev_uwb_tag_pos.x, self.prev_uwb_tag_pos.y)
            curr_pos = (self.uwb_tag_pos.x, self.uwb_tag_pos.y)
            
            if check_line_crossing(prev_pos, curr_pos, anchor0, anchor1):
                self.has_crossed = True
                self.timing.crossing_time = current_time
                self.timing.end_time = current_time
                self.crossing_label.config(text="CROSSED", foreground="green")
                self.end_time_var.set(self._format_time(current_time))
                
                if self.timing.start_time:
                    duration = current_time - self.timing.start_time
                    self.duration_var.set(f"{duration:.3f} s")
    
    def _update_metrics(self):
        """Update speed, distance, and time metrics."""
        if not self.uwb_tag_pos:
            self.speed_var.set("0.00 m/s")
            self.distance_var.set("-- m")
            self.time_to_line_var.set("-- s")
            return
        
        self.speed_var.set(f"{self.current_speed:.2f} m/s")
        
        anchor0 = (self.uwb_anchors[0]["x"], self.uwb_anchors[0]["y"])
        anchor1 = (self.uwb_anchors[1]["x"], self.uwb_anchors[1]["y"])
        tag_pos = (self.uwb_tag_pos.x, self.uwb_tag_pos.y)
        
        distance = abs(calculate_distance_to_line(tag_pos, anchor0, anchor1))
        self.distance_var.set(f"{distance:.2f} m")
        
        if self.current_speed > 0.01:
            time_to_line = distance / self.current_speed
            if time_to_line < 1000:
                self.time_to_line_var.set(f"{time_to_line:.2f} s")
            else:
                self.time_to_line_var.set("-- s")
        else:
            self.time_to_line_var.set("-- s")
    
    def _update_anchor_markers(self):
        """Update anchor markers on GNSS map."""
        if not HAS_MAP or not self.gnss_map:
            return
        
        for marker in self.gnss_markers.values():
            try:
                marker.delete()
            except:
                pass
        self.gnss_markers.clear()
        
        for i in range(3):
            gnss = self.gnss_anchors[i]
            if gnss:
                try:
                    marker = self.gnss_map.set_marker(gnss.lat, gnss.lon, text=f"A{i}")
                    self.gnss_markers[f"anchor{i}"] = marker
                except Exception as e:
                    print(f"Error adding marker {i}: {e}")
        
        gnss0 = self.gnss_anchors[0]
        gnss1 = self.gnss_anchors[1]
        if gnss0 and gnss1:
            try:
                if self.gnss_start_line:
                    try:
                        self.gnss_start_line.delete()
                    except:
                        pass
                self.gnss_start_line = self.gnss_map.set_path(
                    [(gnss0.lat, gnss0.lon), (gnss1.lat, gnss1.lon)],
                    color="red", width=3
                )
                
                center_lat = (gnss0.lat + gnss1.lat) / 2
                center_lon = (gnss0.lon + gnss1.lon) / 2
                self.gnss_map.set_position(center_lat, center_lon)
            except Exception as e:
                print(f"Error drawing start line: {e}")
    
    def _update_gnss_tag_marker(self):
        """Update tag marker on GNSS map with UWB-converted position."""
        if not HAS_MAP or not self.gnss_map or not self.gnss_tag_pos:
            return
        
        try:
            if self.gnss_tag_marker:
                self.gnss_tag_marker.delete()
            self.gnss_tag_marker = self.gnss_map.set_marker(
                self.gnss_tag_pos.lat, self.gnss_tag_pos.lon,
                text="Tag", marker_color_circle="blue", marker_color_outside="blue"
            )
        except Exception as e:
            print(f"Error updating GNSS tag marker: {e}")
    
    def _on_close(self):
        """Handle window close."""
        self.udp_receiver.stop()
        self.gnss_receiver.stop()
        self.tcp_server.stop()
        self.serial_receiver.stop()
        self.root.destroy()


def main():
    """Main entry point."""
    root = tk.Tk()
    app = MainApplication(root)
    root.mainloop()


if __name__ == "__main__":
    main()
