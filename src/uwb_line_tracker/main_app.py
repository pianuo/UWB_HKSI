"""
Main application for UWB and GNSS visualization.
Features:
- Dual map display (UWB and GNSS on OpenStreetMap)
- Start line crossing detection
- Speed, distance, and time calculations
- Timing statistics
- GNSS anchor data reception via TCP/ZMQ
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
from typing import Optional, Tuple
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
from .anchor_calibration import RealTimeCalibrator, AnchorPosition, CalibrationResult
from .trilateration import TrilaterationEngine, UWBPositionCalculator


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
    Dialog for UWB Anchor Self-Calibration.
    
    Features:
    - "Calibrate" button: Gets real-time calibration data and displays it
    - "Confirm" button: Applies the calibrated positions as anchor coordinates
    
    Based on RTLSClient.cpp lines 1046-1179 algorithm.
    """
    
    def __init__(self, parent, calibrator: RealTimeCalibrator, apply_callback=None):
        super().__init__(parent)
        self.title("UWB Anchor Self-Calibration")
        self.geometry("550x550")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        
        self.calibrator = calibrator
        self.apply_callback = apply_callback
        self.result = None
        self.calibration_result: Optional[CalibrationResult] = None
        
        self._create_widgets()
        self._update_status()
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        ttk.Label(main_frame, text="UWB Anchor Self-Calibration", 
                  font=("Arial", 14, "bold")).pack(pady=(0, 10))
        
        # Instructions
        info_text = (
            "This tool calibrates anchor positions using inter-anchor range measurements.\n"
            "Algorithm: MDS (Multi-Dimensional Scaling) + Angle Rotation\n"
            "Result: A0 at origin, A1 on X-axis, A2 in first quadrant"
        )
        ttk.Label(main_frame, text=info_text, font=("Arial", 9), 
                  foreground="gray", justify=tk.LEFT).pack(pady=(0, 10))
        
        # Distance Matrix Display
        dist_frame = ttk.LabelFrame(main_frame, text="Inter-Anchor Distances (meters)", padding=5)
        dist_frame.pack(fill=tk.X, pady=5)
        
        self.distance_labels = {}
        dist_grid = ttk.Frame(dist_frame)
        dist_grid.pack(fill=tk.X)
        
        # Headers
        ttk.Label(dist_grid, text="Pair", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5)
        ttk.Label(dist_grid, text="Count", font=("Arial", 10, "bold")).grid(row=0, column=1, padx=5)
        ttk.Label(dist_grid, text="Avg Distance", font=("Arial", 10, "bold")).grid(row=0, column=2, padx=5)
        
        # Distance rows
        pairs = ["A0-A1", "A0-A2", "A1-A2"]
        for i, pair in enumerate(pairs):
            ttk.Label(dist_grid, text=pair, font=("Consolas", 10)).grid(row=i+1, column=0, padx=5, sticky=tk.W)
            count_label = ttk.Label(dist_grid, text="0", font=("Consolas", 10))
            count_label.grid(row=i+1, column=1, padx=5)
            dist_label = ttk.Label(dist_grid, text="-- m", font=("Consolas", 10))
            dist_label.grid(row=i+1, column=2, padx=5)
            self.distance_labels[pair] = {"count": count_label, "distance": dist_label}
        
        # Manual Distance Entry
        manual_frame = ttk.LabelFrame(main_frame, text="Manual Distance Entry (mm)", padding=5)
        manual_frame.pack(fill=tk.X, pady=5)
        
        manual_row = ttk.Frame(manual_frame)
        manual_row.pack(fill=tk.X)
        
        ttk.Label(manual_row, text="A0-A1:").grid(row=0, column=0, padx=2)
        self.dist_01_entry = ttk.Entry(manual_row, width=10)
        self.dist_01_entry.grid(row=0, column=1, padx=2)
        
        ttk.Label(manual_row, text="A0-A2:").grid(row=0, column=2, padx=2)
        self.dist_02_entry = ttk.Entry(manual_row, width=10)
        self.dist_02_entry.grid(row=0, column=3, padx=2)
        
        ttk.Label(manual_row, text="A1-A2:").grid(row=0, column=4, padx=2)
        self.dist_12_entry = ttk.Entry(manual_row, width=10)
        self.dist_12_entry.grid(row=0, column=5, padx=2)
        
        ttk.Button(manual_row, text="Add", command=self._add_manual_distances).grid(row=0, column=6, padx=5)
        
        # Calibrated Positions Display
        pos_frame = ttk.LabelFrame(main_frame, text="Calibrated Anchor Positions", padding=5)
        pos_frame.pack(fill=tk.X, pady=5)
        
        self.position_labels = {}
        pos_grid = ttk.Frame(pos_frame)
        pos_grid.pack(fill=tk.X)
        
        ttk.Label(pos_grid, text="Anchor", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5)
        ttk.Label(pos_grid, text="X (m)", font=("Arial", 10, "bold")).grid(row=0, column=1, padx=5)
        ttk.Label(pos_grid, text="Y (m)", font=("Arial", 10, "bold")).grid(row=0, column=2, padx=5)
        ttk.Label(pos_grid, text="Z (m)", font=("Arial", 10, "bold")).grid(row=0, column=3, padx=5)
        
        for i in range(3):
            ttk.Label(pos_grid, text=f"A{i}", font=("Consolas", 10)).grid(row=i+1, column=0, padx=5, sticky=tk.W)
            x_label = ttk.Label(pos_grid, text="--", font=("Consolas", 10))
            x_label.grid(row=i+1, column=1, padx=5)
            y_label = ttk.Label(pos_grid, text="--", font=("Consolas", 10))
            y_label.grid(row=i+1, column=2, padx=5)
            z_label = ttk.Label(pos_grid, text="0.00", font=("Consolas", 10))
            z_label.grid(row=i+1, column=3, padx=5)
            self.position_labels[f"A{i}"] = {"x": x_label, "y": y_label, "z": z_label}
        
        # Status
        self.status_var = tk.StringVar(value="Waiting for distance measurements...")
        self.status_label = ttk.Label(main_frame, textvariable=self.status_var, 
                                       font=("Arial", 10), foreground="orange")
        self.status_label.pack(pady=10)
        
        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(btn_frame, text="Calibrate", command=self._calibrate, 
                   width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Reset", command=self._reset, 
                   width=12).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(btn_frame, text="Close", command=self._on_close, 
                   width=12).pack(side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="Confirm", command=self._on_confirm, 
                   width=15).pack(side=tk.RIGHT, padx=5)
    
    def _add_manual_distances(self):
        """Add manual distance measurements."""
        try:
            d01 = self.dist_01_entry.get().strip()
            d02 = self.dist_02_entry.get().strip()
            d12 = self.dist_12_entry.get().strip()
            
            if d01:
                self.calibrator.add_measurement(0, 1, float(d01) / 1000.0)
            if d02:
                self.calibrator.add_measurement(0, 2, float(d02) / 1000.0)
            if d12:
                self.calibrator.add_measurement(1, 2, float(d12) / 1000.0)
            
            self._update_status()
            self.status_var.set("Manual distances added")
            self.status_label.config(foreground="blue")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid distance value: {e}")
    
    def _update_status(self):
        """Update distance and status display."""
        status = self.calibrator.get_measurement_status()
        
        for pair, info in status.items():
            if pair in self.distance_labels:
                self.distance_labels[pair]["count"].config(text=str(info["count"]))
                if info["count"] > 0:
                    self.distance_labels[pair]["distance"].config(
                        text=f"{info['average_distance']:.3f} m"
                    )
                else:
                    self.distance_labels[pair]["distance"].config(text="-- m")
        
        if self.calibrator.has_sufficient_data():
            self.status_var.set("Ready to calibrate")
            self.status_label.config(foreground="green")
        else:
            self.status_var.set("Need more measurements")
            self.status_label.config(foreground="orange")
    
    def _calibrate(self):
        """Perform calibration."""
        if not self.calibrator.has_sufficient_data():
            messagebox.showwarning("Warning", "Insufficient data for calibration.\nNeed at least one measurement for each anchor pair.")
            return
        
        result = self.calibrator.perform_calibration()
        self.calibration_result = result
        
        if result.success:
            # Update position display
            for anchor in result.anchors:
                key = f"A{anchor.anchor_id}"
                if key in self.position_labels:
                    self.position_labels[key]["x"].config(text=f"{anchor.x:.3f}")
                    self.position_labels[key]["y"].config(text=f"{anchor.y:.3f}")
                    self.position_labels[key]["z"].config(text=f"{anchor.z:.3f}")
            
            self.status_var.set("Calibration successful!")
            self.status_label.config(foreground="green")
        else:
            self.status_var.set(f"Calibration failed: {result.error_message}")
            self.status_label.config(foreground="red")
    
    def _reset(self):
        """Reset calibration data."""
        self.calibrator.reset()
        self.calibration_result = None
        
        # Clear position display
        for key in self.position_labels:
            self.position_labels[key]["x"].config(text="--")
            self.position_labels[key]["y"].config(text="--")
        
        # Clear distance entries
        self.dist_01_entry.delete(0, tk.END)
        self.dist_02_entry.delete(0, tk.END)
        self.dist_12_entry.delete(0, tk.END)
        
        self._update_status()
        self.status_var.set("Reset complete")
        self.status_label.config(foreground="blue")
    
    def _on_confirm(self):
        """Confirm and apply calibrated positions."""
        if not self.calibration_result or not self.calibration_result.success:
            messagebox.showwarning("Warning", "No valid calibration result to confirm.\nPlease calibrate first.")
            return
        
        # Convert to anchor dict format
        self.result = []
        for anchor in self.calibration_result.anchors:
            self.result.append({
                "x": anchor.x,
                "y": anchor.y,
                "z": anchor.z
            })
        
        if self.apply_callback:
            self.apply_callback(self.result)
        
        self.status_var.set("Positions confirmed and applied!")
        self.status_label.config(foreground="green")
    
    def _on_close(self):
        """Close the dialog."""
        self.destroy()


class SettingsDialog(tk.Toplevel):
    """Dialog for configuring UWB anchor positions manually."""
    
    def __init__(self, parent, current_anchors: list[dict]):
        super().__init__(parent)
        self.title("UWB Anchor Settings (Manual)")
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
        self.apply_callback = apply_callback  # Callback function to apply settings without closing
        
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
            
            # Update internal values
            self.scale = new_settings["scale"]
            self.rotation_deg = new_settings["rotation_deg"]
            self.offset_x = new_settings["offset_x"]
            self.offset_y = new_settings["offset_y"]
            
            # Call callback to apply settings and refresh map
            if self.apply_callback:
                self.apply_callback(new_settings)
            
            # Show success message
            self.status_label.config(text="Settings applied!", foreground="green")
            self.after(1000, lambda: self.status_label.config(text="", foreground="black"))
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid value: {e}")
    
    def _on_close(self):
        """Close the dialog."""
        # Save final values to result before closing
        try:
            self.result = {
                "scale": float(self.scale_entry.get() or 1.0),
                "rotation_deg": float(self.rotation_entry.get() or 0.0),
                "offset_x": float(self.offset_x_entry.get() or 0.0),
                "offset_y": float(self.offset_y_entry.get() or 0.0),
            }
        except ValueError:
            # If invalid, use current values
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
            # Run tcp_server.py as a module
            self._process = subprocess.Popen(
                [sys.executable, "-m", "uwb_line_tracker.tcp_server"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            
            # Start thread to read output
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
        
        # Anchor self-calibration
        self.anchor_calibrator = RealTimeCalibrator(num_anchors=3)
        
        # Trilateration engine for position calculation
        self.trilateration_engine = TrilaterationEngine()
        self.use_trilateration = False  # Toggle for using trilateration vs UDP position
        
        # UWB anchor positions (to be configured)
        self.uwb_anchors = [
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 25.0, "y": 0.0, "z": 0.0},
            {"x": 12.5, "y": 10.0, "z": 0.0},
        ]
        
        # UWB anchor transformation parameters (for image overlay)
        self.uwb_scale = 1.0  # Scale factor
        self.uwb_rotation_deg = 0.0  # Rotation in degrees
        self.uwb_offset_x = 0.0  # X offset in meters
        self.uwb_offset_y = 0.0  # Y offset in meters
        
        # UWB background image
        self.uwb_bg_image_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            "UWB底图.png"
        )
        self.uwb_bg_image = None
        self.uwb_bg_photo = None
        
        # UWB map canvas (for custom image background) - Square shape
        self.uwb_canvas = None
        self.uwb_canvas_width = 450  # Square: 300x300
        self.uwb_canvas_height = 450
        
        # GNSS tag update throttling (1 second)
        self.last_gnss_tag_update_time = 0.0
        self.gnss_tag_update_interval = 1.0  # 1 second
        
        # GNSS anchor positions (from GNSS data)
        self.gnss_anchors: list[Optional[GNSSPosition]] = [None, None, None]
        self.gnss_calibrated = False
        self.gnss_anchor_markers_updated = False
        
        # GNSS Tag position (direct from GNSS, for right map)
        self.gnss_direct_tag_pos: Optional[GNSSPosition] = None
        
        # Tag tracking (UWB)
        self.uwb_tag_pos: Optional[UWBPosition] = None
        self.gnss_tag_pos: Optional[GNSSPosition] = None  # UWB converted to GNSS
        self.prev_uwb_tag_pos: Optional[UWBPosition] = None
        self.prev_update_time: Optional[float] = None
        self.last_uwb_data_time: Optional[float] = None  # Timestamp of last UWB data
        
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
        
        # Give TCP server time to start
        time.sleep(0.5)
        
        # Start GNSS receiver (ZMQ)
        self.gnss_receiver.set_callback(self._on_gnss_data)
        self.gnss_receiver.start()
        
        # Start UDP receiver for UWB
        self.udp_receiver.start()
    
    def _on_gnss_data(self, data: GNSSData):
        """Callback when GNSS data is received."""
        # Update anchor positions (client_id 0, 1, 2 are anchors)
        if data.client_id in [0, 1, 2]:
            self.gnss_anchors[data.client_id] = GNSSPosition(
                lat=data.lat, lon=data.lng, alt=data.alt
            )
            
            # Update GNSS status
            anchor_count = sum(1 for a in self.gnss_anchors if a is not None)
            self.root.after(0, lambda: self._update_gnss_status(anchor_count))
            
            # Check if we have all anchors for calibration
            if not self.gnss_calibrated and all(a is not None for a in self.gnss_anchors):
                self.gnss_calibrated = True
                # Schedule calibration on main thread
                self.root.after(100, self._auto_calibrate)
        else:
            # Other client_ids are tags - update direct GNSS tag position
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
        # Pack without expand to maintain square shape
        uwb_frame.pack(side=tk.LEFT, padx=(0, 2))
        
        # Create canvas with fixed square size
        self.uwb_canvas = tk.Canvas(
            uwb_frame, 
            width=self.uwb_canvas_width, 
            height=self.uwb_canvas_height, 
            bg="white", 
            highlightthickness=0
        )
        # Pack canvas without fill/expand to maintain exact square size
        self.uwb_canvas.pack()
        self.uwb_map = None  # Not using tkintermapview for UWB map
        
        # Load background image
        self._load_uwb_background_image()
        
        # GNSS Map (Right) - Smaller size (matching UWB square)
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
        
        # Row 2: Metrics
        metrics_frame = ttk.Frame(bottom_frame)
        metrics_frame.pack(fill=tk.X, pady=5)
        
        # Speed (larger font - 1.5x: 24 -> 36)
        speed_frame = ttk.LabelFrame(metrics_frame, text="Speed", padding=8)
        speed_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.speed_var = tk.StringVar(value="0.00 m/s")
        ttk.Label(speed_frame, textvariable=self.speed_var, font=("Consolas", 36, "bold")).pack()
        
        # Distance to line (larger font - 1.5x: 24 -> 36)
        dist_frame = ttk.LabelFrame(metrics_frame, text="Distance to Start Line", padding=8)
        dist_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.distance_var = tk.StringVar(value="0.00 m")
        ttk.Label(dist_frame, textvariable=self.distance_var, font=("Consolas", 36, "bold")).pack()
        
        # Time to line (larger font - 1.5x: 24 -> 36)
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
        
        # Anchor configuration buttons
        ttk.Button(ctrl_frame, text="Anchor Self-Calibration", 
                   command=self._show_anchor_settings).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Manual Anchor Settings", 
                   command=self._show_manual_anchor_settings).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="UWB Transform Settings", 
                   command=self._show_transform_settings).pack(side=tk.LEFT, padx=5)
        
        # Separator
        ttk.Separator(ctrl_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # Timing buttons
        ttk.Button(ctrl_frame, text="Reset Timing", 
                   command=self._reset_timing).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Start Timing", 
                   command=self._start_timing).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Recalibrate GNSS", 
                   command=self._manual_calibrate).pack(side=tk.LEFT, padx=5)
        
        # Status indicators frame
        status_container = ttk.LabelFrame(ctrl_frame, text="Connection Status", padding=3)
        status_container.pack(side=tk.RIGHT, padx=5)
        
        # UWB status with data counter
        self.uwb_data_count = 0
        self.udp_status_var = tk.StringVar(value="UWB: Waiting (UDP 8080)")
        self.udp_label = ttk.Label(status_container, textvariable=self.udp_status_var, foreground="red")
        self.udp_label.pack(side=tk.LEFT, padx=5)
        
        # GNSS status
        self.gnss_status_var = tk.StringVar(value="GNSS: Waiting (TCP 7799)")
        self.gnss_label = ttk.Label(status_container, textvariable=self.gnss_status_var, foreground="orange")
        self.gnss_label.pack(side=tk.LEFT, padx=5)
    
    def _load_uwb_background_image(self):
        """Load the UWB background image, zoom in 2x and fill the canvas."""
        if not HAS_PIL:
            return
        
        try:
            if os.path.exists(self.uwb_bg_image_path):
                # Load original image
                original_image = Image.open(self.uwb_bg_image_path)
                orig_width, orig_height = original_image.size
                
                # Calculate crop area for 2x zoom (show center portion)
                # Crop to show 1/2 of the original (center crop)
                crop_size = min(orig_width, orig_height) // 2
                left = (orig_width - crop_size) // 2
                top = (orig_height - crop_size) // 2
                right = left + crop_size
                bottom = top + crop_size
                
                # Crop center portion
                cropped_image = original_image.crop((left, top, right, bottom))
                
                # Resize cropped image to fill canvas (2x zoom effect)
                self.uwb_bg_image = cropped_image.resize(
                    (self.uwb_canvas_width, self.uwb_canvas_height),
                    Image.Resampling.LANCZOS
                )
                
                self.uwb_bg_photo = ImageTk.PhotoImage(self.uwb_bg_image)
                print(f"[UWB] Loaded background image: {self.uwb_bg_image_path}")
                print(f"[UWB] Original: {orig_width}x{orig_height}, Cropped (2x zoom): {crop_size}x{crop_size}, Canvas: {self.uwb_canvas_width}x{self.uwb_canvas_height}")
            else:
                print(f"[UWB] Background image not found: {self.uwb_bg_image_path}")
        except Exception as e:
            print(f"[UWB] Failed to load background image: {e}")
    
    def _show_anchor_settings(self):
        """Show anchor self-calibration dialog."""
        def apply_calibrated_anchors(anchors):
            """Apply calibrated anchor positions."""
            self.uwb_anchors = anchors
            # Update trilateration engine
            self.trilateration_engine.set_anchors(anchors)
            if self.gnss_calibrated:
                self._auto_calibrate()
            self._draw_uwb_map()
            print(f"[Anchor Calibration] Applied new positions:")
            for i, a in enumerate(anchors):
                print(f"  A{i}: ({a['x']:.3f}, {a['y']:.3f}, {a['z']:.3f})")
        
        dialog = AnchorCalibrationDialog(
            self.root, 
            self.anchor_calibrator,
            apply_callback=apply_calibrated_anchors
        )
        self.root.wait_window(dialog)
        
        # If user confirmed calibration
        if dialog.result:
            apply_calibrated_anchors(dialog.result)
    
    def _show_manual_anchor_settings(self):
        """Show manual anchor settings dialog."""
        dialog = SettingsDialog(self.root, self.uwb_anchors)
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.uwb_anchors = dialog.result
            self.trilateration_engine.set_anchors(dialog.result)
            if self.gnss_calibrated:
                self._auto_calibrate()
            self._draw_uwb_map()
    
    def _show_transform_settings(self):
        """Show transform settings dialog."""
        def apply_settings(settings):
            """Callback to apply settings immediately."""
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
            apply_callback=apply_settings  # Pass callback function
        )
        self.root.wait_window(dialog)
        
        # Final update when dialog closes (in case user changed values but didn't click Apply)
        if dialog.result:
            self.uwb_scale = dialog.result["scale"]
            self.uwb_rotation_deg = dialog.result["rotation_deg"]
            self.uwb_offset_x = dialog.result["offset_x"]
            self.uwb_offset_y = dialog.result["offset_y"]
            self._draw_uwb_map()
    
    def _transform_uwb_to_canvas(self, x: float, y: float) -> Tuple[int, int]:
        """Transform UWB coordinates to canvas coordinates."""
        # Apply rotation
        angle_rad = math.radians(self.uwb_rotation_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # Rotate around origin
        x_rot = x * cos_a - y * sin_a
        y_rot = x * sin_a + y * cos_a
        
        # Apply scale
        x_scaled = x_rot * self.uwb_scale
        y_scaled = y_rot * self.uwb_scale
        
        # Apply offset
        x_final = x_scaled + self.uwb_offset_x
        y_final = y_scaled + self.uwb_offset_y
        
        # Convert to canvas coordinates (center origin, Y flipped)
        canvas_x = self.uwb_canvas_width / 2 + x_final
        canvas_y = self.uwb_canvas_height / 2 - y_final  # Flip Y axis
        
        return int(canvas_x), int(canvas_y)
    
    def _draw_uwb_map(self):
        """Draw UWB map on canvas with background image and markers."""
        if not self.uwb_canvas:
            return
        
        # Clear canvas
        self.uwb_canvas.delete("all")
        
        # Draw background image
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
            # Draw anchor circle
            self.uwb_canvas.create_oval(
                x - 8, y - 8, x + 8, y + 8,
                fill=anchor_colors[i], outline="black", width=2
            )
            # Draw anchor label
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
            # Draw tag marker
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
            # Get latest UWB data
            uwb_data = self.udp_receiver.get_latest_data()
            if uwb_data:
                self._process_uwb_data(uwb_data)
            
            # Update UWB map (Canvas)
            self._draw_uwb_map()
            
            # Update GNSS map with UWB-converted tag position (throttled to 1Hz)
            current_time = time.time()
            if current_time - self.last_gnss_tag_update_time >= self.gnss_tag_update_interval:
                self._update_gnss_tag_marker()
                self.last_gnss_tag_update_time = current_time
            
            # Always update metrics display (even if no new data)
            self._update_metrics()
            
            # Update GNSS anchor markers if we have new data
            if self.gnss_calibrated and not self.gnss_anchor_markers_updated:
                self._update_anchor_markers()
                self.gnss_anchor_markers_updated = True
            
        except Exception as e:
            print(f"Update error: {e}")
        
        # Schedule next update
        self.root.after(50, self._update_loop)  # 20Hz update rate
    
    def _process_uwb_data(self, data: UWBTagData):
        """Process received UWB data."""
        current_time = time.time()
        
        # Update UWB data counter and status
        self.uwb_data_count += 1
        self.udp_status_var.set(f"UWB: Active ({self.uwb_data_count} pkts)")
        self.udp_label.config(foreground="green")
        
        # Store previous position for speed calculation
        self.prev_uwb_tag_pos = self.uwb_tag_pos
        self.uwb_tag_pos = UWBPosition(data.x, data.y, data.z)
        
        # Debug: Print received position
        if self.uwb_data_count <= 5 or self.uwb_data_count % 100 == 0:
            print(f"[UWB] Received: X={data.x:.3f}, Y={data.y:.3f}, Z={data.z:.3f}")
        
        # Calculate speed using data timestamps
        if self.prev_uwb_tag_pos and self.last_uwb_data_time is not None:
            # Use actual time difference between data packets
            dt = current_time - self.last_uwb_data_time
            if dt > 0 and dt < 1.0:  # Sanity check: dt should be reasonable
                dx = self.uwb_tag_pos.x - self.prev_uwb_tag_pos.x
                dy = self.uwb_tag_pos.y - self.prev_uwb_tag_pos.y
                dist = math.hypot(dx, dy)
                instant_speed = dist / dt
                
                # Smooth speed with moving average
                self.speed_history.append(instant_speed)
                if len(self.speed_history) > 10:
                    self.speed_history.pop(0)
                if len(self.speed_history) > 0:
                    self.current_speed = sum(self.speed_history) / len(self.speed_history)
        elif self.prev_uwb_tag_pos is None:
            # First data packet, initialize
            self.current_speed = 0.0
        
        # Store timestamp for next calculation
        self.last_uwb_data_time = current_time
        
        # Transform to GNSS coordinates
        if self.transformer.is_calibrated:
            self.gnss_tag_pos = self.transformer.transform(self.uwb_tag_pos)
        
        # Check line crossing
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
        # Always update display, even if no data yet
        if not self.uwb_tag_pos:
            self.speed_var.set("0.00 m/s")
            self.distance_var.set("-- m")
            self.time_to_line_var.set("-- s")
            return
        
        # Display current speed (always update)
        self.speed_var.set(f"{self.current_speed:.2f} m/s")
        
        # Calculate distance to start line (always update)
        anchor0 = (self.uwb_anchors[0]["x"], self.uwb_anchors[0]["y"])
        anchor1 = (self.uwb_anchors[1]["x"], self.uwb_anchors[1]["y"])
        tag_pos = (self.uwb_tag_pos.x, self.uwb_tag_pos.y)
        
        distance = abs(calculate_distance_to_line(tag_pos, anchor0, anchor1))
        self.distance_var.set(f"{distance:.2f} m")
        
        # Calculate time to line (always update)
        if self.current_speed > 0.01:  # Avoid division by very small numbers
            time_to_line = distance / self.current_speed
            if time_to_line < 1000:  # Reasonable upper limit
                self.time_to_line_var.set(f"{time_to_line:.2f} s")
            else:
                self.time_to_line_var.set("-- s")
        else:
            self.time_to_line_var.set("-- s")
    
    def _update_anchor_markers(self):
        """Update anchor markers on GNSS map."""
        if not HAS_MAP or not self.gnss_map:
            return
        
        # Clear existing markers
        for marker in self.gnss_markers.values():
            try:
                marker.delete()
            except:
                pass
        self.gnss_markers.clear()
        
        # Add anchor markers from GNSS data (GNSS map only)
        for i in range(3):
            gnss = self.gnss_anchors[i]
            if gnss:
                try:
                    # GNSS map
                    marker = self.gnss_map.set_marker(gnss.lat, gnss.lon, text=f"A{i}")
                    self.gnss_markers[f"anchor{i}"] = marker
                except Exception as e:
                    print(f"Error adding marker {i}: {e}")
        
        # Draw start line (anchor0 to anchor1) on GNSS map
        gnss0 = self.gnss_anchors[0]
        gnss1 = self.gnss_anchors[1]
        if gnss0 and gnss1:
            try:
                # GNSS map start line
                if self.gnss_start_line:
                    try:
                        self.gnss_start_line.delete()
                    except:
                        pass
                self.gnss_start_line = self.gnss_map.set_path(
                    [(gnss0.lat, gnss0.lon), (gnss1.lat, gnss1.lon)],
                    color="red", width=3
                )
                
                # Center GNSS map on anchors
                center_lat = (gnss0.lat + gnss1.lat) / 2
                center_lon = (gnss0.lon + gnss1.lon) / 2
                self.gnss_map.set_position(center_lat, center_lon)
            except Exception as e:
                print(f"Error drawing start line: {e}")
    
    def _update_uwb_tag_marker(self):
        """Update tag marker on UWB map (left map) - now handled by _draw_uwb_map."""
        # UWB map is now drawn on Canvas, so this function is no longer needed
        # Tag is drawn in _draw_uwb_map()
        pass
    
    def _update_gnss_tag_marker(self):
        """Update tag marker on GNSS map (right map) with UWB-converted position."""
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
    
    def set_gnss_anchors(self, anchors: list[GNSSPosition]):
        """Set GNSS anchor positions (for testing/manual configuration)."""
        for i, gnss in enumerate(anchors[:3]):
            self.gnss_anchors[i] = gnss
        
        if all(a is not None for a in self.gnss_anchors):
            self.gnss_calibrated = True
            self._auto_calibrate()
    
    def _on_close(self):
        """Handle window close."""
        self.udp_receiver.stop()
        self.gnss_receiver.stop()
        self.tcp_server.stop()
        self.root.destroy()


def main():
    """Main entry point."""
    root = tk.Tk()
    app = MainApplication(root)
    root.mainloop()


if __name__ == "__main__":
    main()
