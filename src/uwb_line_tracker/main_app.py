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


class SettingsDialog(tk.Toplevel):
    """Dialog for configuring UWB anchor positions."""
    
    def __init__(self, parent, current_anchors: list[dict]):
        super().__init__(parent)
        self.title("UWB Anchor Settings")
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
    
    def __init__(self, parent, scale: float, rotation_deg: float, offset_x: float, offset_y: float):
        super().__init__(parent)
        self.title("UWB Transform Settings")
        self.geometry("400x250")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        
        self.result = None
        self.scale = scale
        self.rotation_deg = rotation_deg
        self.offset_x = offset_x
        self.offset_y = offset_y
        
        self._create_widgets()
        self._load_values()
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        ttk.Label(main_frame, text="UWB Coordinate Transformation", 
                  font=("Arial", 11, "bold")).pack(pady=(0, 10))
        
        # Scale
        scale_frame = ttk.LabelFrame(main_frame, text="Scale", padding=5)
        scale_frame.pack(fill=tk.X, pady=5)
        ttk.Label(scale_frame, text="Scale Factor:").pack(side=tk.LEFT, padx=5)
        self.scale_entry = ttk.Entry(scale_frame, width=15)
        self.scale_entry.pack(side=tk.LEFT, padx=5)
        
        # Rotation
        rot_frame = ttk.LabelFrame(main_frame, text="Rotation", padding=5)
        rot_frame.pack(fill=tk.X, pady=5)
        ttk.Label(rot_frame, text="Rotation (degrees):").pack(side=tk.LEFT, padx=5)
        self.rotation_entry = ttk.Entry(rot_frame, width=15)
        self.rotation_entry.pack(side=tk.LEFT, padx=5)
        
        # Offset
        offset_frame = ttk.LabelFrame(main_frame, text="Offset (meters)", padding=5)
        offset_frame.pack(fill=tk.X, pady=5)
        
        offset_row = ttk.Frame(offset_frame)
        offset_row.pack(fill=tk.X)
        
        ttk.Label(offset_row, text="X:").grid(row=0, column=0, padx=(5, 2))
        self.offset_x_entry = ttk.Entry(offset_row, width=12)
        self.offset_x_entry.grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(offset_row, text="Y:").grid(row=0, column=2, padx=(5, 2))
        self.offset_y_entry = ttk.Entry(offset_row, width=12)
        self.offset_y_entry.grid(row=0, column=3, padx=(0, 5))
        
        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(20, 0))
        
        ttk.Button(btn_frame, text="OK", command=self._on_ok).pack(side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side=tk.RIGHT)
    
    def _load_values(self):
        self.scale_entry.insert(0, str(self.scale))
        self.rotation_entry.insert(0, str(self.rotation_deg))
        self.offset_x_entry.insert(0, str(self.offset_x))
        self.offset_y_entry.insert(0, str(self.offset_y))
    
    def _on_ok(self):
        try:
            self.result = {
                "scale": float(self.scale_entry.get() or 1.0),
                "rotation_deg": float(self.rotation_entry.get() or 0.0),
                "offset_x": float(self.offset_x_entry.get() or 0.0),
                "offset_y": float(self.offset_y_entry.get() or 0.0),
            }
            self.destroy()
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid value: {e}")
    
    def _on_cancel(self):
        self.result = None
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
        
        # UWB map canvas (for custom image background)
        self.uwb_canvas = None
        self.uwb_canvas_width = 450
        self.uwb_canvas_height = 400
        
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
        
        # UWB Map (Left) - Canvas with image background
        uwb_frame = ttk.LabelFrame(maps_frame, text="UWB Positioning (Real-time)", padding=5)
        uwb_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 2))
        
        self.uwb_canvas = tk.Canvas(uwb_frame, width=self.uwb_canvas_width, height=self.uwb_canvas_height, bg="white")
        self.uwb_canvas.pack(fill=tk.BOTH, expand=True)
        self.uwb_map = None  # Not using tkintermapview for UWB map
        
        # Load background image
        self._load_uwb_background_image()
        
        # GNSS Map (Right) - Smaller size
        gnss_frame = ttk.LabelFrame(maps_frame, text="GNSS Positioning (1Hz)", padding=5)
        gnss_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(2, 0))
        
        if HAS_MAP:
            self.gnss_map = tkintermapview.TkinterMapView(gnss_frame, width=450, height=400)
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
        
        # Speed (larger font)
        speed_frame = ttk.LabelFrame(metrics_frame, text="Speed", padding=5)
        speed_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.speed_var = tk.StringVar(value="0.00 m/s")
        ttk.Label(speed_frame, textvariable=self.speed_var, font=("Consolas", 24, "bold")).pack()
        
        # Distance to line (larger font)
        dist_frame = ttk.LabelFrame(metrics_frame, text="Distance to Start Line", padding=5)
        dist_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.distance_var = tk.StringVar(value="0.00 m")
        ttk.Label(dist_frame, textvariable=self.distance_var, font=("Consolas", 24, "bold")).pack()
        
        # Time to line (larger font)
        time_frame = ttk.LabelFrame(metrics_frame, text="Time to Start Line", padding=5)
        time_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.time_to_line_var = tk.StringVar(value="-- s")
        ttk.Label(time_frame, textvariable=self.time_to_line_var, font=("Consolas", 24, "bold")).pack()
        
        # Row 3: Timing statistics
        timing_frame = ttk.LabelFrame(bottom_frame, text="Timing Statistics", padding=5)
        timing_frame.pack(fill=tk.X, pady=5)
        
        timing_grid = ttk.Frame(timing_frame)
        timing_grid.pack(fill=tk.X)
        
        ttk.Label(timing_grid, text="Start Time:", font=("Arial", 11)).grid(row=0, column=0, padx=5, sticky=tk.W)
        self.start_time_var = tk.StringVar(value="--:--:--.---")
        ttk.Label(timing_grid, textvariable=self.start_time_var, font=("Consolas", 18, "bold")).grid(row=0, column=1, padx=5, sticky=tk.W)
        
        ttk.Label(timing_grid, text="End Time:", font=("Arial", 11)).grid(row=0, column=2, padx=5, sticky=tk.W)
        self.end_time_var = tk.StringVar(value="--:--:--.---")
        ttk.Label(timing_grid, textvariable=self.end_time_var, font=("Consolas", 18, "bold")).grid(row=0, column=3, padx=5, sticky=tk.W)
        
        ttk.Label(timing_grid, text="Crossing Duration:", font=("Arial", 11)).grid(row=0, column=4, padx=5, sticky=tk.W)
        self.duration_var = tk.StringVar(value="--.--- s")
        ttk.Label(timing_grid, textvariable=self.duration_var, font=("Consolas", 18, "bold")).grid(row=0, column=5, padx=5, sticky=tk.W)
        
        # Row 4: Controls
        ctrl_frame = ttk.Frame(bottom_frame)
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(ctrl_frame, text="Configure UWB Anchors", 
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
        """Load the UWB background image."""
        if not HAS_PIL:
            return
        
        try:
            if os.path.exists(self.uwb_bg_image_path):
                self.uwb_bg_image = Image.open(self.uwb_bg_image_path)
                # Resize to fit canvas
                self.uwb_bg_image = self.uwb_bg_image.resize(
                    (self.uwb_canvas_width, self.uwb_canvas_height),
                    Image.Resampling.LANCZOS
                )
                self.uwb_bg_photo = ImageTk.PhotoImage(self.uwb_bg_image)
                print(f"[UWB] Loaded background image: {self.uwb_bg_image_path}")
            else:
                print(f"[UWB] Background image not found: {self.uwb_bg_image_path}")
        except Exception as e:
            print(f"[UWB] Failed to load background image: {e}")
    
    def _show_anchor_settings(self):
        """Show anchor settings dialog."""
        dialog = SettingsDialog(self.root, self.uwb_anchors)
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.uwb_anchors = dialog.result
            if self.gnss_calibrated:
                self._auto_calibrate()
            self._draw_uwb_map()
    
    def _show_transform_settings(self):
        """Show transform settings dialog."""
        dialog = TransformSettingsDialog(
            self.root, 
            self.uwb_scale, 
            self.uwb_rotation_deg,
            self.uwb_offset_x,
            self.uwb_offset_y
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
