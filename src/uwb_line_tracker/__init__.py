"""
UWB Line Tracker package.

Provides UWB and GNSS visualization for sailing start line tracking.

Features:
- Serial port UWB data reception and parsing
- Trilateration positioning using DLL
- Anchor self-calibration using MDS algorithm
- GNSS coordinate transformation
- Real-time visualization
"""

__all__ = [
    "config",
    "models",
    "ds_twr",
    "simulator",
    "terminal",
    "history",
    "visualizer",
    "coordinate_transform",
    "udp_receiver",
    "gnss_receiver",
    "tcp_server",
    "main_app",
    "serial_receiver",
    "trilateration_wrapper",
    "anchor_calibration",
]
