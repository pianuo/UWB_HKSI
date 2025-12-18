"""
Global configuration for the UWB line tracking demo.
"""

from __future__ import annotations

# Physical constants
SPEED_OF_LIGHT_M_S = 299_792_458.0

# Anchors define the virtual start line. Positions are in meters (x, y, z).
ANCHORS = [
    {"id": "A", "position": (0.0, 0.0, 0.0)},
    {"id": "B", "position": (25.0, 0.0, 0.0)},
]

# Simulation related parameters.
SIMULATION = {
    "tag_id": "TAG-01",
    "update_rate_hz": 100,  # Target DS-TWR update frequency
    "clock_drift_ns": 4.0,  # Deterministic offset added to ToF
    "distance_noise_std_m": 0.015,  # 1.5 cm sigma noise on range
    "responder_processing_ns": 120.0,  # Emulated responder delay
    "history_limit": 5000,
    # Vertical crossing + semicircle return pattern
    "approach_speed_m_s": 4.0,  # Speed when approaching the line
    "approach_distance_m": 15.0,  # Distance from line to start approach (below the line)
    "crossing_to_anchor_line_m": 12.5,  # Distance from line to anchor line intersection (above the line)
    "semicircle_center_anchor": "A",  # Which anchor to use as semicircle center reference
    "semicircle_radius_m": 18.0,  # Radius of the return semicircle
    "semicircle_speed_m_s": 3.5,  # Speed during semicircle turn
}

# Networking options for simulator <-> terminal communication.
NETWORK = {
    "host": "127.0.0.1",
    "port": 8765,
    "max_clients": 4,
}

# Line crossing detection preference.
LINE_OPTIONS = {
    "prefer_positive_side": True,
}




