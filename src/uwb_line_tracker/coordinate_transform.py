"""
UWB to GPS coordinate transformation module.
Based on the algorithm described in uwb2llh.md
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


# WGS84 ellipsoid parameters
WGS84_A = 6378137.0  # Semi-major axis (m)
WGS84_B = 6356752.314245  # Semi-minor axis (m)
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2  # First eccentricity squared


@dataclass
class GNSSPosition:
    """GPS position in lat/lon/alt."""
    lat: float  # Latitude in degrees
    lon: float  # Longitude in degrees
    alt: float  # Altitude in meters


@dataclass
class UWBPosition:
    """UWB position in local coordinates."""
    x: float
    y: float
    z: float


@dataclass
class ENUPosition:
    """Position in ENU (East-North-Up) coordinates."""
    e: float  # East
    n: float  # North
    u: float  # Up


def lla_to_ecef(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """
    Convert LLA (Latitude, Longitude, Altitude) to ECEF coordinates.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters
        
    Returns:
        (x, y, z) in ECEF coordinates (meters)
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    
    # Radius of curvature in the prime vertical
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
    
    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt) * sin_lat
    
    return x, y, z


def ecef_to_enu(x: float, y: float, z: float, 
                ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert ECEF to ENU coordinates relative to a reference point.
    
    Args:
        x, y, z: ECEF coordinates (meters)
        ref_lat, ref_lon, ref_alt: Reference point in LLA
        
    Returns:
        (e, n, u) in ENU coordinates (meters)
    """
    # Get reference point in ECEF
    ref_x, ref_y, ref_z = lla_to_ecef(ref_lat, ref_lon, ref_alt)
    
    # Difference vector
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z
    
    # Rotation matrix from ECEF to ENU
    lat_rad = math.radians(ref_lat)
    lon_rad = math.radians(ref_lon)
    
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    
    # ENU transformation
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    
    return e, n, u


def enu_to_lla(e: float, n: float, u: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert ENU coordinates to LLA (Latitude, Longitude, Altitude).
    
    Args:
        e, n, u: ENU coordinates (meters) relative to reference
        ref_lat, ref_lon, ref_alt: Reference point in LLA
        
    Returns:
        (lat, lon, alt) in degrees and meters
    """
    lat_rad = math.radians(ref_lat)
    lon_rad = math.radians(ref_lon)
    
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    
    # Convert ENU to ECEF delta
    dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u
    dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u
    dz = cos_lat * n + sin_lat * u
    
    # Get reference ECEF
    ref_x, ref_y, ref_z = lla_to_ecef(ref_lat, ref_lon, ref_alt)
    
    # Target ECEF
    x = ref_x + dx
    y = ref_y + dy
    z = ref_z + dz
    
    # Convert ECEF to LLA
    return ecef_to_lla(x, y, z)


def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Convert ECEF coordinates to LLA.
    
    Args:
        x, y, z: ECEF coordinates (meters)
        
    Returns:
        (lat, lon, alt) in degrees and meters
    """
    # Longitude
    lon = math.atan2(y, x)
    
    # Iterative calculation for latitude and altitude
    p = math.sqrt(x ** 2 + y ** 2)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    
    for _ in range(10):  # Usually converges in 2-3 iterations
        sin_lat = math.sin(lat)
        N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        lat = math.atan2(z + WGS84_E2 * N * sin_lat, p)
    
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
    
    if abs(cos_lat) > 1e-10:
        alt = p / cos_lat - N
    else:
        alt = abs(z) - WGS84_B
    
    return math.degrees(lat), math.degrees(lon), alt


def normalize(v: np.ndarray) -> np.ndarray:
    """Normalize a vector."""
    norm = np.linalg.norm(v)
    if norm < 1e-10:
        return v
    return v / norm


class UWBToGPSTransformer:
    """
    Transforms UWB local coordinates to GPS global coordinates.
    Uses 3 anchor points with known positions in both coordinate systems.
    """
    
    def __init__(self):
        self._is_calibrated = False
        self._R: Optional[np.ndarray] = None  # Rotation matrix
        self._t: Optional[np.ndarray] = None  # Translation vector
        self._ref_gnss: Optional[GNSSPosition] = None  # Reference GNSS position (anchor0)
        
        # Store anchor positions
        self._uwb_anchors: list[UWBPosition] = []
        self._gnss_anchors: list[GNSSPosition] = []
    
    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated
    
    def calibrate(self, 
                  uwb_anchors: list[UWBPosition],
                  gnss_anchors: list[GNSSPosition]) -> bool:
        """
        Calibrate the transformer using 3 anchor points.
        
        Args:
            uwb_anchors: List of 3 UWB anchor positions
            gnss_anchors: List of 3 GNSS anchor positions (corresponding to UWB anchors)
            
        Returns:
            True if calibration successful
        """
        if len(uwb_anchors) < 3 or len(gnss_anchors) < 3:
            return False
        
        self._uwb_anchors = uwb_anchors[:3]
        self._gnss_anchors = gnss_anchors[:3]
        self._ref_gnss = gnss_anchors[0]
        
        try:
            # Step 1: Convert GNSS anchors to ENU (relative to anchor0)
            enu_anchors = []
            for gnss in gnss_anchors[:3]:
                x, y, z = lla_to_ecef(gnss.lat, gnss.lon, gnss.alt)
                e, n, u = ecef_to_enu(x, y, z, 
                                       self._ref_gnss.lat, 
                                       self._ref_gnss.lon, 
                                       self._ref_gnss.alt)
                enu_anchors.append(np.array([e, n, u]))
            
            # Step 2: Build UWB coordinate basis
            p0_u = np.array([uwb_anchors[0].x, uwb_anchors[0].y, uwb_anchors[0].z])
            p1_u = np.array([uwb_anchors[1].x, uwb_anchors[1].y, uwb_anchors[1].z])
            p2_u = np.array([uwb_anchors[2].x, uwb_anchors[2].y, uwb_anchors[2].z])
            
            x_u = normalize(p1_u - p0_u)
            z_u = normalize(np.cross(p1_u - p0_u, p2_u - p0_u))
            y_u = np.cross(z_u, x_u)
            
            U = np.column_stack([x_u, y_u, z_u])
            
            # Step 3: Build ENU coordinate basis
            p0_e = enu_anchors[0]
            p1_e = enu_anchors[1]
            p2_e = enu_anchors[2]
            
            x_e = normalize(p1_e - p0_e)
            z_e = normalize(np.cross(p1_e - p0_e, p2_e - p0_e))
            y_e = np.cross(z_e, x_e)
            
            E = np.column_stack([x_e, y_e, z_e])
            
            # Step 4: Calculate rotation matrix R = E * U^T
            self._R = E @ U.T
            
            # Step 5: Calculate translation vector t = p0_E - R * p0_U
            self._t = p0_e - self._R @ p0_u
            
            # Verify rotation matrix properties
            det = np.linalg.det(self._R)
            if abs(det - 1.0) > 0.1:
                print(f"Warning: Rotation matrix determinant = {det}, should be ~1.0")
            
            self._is_calibrated = True
            return True
            
        except Exception as e:
            print(f"Calibration error: {e}")
            return False
    
    def transform(self, uwb_pos: UWBPosition) -> Optional[GNSSPosition]:
        """
        Transform a UWB position to GPS coordinates.
        
        Args:
            uwb_pos: UWB position in local coordinates
            
        Returns:
            GPS position, or None if not calibrated
        """
        if not self._is_calibrated:
            return None
        
        # Apply rigid body transformation: p_E = R * p_U + t
        p_u = np.array([uwb_pos.x, uwb_pos.y, uwb_pos.z])
        p_e = self._R @ p_u + self._t
        
        # Convert ENU to LLA
        lat, lon, alt = enu_to_lla(
            p_e[0], p_e[1], p_e[2],
            self._ref_gnss.lat,
            self._ref_gnss.lon,
            self._ref_gnss.alt
        )
        
        return GNSSPosition(lat=lat, lon=lon, alt=alt)
    
    def get_anchor_gnss(self, index: int) -> Optional[GNSSPosition]:
        """Get GNSS position of an anchor."""
        if index < len(self._gnss_anchors):
            return self._gnss_anchors[index]
        return None


def calculate_distance_to_line(point: Tuple[float, float], 
                                line_start: Tuple[float, float], 
                                line_end: Tuple[float, float]) -> float:
    """
    Calculate perpendicular distance from a point to a line segment (in 2D).
    
    Args:
        point: (x, y) coordinates of the point
        line_start: (x, y) coordinates of line start
        line_end: (x, y) coordinates of line end
        
    Returns:
        Signed perpendicular distance (positive = left side, negative = right side)
    """
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    # Line direction
    dx = x2 - x1
    dy = y2 - y1
    line_length = math.hypot(dx, dy)
    
    if line_length < 1e-10:
        return math.hypot(px - x1, py - y1)
    
    # Signed distance using cross product
    cross = (px - x1) * dy - (py - y1) * dx
    return cross / line_length


def check_line_crossing(prev_pos: Tuple[float, float], 
                        curr_pos: Tuple[float, float],
                        line_start: Tuple[float, float], 
                        line_end: Tuple[float, float]) -> bool:
    """
    Check if movement from prev_pos to curr_pos crosses the line.
    
    Returns:
        True if the line was crossed
    """
    prev_dist = calculate_distance_to_line(prev_pos, line_start, line_end)
    curr_dist = calculate_distance_to_line(curr_pos, line_start, line_end)
    
    # Crossed if signs are different (or one is zero)
    return prev_dist * curr_dist < 0 or (prev_dist != 0 and curr_dist == 0)


