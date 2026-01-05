"""
UWB Trilateration Module.

This module provides trilateration functionality for UWB positioning.
It can use either:
1. The native DLL from uwbdemo (Trilateration.dll)
2. A pure Python fallback implementation

The DLL-based approach is preferred as it matches the hardware vendor's algorithm.
"""

import os
import sys
import math
from ctypes import Structure, c_double, c_int, byref, POINTER, cdll
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Vec3d:
    """3D vector/position."""
    x: float
    y: float
    z: float


class UWBMsg(Structure):
    """C structure for UWB position (matches DLL interface)."""
    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double)
    ]


class TrilaterationEngine:
    """
    Trilateration engine for UWB positioning.
    
    Supports up to 8 anchors. Uses the Trilateration.dll if available,
    otherwise falls back to pure Python implementation.
    """
    
    MAX_ANCHORS = 8
    
    def __init__(self, dll_path: Optional[str] = None):
        """
        Initialize trilateration engine.
        
        Args:
            dll_path: Path to Trilateration.dll. If None, tries default locations.
        """
        self.dll = None
        self.use_dll = False
        
        # Anchor positions (up to 8)
        self.anchors: List[Optional[Vec3d]] = [None] * self.MAX_ANCHORS
        
        # Try to load DLL
        self._load_dll(dll_path)
    
    def _load_dll(self, dll_path: Optional[str] = None):
        """Load the trilateration DLL."""
        paths_to_try = []
        
        if dll_path:
            paths_to_try.append(dll_path)
        
        # Try common locations
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        paths_to_try.extend([
            os.path.join(base_dir, "uwbdemo", "Trilateration.dll"),
            os.path.join(base_dir, "uwbdemo", "trilateration.dll"),
            os.path.join(base_dir, "uwbdemo", "vs2010 project", "x64", "Debug", "trilateration.dll"),
            "./Trilateration.dll",
            "./trilateration.dll",
        ])
        
        for path in paths_to_try:
            if os.path.exists(path):
                try:
                    self.dll = cdll.LoadLibrary(path)
                    self.use_dll = True
                    print(f"[Trilateration] Loaded DLL: {path}")
                    return
                except Exception as e:
                    print(f"[Trilateration] Failed to load {path}: {e}")
        
        print("[Trilateration] DLL not found, using Python fallback")
        self.use_dll = False
    
    def set_anchor(self, anchor_id: int, x: float, y: float, z: float):
        """
        Set anchor position.
        
        Args:
            anchor_id: Anchor ID (0-7)
            x, y, z: Anchor coordinates in meters
        """
        if 0 <= anchor_id < self.MAX_ANCHORS:
            self.anchors[anchor_id] = Vec3d(x, y, z)
    
    def set_anchors(self, anchors: List[dict]):
        """
        Set multiple anchor positions.
        
        Args:
            anchors: List of dicts with 'x', 'y', 'z' keys
        """
        for i, anchor in enumerate(anchors):
            if i < self.MAX_ANCHORS:
                self.set_anchor(i, anchor.get('x', 0), anchor.get('y', 0), anchor.get('z', 0))
    
    def get_location(self, distances: List[int]) -> Optional[Vec3d]:
        """
        Calculate tag location from distances to anchors.
        
        Args:
            distances: List of distances in millimeters (use -1 for invalid)
            
        Returns:
            Vec3d with calculated position, or None if calculation fails
        """
        if self.use_dll:
            return self._get_location_dll(distances)
        else:
            return self._get_location_python(distances)
    
    def _get_location_dll(self, distances: List[int]) -> Optional[Vec3d]:
        """Calculate location using DLL."""
        try:
            # Prepare anchor array
            anchor_array = (UWBMsg * self.MAX_ANCHORS)()
            for i in range(self.MAX_ANCHORS):
                if self.anchors[i]:
                    anchor_array[i].x = self.anchors[i].x
                    anchor_array[i].y = self.anchors[i].y
                    anchor_array[i].z = self.anchors[i].z
                else:
                    anchor_array[i].x = 0
                    anchor_array[i].y = 0
                    anchor_array[i].z = 0
            
            # Prepare distance array (use -1 for invalid)
            distance_array = (c_int * self.MAX_ANCHORS)()
            for i in range(self.MAX_ANCHORS):
                if i < len(distances):
                    distance_array[i] = distances[i]
                else:
                    distance_array[i] = -1
            
            # Result structure
            location = UWBMsg()
            
            # Call DLL
            result = self.dll.GetLocation(byref(location), anchor_array, distance_array)
            
            if result >= 0:
                return Vec3d(location.x, location.y, location.z)
            else:
                return None
                
        except Exception as e:
            print(f"[Trilateration] DLL error: {e}")
            return None
    
    def _get_location_python(self, distances: List[int]) -> Optional[Vec3d]:
        """
        Calculate location using pure Python trilateration.
        
        This is a fallback implementation using least squares optimization.
        """
        # Collect valid anchors and distances
        valid_anchors = []
        valid_distances = []
        
        for i, d in enumerate(distances):
            if d > 0 and i < self.MAX_ANCHORS and self.anchors[i]:
                valid_anchors.append(self.anchors[i])
                valid_distances.append(d / 1000.0)  # Convert mm to meters
        
        if len(valid_anchors) < 3:
            return None
        
        try:
            # Use first 3 anchors for basic trilateration
            return self._trilaterate_3spheres(
                valid_anchors[:3],
                valid_distances[:3]
            )
        except Exception as e:
            print(f"[Trilateration] Python fallback error: {e}")
            return None
    
    def _trilaterate_3spheres(self, anchors: List[Vec3d], distances: List[float]) -> Optional[Vec3d]:
        """
        3-sphere trilateration algorithm.
        
        Based on the mathematical solution for finding the intersection
        of three spheres.
        """
        if len(anchors) < 3 or len(distances) < 3:
            return None
        
        # Anchor positions
        p1 = (anchors[0].x, anchors[0].y, anchors[0].z)
        p2 = (anchors[1].x, anchors[1].y, anchors[1].z)
        p3 = (anchors[2].x, anchors[2].y, anchors[2].z)
        
        # Distances (radii)
        r1, r2, r3 = distances[0], distances[1], distances[2]
        
        # Vector operations
        def vsub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        def vadd(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
        def vmul(v, s): return (v[0]*s, v[1]*s, v[2]*s)
        def vdiv(v, s): return (v[0]/s, v[1]/s, v[2]/s) if s != 0 else (0, 0, 0)
        def vnorm(v): return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        def vdot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        def vcross(a, b): return (
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        )
        
        # ex = (P2 - P1) / |P2 - P1|
        temp = vsub(p2, p1)
        d = vnorm(temp)
        if d < 1e-10:
            return None
        ex = vdiv(temp, d)
        
        # i = ex . (P3 - P1)
        temp = vsub(p3, p1)
        i = vdot(ex, temp)
        
        # ey = (P3 - P1 - i*ex) / |P3 - P1 - i*ex|
        temp2 = vmul(ex, i)
        temp = vsub(temp, temp2)
        j = vnorm(temp)
        if j < 1e-10:
            return None
        ey = vdiv(temp, j)
        
        # ez = ex x ey
        ez = vcross(ex, ey)
        
        # x = (r1^2 - r2^2 + d^2) / (2*d)
        x = (r1*r1 - r2*r2 + d*d) / (2*d)
        
        # y = (r1^2 - r3^2 + i^2 + j^2) / (2*j) - (i/j)*x
        y = (r1*r1 - r3*r3 + i*i + j*j) / (2*j) - (i/j)*x
        
        # z = sqrt(r1^2 - x^2 - y^2)
        z_sq = r1*r1 - x*x - y*y
        if z_sq < 0:
            z = 0  # No valid intersection, use plane solution
        else:
            z = math.sqrt(z_sq)
        
        # Final position = P1 + x*ex + y*ey + z*ez
        # Take the solution with positive z (above the plane)
        result = vadd(p1, vmul(ex, x))
        result = vadd(result, vmul(ey, y))
        result = vadd(result, vmul(ez, z))
        
        return Vec3d(result[0], result[1], result[2])


class UWBPositionCalculator:
    """
    High-level interface for UWB position calculation.
    
    Integrates with the UWB hardware data format.
    """
    
    def __init__(self, dll_path: Optional[str] = None):
        self.engine = TrilaterationEngine(dll_path)
        self._last_position: Optional[Vec3d] = None
    
    def configure_anchors(self, anchors: List[dict]):
        """
        Configure anchor positions.
        
        Args:
            anchors: List of anchor dicts with 'x', 'y', 'z' keys
        """
        self.engine.set_anchors(anchors)
    
    def calculate_position(self, range_data: dict) -> Optional[Tuple[float, float, float]]:
        """
        Calculate position from range data.
        
        Args:
            range_data: Dict mapping anchor IDs to distances in mm
            
        Returns:
            Tuple (x, y, z) or None if calculation fails
        """
        # Convert to distance array
        distances = [-1] * TrilaterationEngine.MAX_ANCHORS
        for anchor_id, dist in range_data.items():
            if 0 <= anchor_id < TrilaterationEngine.MAX_ANCHORS:
                distances[anchor_id] = int(dist)
        
        result = self.engine.get_location(distances)
        if result:
            self._last_position = result
            return (result.x, result.y, result.z)
        return None
    
    @property
    def last_position(self) -> Optional[Tuple[float, float, float]]:
        """Get last calculated position."""
        if self._last_position:
            return (self._last_position.x, self._last_position.y, self._last_position.z)
        return None
    
    @property
    def is_dll_loaded(self) -> bool:
        """Check if DLL is loaded."""
        return self.engine.use_dll


def test_trilateration():
    """Test trilateration with known positions."""
    print("Testing trilateration...")
    
    engine = TrilaterationEngine()
    
    # Set up anchors (same as uwbdemo)
    engine.set_anchor(0, 0, 0, 2)
    engine.set_anchor(1, 0, 10, 2)
    engine.set_anchor(2, 10, 10, 2)
    engine.set_anchor(3, 10, 0, 2)
    
    # Test distances (from uwbdemo)
    distances = [7433, 3905, 8078, -1, -1, -1, -1, -1]
    
    result = engine.get_location(distances)
    
    if result:
        print(f"Calculated position: x={result.x:.3f}, y={result.y:.3f}, z={result.z:.3f}")
    else:
        print("Trilateration failed")
    
    return result


if __name__ == "__main__":
    test_trilateration()

