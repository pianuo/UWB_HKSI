"""
UWB Trilateration using the Trilateration DLL.
Wrapper for calling the Decawave trilateration algorithm.
"""

from __future__ import annotations

import os
import ctypes
from ctypes import c_double, c_int, Structure, POINTER, byref
from dataclasses import dataclass
from typing import List, Optional, Tuple
import math


@dataclass
class Position3D:
    """3D position result."""
    x: float
    y: float
    z: float
    valid: bool = True


class Vec3D(Structure):
    """C structure for 3D vector (matches vec3d in DLL)."""
    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double)
    ]


class TrilaterationEngine:
    """
    Trilateration engine using the Decawave DLL.
    
    Calculates tag position from anchor positions and tag-to-anchor distances.
    Supports 3-8 anchors.
    """
    
    MAX_ANCHORS = 8
    
    def __init__(self, dll_path: Optional[str] = None):
        self._dll = None
        self._dll_path = dll_path
        self._anchors: List[Position3D] = []
        self._num_anchors = 0
        
        # Try to load DLL
        self._load_dll()
    
    def _load_dll(self):
        """Load the trilateration DLL."""
        # Try multiple possible DLL paths
        possible_paths = []
        
        if self._dll_path:
            possible_paths.append(self._dll_path)
        
        # Current module directory
        module_dir = os.path.dirname(os.path.abspath(__file__))
        possible_paths.extend([
            os.path.join(module_dir, "Trilateration.dll"),
            os.path.join(module_dir, "trilateration.dll"),
        ])
        
        # uwbdemo directory
        project_root = os.path.dirname(os.path.dirname(module_dir))
        possible_paths.extend([
            os.path.join(project_root, "uwbdemo", "trilateration.dll"),
            os.path.join(project_root, "uwbdemo", "Trilateration.dll"),
            os.path.join(project_root, "uwbdemo", "vs2010 project", "x64", "Debug", "trilateration.dll"),
        ])
        
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    self._dll = ctypes.cdll.LoadLibrary(path)
                    self._setup_dll_functions()
                    print(f"[Trilateration] Loaded DLL from: {path}")
                    return
                except Exception as e:
                    print(f"[Trilateration] Failed to load DLL from {path}: {e}")
        
        print("[Trilateration] Warning: DLL not found, will use Python fallback")
    
    def _setup_dll_functions(self):
        """Setup DLL function signatures."""
        if not self._dll:
            return
        
        try:
            # GetLocation(vec3d *best_solution, vec3d* anchorArray, int *distanceArray)
            self._dll.GetLocation.argtypes = [
                POINTER(Vec3D),  # best_solution
                POINTER(Vec3D * self.MAX_ANCHORS),  # anchorArray
                POINTER(c_int * self.MAX_ANCHORS)   # distanceArray
            ]
            self._dll.GetLocation.restype = c_int
        except Exception as e:
            print(f"[Trilateration] Failed to setup DLL functions: {e}")
            self._dll = None
    
    def set_anchors(self, anchors: List[Tuple[float, float, float]]):
        """
        Set anchor positions.
        
        Args:
            anchors: List of (x, y, z) tuples for each anchor position (in meters)
        """
        self._anchors = [Position3D(x=a[0], y=a[1], z=a[2]) for a in anchors]
        self._num_anchors = len(self._anchors)
        print(f"[Trilateration] Set {self._num_anchors} anchors")
    
    def set_anchor_positions(self, positions: List[Position3D]):
        """Set anchor positions from Position3D objects."""
        self._anchors = positions
        self._num_anchors = len(self._anchors)
    
    def calculate_position(self, distances: List[int]) -> Optional[Position3D]:
        """
        Calculate tag position from distances to anchors.
        
        Args:
            distances: List of distances in mm (-1 for invalid/unavailable)
        
        Returns:
            Position3D with calculated position, or None if calculation fails
        """
        if self._num_anchors < 3:
            return None
        
        # Ensure we have enough distance values
        while len(distances) < self.MAX_ANCHORS:
            distances.append(-1)
        
        # Try DLL first
        if self._dll:
            return self._calculate_with_dll(distances)
        
        # Fallback to Python implementation
        return self._calculate_python(distances)
    
    def _calculate_with_dll(self, distances: List[int]) -> Optional[Position3D]:
        """Calculate position using the DLL."""
        try:
            # Create anchor array
            anchor_array = (Vec3D * self.MAX_ANCHORS)()
            for i, anchor in enumerate(self._anchors):
                anchor_array[i].x = anchor.x
                anchor_array[i].y = anchor.y
                anchor_array[i].z = anchor.z
            # Fill remaining with zeros
            for i in range(len(self._anchors), self.MAX_ANCHORS):
                anchor_array[i].x = 0
                anchor_array[i].y = 0
                anchor_array[i].z = 0
            
            # Create distance array
            distance_array = (c_int * self.MAX_ANCHORS)(*distances[:self.MAX_ANCHORS])
            
            # Create result structure
            result = Vec3D()
            
            # Call DLL
            ret = self._dll.GetLocation(byref(result), byref(anchor_array), byref(distance_array))
            
            if ret >= 0:
                return Position3D(x=result.x, y=result.y, z=result.z)
            else:
                return None
                
        except Exception as e:
            print(f"[Trilateration] DLL error: {e}")
            return None
    
    def _calculate_python(self, distances: List[int]) -> Optional[Position3D]:
        """
        Python fallback implementation of trilateration.
        Uses a simplified least-squares approach.
        """
        # Filter valid anchors and distances
        valid_pairs = []
        for i, (anchor, dist) in enumerate(zip(self._anchors, distances)):
            if dist > 0:
                valid_pairs.append((anchor, dist / 1000.0))  # Convert mm to meters
        
        if len(valid_pairs) < 3:
            return None
        
        # Use first 3-4 anchors for trilateration
        try:
            if len(valid_pairs) == 3:
                return self._trilaterate_3(valid_pairs)
            else:
                return self._trilaterate_4(valid_pairs[:4])
        except Exception as e:
            print(f"[Trilateration] Python calculation error: {e}")
            return None
    
    def _trilaterate_3(self, pairs: List[Tuple[Position3D, float]]) -> Position3D:
        """Trilateration with 3 spheres."""
        p1, r1 = pairs[0]
        p2, r2 = pairs[1]
        p3, r3 = pairs[2]
        
        # Transform to coordinate system where p1 is at origin
        # and p2 is on x-axis
        
        # ex: unit vector from p1 to p2
        d = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
        if d < 0.001:
            return Position3D(x=p1.x, y=p1.y, z=p1.z, valid=False)
        
        ex = ((p2.x - p1.x) / d, (p2.y - p1.y) / d, (p2.z - p1.z) / d)
        
        # i: projection of p3-p1 onto ex
        p3_p1 = (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)
        i = ex[0] * p3_p1[0] + ex[1] * p3_p1[1] + ex[2] * p3_p1[2]
        
        # ey: perpendicular to ex in the plane of p1, p2, p3
        ey_unnorm = (p3_p1[0] - i * ex[0], p3_p1[1] - i * ex[1], p3_p1[2] - i * ex[2])
        j = math.sqrt(ey_unnorm[0]**2 + ey_unnorm[1]**2 + ey_unnorm[2]**2)
        
        if j < 0.001:
            # Collinear points
            return Position3D(x=p1.x, y=p1.y, z=p1.z, valid=False)
        
        ey = (ey_unnorm[0] / j, ey_unnorm[1] / j, ey_unnorm[2] / j)
        
        # Calculate x, y in this coordinate system
        x = (r1**2 - r2**2 + d**2) / (2 * d)
        y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i * x) / j
        
        # z^2 = r1^2 - x^2 - y^2
        z_sq = r1**2 - x**2 - y**2
        if z_sq < 0:
            z_sq = 0  # Handle numerical errors
        z = math.sqrt(z_sq)
        
        # Transform back to original coordinates
        result_x = p1.x + x * ex[0] + y * ey[0]
        result_y = p1.y + x * ex[1] + y * ey[1]
        result_z = p1.z + x * ex[2] + y * ey[2]
        
        # Take the solution with smaller z (assuming tag is below anchors)
        return Position3D(x=result_x, y=result_y, z=result_z)
    
    def _trilaterate_4(self, pairs: List[Tuple[Position3D, float]]) -> Position3D:
        """Trilateration with 4 spheres (uses least squares)."""
        # First get result from 3 spheres
        result = self._trilaterate_3(pairs[:3])
        
        if not result.valid:
            return result
        
        # Could refine with 4th sphere using least squares
        # For now, just return the 3-sphere result
        return result
    
    @property
    def is_dll_loaded(self) -> bool:
        """Check if DLL is loaded."""
        return self._dll is not None
    
    @property
    def num_anchors(self) -> int:
        """Get number of configured anchors."""
        return self._num_anchors


# Global instance for easy access
_engine: Optional[TrilaterationEngine] = None


def get_engine() -> TrilaterationEngine:
    """Get or create the global trilateration engine."""
    global _engine
    if _engine is None:
        _engine = TrilaterationEngine()
    return _engine


def calculate_tag_position(
    anchors: List[Tuple[float, float, float]],
    distances: List[int]
) -> Optional[Position3D]:
    """
    Convenience function to calculate tag position.
    
    Args:
        anchors: List of anchor positions as (x, y, z) tuples
        distances: List of distances in mm (-1 for invalid)
    
    Returns:
        Position3D or None
    """
    engine = get_engine()
    engine.set_anchors(anchors)
    return engine.calculate_position(distances)


# Test code
if __name__ == "__main__":
    print("Testing trilateration engine...")
    
    engine = TrilaterationEngine()
    print(f"DLL loaded: {engine.is_dll_loaded}")
    
    # Setup anchors (same as UWB_demo.py)
    engine.set_anchors([
        (0, 0, 2),    # A0
        (0, 10, 2),   # A1
        (10, 10, 2),  # A2
        (10, 0, 2),   # A3
    ])
    
    # Test distances (in mm)
    distances = [7433, 3905, 8078, -1, -1, -1, -1, -1]
    
    result = engine.calculate_position(distances)
    if result:
        print(f"Result: x={result.x:.3f}, y={result.y:.3f}, z={result.z:.3f}")
    else:
        print("Calculation failed")

