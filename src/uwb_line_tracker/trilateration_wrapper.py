"""
Wrapper for trilateration.dll to calculate UWB tag position.
Uses the DLL from uwbdemo folder.
"""

from __future__ import annotations

import os
import ctypes
from ctypes import Structure, c_double, c_int, POINTER, byref
from dataclasses import dataclass
from typing import Optional, List, Tuple

# DLL path
DLL_NAME = "trilateration.dll"


class Vec3D(Structure):
    """3D vector structure matching the DLL's vec3d struct."""
    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double)
    ]
    
    def __repr__(self):
        return f"Vec3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"


@dataclass
class Position3D:
    """Python-friendly 3D position."""
    x: float
    y: float
    z: float
    
    @classmethod
    def from_vec3d(cls, vec: Vec3D) -> 'Position3D':
        return cls(x=vec.x, y=vec.y, z=vec.z)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


class TrilaterationEngine:
    """
    Engine for calculating tag position using trilateration.
    Wraps the trilateration.dll GetLocation function.
    
    The DLL supports 3-8 anchors and uses trilateration algorithm
    to calculate the tag position based on distances.
    """
    
    MAX_ANCHORS = 8
    
    def __init__(self, dll_path: Optional[str] = None):
        """
        Initialize the trilateration engine.
        
        Args:
            dll_path: Path to trilateration.dll. If None, searches in common locations.
        """
        self._dll = None
        self._dll_path = dll_path
        self._anchors: List[Vec3D] = []
        self._anchor_array = (Vec3D * self.MAX_ANCHORS)()
        self._distance_array = (c_int * self.MAX_ANCHORS)()
        self._location = Vec3D()
        
        # Initialize distance array with -1 (invalid)
        for i in range(self.MAX_ANCHORS):
            self._distance_array[i] = -1
        
        self._load_dll()
    
    def _find_dll(self) -> Optional[str]:
        """Find the trilateration.dll in common locations."""
        # Get the directory containing this module
        module_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(module_dir))
        
        # Search paths
        search_paths = [
            # Provided path
            self._dll_path,
            # uwbdemo folder in project root
            os.path.join(project_root, "uwbdemo", "trilateration.dll"),
            os.path.join(project_root, "uwbdemo", "Trilateration.dll"),
            # x64 Debug build
            os.path.join(project_root, "uwbdemo", "vs2010 project", "trilateration", "x64", "Debug", "trilateration.dll"),
            # Current directory
            os.path.join(os.getcwd(), "trilateration.dll"),
            os.path.join(os.getcwd(), "uwbdemo", "trilateration.dll"),
            # Module directory
            os.path.join(module_dir, "trilateration.dll"),
        ]
        
        for path in search_paths:
            if path and os.path.exists(path):
                return path
        
        return None
    
    def _load_dll(self) -> bool:
        """Load the trilateration DLL."""
        dll_path = self._find_dll()
        
        if not dll_path:
            print(f"[Trilateration] DLL not found. Searched paths:")
            print(f"  - uwbdemo/trilateration.dll")
            print(f"  - uwbdemo/vs2010 project/trilateration/x64/Debug/trilateration.dll")
            return False
        
        try:
            # Load DLL
            self._dll = ctypes.cdll.LoadLibrary(dll_path)
            
            # Set up GetLocation function signature
            # int GetLocation(vec3d *best_solution, vec3d* anchorArray, int *distanceArray)
            self._dll.GetLocation.argtypes = [
                POINTER(Vec3D),      # best_solution
                POINTER(Vec3D),      # anchorArray
                POINTER(c_int)       # distanceArray
            ]
            self._dll.GetLocation.restype = c_int
            
            print(f"[Trilateration] Loaded DLL: {dll_path}")
            return True
            
        except Exception as e:
            print(f"[Trilateration] Failed to load DLL: {e}")
            self._dll = None
            return False
    
    @property
    def is_loaded(self) -> bool:
        """Check if DLL is loaded."""
        return self._dll is not None
    
    def set_anchors(self, anchors: List[Tuple[float, float, float]]):
        """
        Set anchor positions.
        
        Args:
            anchors: List of (x, y, z) tuples for each anchor.
                     Can have 3-8 anchors.
        """
        if len(anchors) > self.MAX_ANCHORS:
            raise ValueError(f"Maximum {self.MAX_ANCHORS} anchors supported")
        
        self._anchors = []
        for i, (x, y, z) in enumerate(anchors):
            self._anchor_array[i].x = x
            self._anchor_array[i].y = y
            self._anchor_array[i].z = z
            self._anchors.append(Vec3D(x, y, z))
        
        # Fill remaining with zeros
        for i in range(len(anchors), self.MAX_ANCHORS):
            self._anchor_array[i].x = 0
            self._anchor_array[i].y = 0
            self._anchor_array[i].z = 0
        
        print(f"[Trilateration] Set {len(anchors)} anchors")
    
    def calculate_position(self, distances: List[int]) -> Optional[Position3D]:
        """
        Calculate tag position from distances to anchors.
        
        Args:
            distances: List of distances in millimeters.
                       Use -1 for invalid/missing distances.
                       At least 3 valid distances required.
        
        Returns:
            Position3D if successful, None otherwise.
        """
        if not self._dll:
            print("[Trilateration] DLL not loaded")
            return None
        
        if len(distances) > self.MAX_ANCHORS:
            distances = distances[:self.MAX_ANCHORS]
        
        # Count valid distances
        valid_count = sum(1 for d in distances if d > 0)
        if valid_count < 3:
            # print("[Trilateration] Need at least 3 valid distances")
            return None
        
        # Set distances
        for i in range(self.MAX_ANCHORS):
            if i < len(distances):
                self._distance_array[i] = distances[i]
            else:
                self._distance_array[i] = -1
        
        # Call DLL function
        try:
            result = self._dll.GetLocation(
                byref(self._location),
                self._anchor_array,
                self._distance_array
            )
            
            if result >= 0:
                return Position3D.from_vec3d(self._location)
            else:
                # print(f"[Trilateration] Calculation failed with result: {result}")
                return None
                
        except Exception as e:
            print(f"[Trilateration] Error: {e}")
            return None
    
    def calculate_position_meters(self, distances_mm: List[int]) -> Optional[Position3D]:
        """
        Calculate tag position from distances (in mm) and return position in meters.
        
        This is the same as calculate_position, as the DLL internally converts
        mm to meters (divides by 1000).
        """
        return self.calculate_position(distances_mm)


# Global engine instance
_engine: Optional[TrilaterationEngine] = None


def get_trilateration_engine() -> TrilaterationEngine:
    """Get or create the global trilateration engine."""
    global _engine
    if _engine is None:
        _engine = TrilaterationEngine()
    return _engine


def calculate_tag_position(
    anchor_positions: List[Tuple[float, float, float]],
    distances_mm: List[int]
) -> Optional[Position3D]:
    """
    Convenience function to calculate tag position.
    
    Args:
        anchor_positions: List of (x, y, z) anchor positions in meters.
        distances_mm: List of distances in millimeters (-1 for invalid).
    
    Returns:
        Position3D in meters if successful, None otherwise.
    """
    engine = get_trilateration_engine()
    if not engine.is_loaded:
        return None
    
    engine.set_anchors(anchor_positions)
    return engine.calculate_position(distances_mm)


# Test function
def test_trilateration():
    """Test the trilateration engine."""
    engine = TrilaterationEngine()
    
    if not engine.is_loaded:
        print("DLL not loaded, cannot test")
        return
    
    # Set up anchors (same as UWB_demo.py)
    anchors = [
        (0, 0, 2),
        (0, 10, 2),
        (10, 10, 2),
        (10, 0, 2),
        (20, 0, 2),
        (20, 10, 2),
        (30, 0, 2),
        (30, 10, 2),
    ]
    engine.set_anchors(anchors)
    
    # Test distances
    distances = [7433, 3905, 8078, -1, 18848, -1, -1, -1]
    
    result = engine.calculate_position(distances)
    if result:
        print(f"Calculated position: x={result.x:.3f}, y={result.y:.3f}, z={result.z:.3f}")
    else:
        print("Failed to calculate position")


if __name__ == "__main__":
    test_trilateration()

