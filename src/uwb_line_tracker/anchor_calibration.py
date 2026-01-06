"""
UWB Anchor Self-Calibration using MDS (Multi-Dimensional Scaling).
Based on the RTLSClient.cpp algorithm for anchor position estimation.

The algorithm:
1. Collect TWR distances between all anchor pairs
2. Build a distance matrix
3. Apply MDS to estimate relative positions
4. Apply angle rotation to align with reference coordinate system
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
import threading
import time


@dataclass
class AnchorPosition:
    """Estimated anchor position."""
    id: int
    x: float
    y: float
    z: float = 0.0


@dataclass
class CalibrationResult:
    """Result of anchor self-calibration."""
    success: bool
    anchors: List[AnchorPosition]
    error_message: str = ""
    timestamp: float = 0.0


class AnchorCalibrator:
    """
    MDS-based anchor self-calibration.
    
    Uses inter-anchor distance measurements to estimate anchor positions
    in a local coordinate system where:
    - Anchor 0 is at origin (0, 0)
    - Anchor 1 is on the positive X-axis
    - Anchor 2 is positioned based on distances to Anchor 0 and 1
    """
    
    def __init__(self, num_anchors: int = 3, n_dim: int = 2):
        self.num_anchors = num_anchors
        self.n_dim = n_dim  # 2D or 3D
        
        # Distance matrix: distance[i][j] = distance from anchor i to anchor j
        self._distance_matrix: Dict[Tuple[int, int], List[int]] = {}
        self._lock = threading.Lock()
        
        # Calibration state
        self._collecting = False
        self._collection_start_time = 0.0
        self._min_samples = 10  # Minimum samples per anchor pair
        
        # Result
        self._last_result: Optional[CalibrationResult] = None
    
    def start_collection(self):
        """Start collecting distance measurements."""
        with self._lock:
            self._distance_matrix.clear()
            self._collecting = True
            self._collection_start_time = time.time()
        print("[Calibrator] Started collecting measurements")
    
    def stop_collection(self):
        """Stop collecting and process measurements."""
        with self._lock:
            self._collecting = False
        print("[Calibrator] Stopped collecting measurements")
    
    def add_measurement(self, anchor_id: int, ranges: Dict[int, int]):
        """
        Add a distance measurement.
        
        Args:
            anchor_id: The anchor that made the measurement
            ranges: Dictionary of {other_anchor_id: distance_mm}
        """
        if not self._collecting:
            return
        
        with self._lock:
            for other_id, distance in ranges.items():
                if distance > 0 and other_id != anchor_id:
                    key = (min(anchor_id, other_id), max(anchor_id, other_id))
                    if key not in self._distance_matrix:
                        self._distance_matrix[key] = []
                    self._distance_matrix[key].append(distance)
    
    def get_collection_status(self) -> Dict[str, any]:
        """Get current collection status."""
        with self._lock:
            status = {
                "collecting": self._collecting,
                "duration": time.time() - self._collection_start_time if self._collecting else 0,
                "pairs": {}
            }
            for (i, j), distances in self._distance_matrix.items():
                status["pairs"][f"{i}-{j}"] = len(distances)
            return status
    
    def has_enough_data(self) -> bool:
        """Check if we have enough measurements to calibrate."""
        required_pairs = self.num_anchors * (self.num_anchors - 1) // 2
        with self._lock:
            if len(self._distance_matrix) < required_pairs:
                return False
            for distances in self._distance_matrix.values():
                if len(distances) < self._min_samples:
                    return False
        return True
    
    def calibrate(self) -> CalibrationResult:
        """
        Perform calibration using collected measurements.
        
        Returns:
            CalibrationResult with estimated anchor positions.
        """
        with self._lock:
            distance_data = dict(self._distance_matrix)
        
        # Check if we have enough data
        required_pairs = self.num_anchors * (self.num_anchors - 1) // 2
        if len(distance_data) < required_pairs:
            return CalibrationResult(
                success=False,
                anchors=[],
                error_message=f"Not enough anchor pairs. Got {len(distance_data)}, need {required_pairs}"
            )
        
        try:
            # Build average distance matrix
            n = self.num_anchors
            twr_distance = np.zeros((n, n))
            
            for (i, j), distances in distance_data.items():
                if i < n and j < n:
                    avg_dist = np.mean(distances) / 1000.0  # Convert mm to meters
                    twr_distance[i, j] = avg_dist
                    twr_distance[j, i] = avg_dist  # Symmetric
            
            # Apply MDS algorithm
            trans_coord = self._mds(twr_distance, n)
            
            # Apply angle rotation to align coordinate system
            est_coord = self._angle_rotation(trans_coord, n)
            
            # Create result
            anchors = []
            for i in range(n):
                anchors.append(AnchorPosition(
                    id=i,
                    x=float(est_coord[i, 0]),
                    y=float(est_coord[i, 1]),
                    z=0.0 if self.n_dim == 2 else float(est_coord[i, 2]) if est_coord.shape[1] > 2 else 0.0
                ))
            
            self._last_result = CalibrationResult(
                success=True,
                anchors=anchors,
                timestamp=time.time()
            )
            return self._last_result
            
        except Exception as e:
            return CalibrationResult(
                success=False,
                anchors=[],
                error_message=str(e)
            )
    
    def _mds(self, twr_distance: np.ndarray, n_nodes: int) -> np.ndarray:
        """
        Multi-Dimensional Scaling algorithm.
        Converts distance matrix to coordinate estimates.
        
        Based on RTLSClient::mds() from the C++ code.
        """
        # Square the distances
        dist_squared = np.square(twr_distance)
        
        # Centering operator: J = I - (1/n) * ones
        centering_operator = np.eye(n_nodes) - np.ones((n_nodes, n_nodes)) / n_nodes
        
        # Double centering: B = -0.5 * J * D^2 * J
        centered_dist_squared = -0.5 * centering_operator @ dist_squared @ centering_operator
        
        # SVD decomposition
        U, S, Vt = np.linalg.svd(centered_dist_squared)
        
        # Estimate geometry from SVD
        # Take first n_dim dimensions
        est_geom = np.zeros((n_nodes, self.n_dim))
        for i in range(min(self.n_dim, len(S))):
            est_geom[:, i] = U[:, i] * np.sqrt(max(S[i], 0))
        
        # Translate so that first node (view_node=0) is at origin
        view_node = 0
        trans_coord = est_geom - est_geom[view_node, :]
        
        return trans_coord
    
    def _angle_rotation(self, trans_coord: np.ndarray, n_nodes: int) -> np.ndarray:
        """
        Rotate coordinates to align with reference axes.
        
        Based on RTLSClient::angleRotation() from the C++ code.
        The result places:
        - A0 at origin
        - A1 on the positive X-axis
        - A2 with positive Y (above the X-axis)
        """
        if trans_coord.shape[1] < 2:
            return trans_coord
        
        # Calculate distances from origin
        trans_distance = np.sqrt(np.sum(trans_coord ** 2, axis=1))
        
        # Calculate current angle of each point
        current_angle = np.zeros(n_nodes)
        for i in range(n_nodes):
            if trans_distance[i] > 1e-6:
                current_angle[i] = np.arccos(np.clip(trans_coord[i, 0] / trans_distance[i], -1, 1))
                if trans_coord[i, 1] < 0:
                    current_angle[i] = -current_angle[i]
        
        # Rotate so that anchor 1 is on positive X-axis
        rotate_angle = current_angle[1] if n_nodes > 1 else 0
        
        # Build rotation matrix
        cos_r = np.cos(rotate_angle)
        sin_r = np.sin(rotate_angle)
        rotation_matrix = np.array([
            [cos_r, -sin_r],
            [sin_r, cos_r]
        ])
        
        # Apply rotation
        coord = trans_coord[:, :2] @ rotation_matrix
        
        # Ensure anchor 2 has positive Y (if it exists)
        if n_nodes > 2 and coord[2, 1] < 0:
            coord[:, 1] = -coord[:, 1]
        
        # If 3D, keep Z coordinate
        if trans_coord.shape[1] > 2:
            result = np.zeros((n_nodes, 3))
            result[:, :2] = coord
            result[:, 2] = trans_coord[:, 2]
            return result
        
        return coord
    
    def get_last_result(self) -> Optional[CalibrationResult]:
        """Get the last calibration result."""
        return self._last_result
    
    def set_num_anchors(self, num: int):
        """Set the number of anchors."""
        self.num_anchors = num
    
    def clear(self):
        """Clear all collected data."""
        with self._lock:
            self._distance_matrix.clear()
            self._collecting = False
            self._last_result = None


# Simplified calibration for 3 anchors using trilateration-style approach
def calibrate_3_anchors(d01: float, d02: float, d12: float) -> List[AnchorPosition]:
    """
    Calibrate 3 anchors using inter-anchor distances.
    
    Places anchors as:
    - A0 at origin (0, 0)
    - A1 at (d01, 0) - on positive X-axis
    - A2 at calculated position based on d02 and d12
    
    Args:
        d01: Distance between anchor 0 and 1 (in meters)
        d02: Distance between anchor 0 and 2 (in meters)
        d12: Distance between anchor 1 and 2 (in meters)
    
    Returns:
        List of 3 AnchorPosition objects
    """
    # A0 at origin
    a0 = AnchorPosition(id=0, x=0.0, y=0.0, z=0.0)
    
    # A1 on positive X-axis
    a1 = AnchorPosition(id=1, x=d01, y=0.0, z=0.0)
    
    # A2 using trilateration
    # From A0: x^2 + y^2 = d02^2
    # From A1: (x - d01)^2 + y^2 = d12^2
    # Solving: x = (d01^2 + d02^2 - d12^2) / (2 * d01)
    x2 = (d01**2 + d02**2 - d12**2) / (2 * d01) if d01 > 0 else 0
    
    # y^2 = d02^2 - x^2
    y2_squared = d02**2 - x2**2
    if y2_squared < 0:
        y2_squared = 0  # Handle numerical errors
    y2 = math.sqrt(y2_squared)
    
    a2 = AnchorPosition(id=2, x=x2, y=y2, z=0.0)
    
    return [a0, a1, a2]


# Test code
if __name__ == "__main__":
    # Test simple 3-anchor calibration
    print("Testing 3-anchor calibration:")
    # Example: equilateral triangle with 10m sides
    anchors = calibrate_3_anchors(10.0, 10.0, 10.0)
    for a in anchors:
        print(f"  Anchor {a.id}: ({a.x:.3f}, {a.y:.3f}, {a.z:.3f})")
    
    # Test MDS calibrator
    print("\nTesting MDS calibrator:")
    calibrator = AnchorCalibrator(num_anchors=3)
    calibrator.start_collection()
    
    # Simulate measurements (10m equilateral triangle)
    for _ in range(20):
        calibrator.add_measurement(0, {1: 10000, 2: 10000})  # 10m in mm
        calibrator.add_measurement(1, {0: 10000, 2: 10000})
        calibrator.add_measurement(2, {0: 10000, 1: 10000})
    
    calibrator.stop_collection()
    
    result = calibrator.calibrate()
    print(f"Success: {result.success}")
    if result.success:
        for a in result.anchors:
            print(f"  Anchor {a.id}: ({a.x:.3f}, {a.y:.3f}, {a.z:.3f})")
    else:
        print(f"Error: {result.error_message}")

