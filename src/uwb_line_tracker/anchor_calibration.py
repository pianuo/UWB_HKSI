"""
UWB Anchor Self-Calibration using geometric trilateration.
Based on the RTLSClient.cpp algorithm for anchor position estimation.

For 3 anchors, uses simple geometric calculation:
- A0 at origin (0, 0)
- A1 on positive X-axis
- A2 positioned based on distances to A0 and A1
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
    Anchor self-calibration using geometric calculation.
    
    For 3 anchors, positions are calculated as:
    - Anchor 0 at origin (0, 0, z0)
    - Anchor 1 on positive X-axis (d01, 0, z1)
    - Anchor 2 positioned based on distances (x2, y2, z2)
    """
    
    def __init__(self, num_anchors: int = 3, n_dim: int = 2):
        self.num_anchors = num_anchors
        self.n_dim = n_dim  # 2D or 3D
        
        # Store distances for each anchor pair: (i, j) -> list of distances in mm
        self._distance_matrix: Dict[Tuple[int, int], List[int]] = {}
        self._lock = threading.Lock()
        
        # Calibration state
        self._collecting = False
        self._collection_start_time = 0.0
        self._min_samples = 5  # Minimum samples per anchor pair
        
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
            anchor_id: The anchor that made the measurement (source)
            ranges: Dictionary of {target_anchor_id: distance_mm}
        """
        if not self._collecting:
            return
        
        with self._lock:
            for target_id, distance in ranges.items():
                # Only consider valid distances and anchors within our range
                if distance > 0 and target_id != anchor_id:
                    # Debug: Print first measurement for each pair
                    key = (min(anchor_id, target_id), max(anchor_id, target_id))
                    if key not in self._distance_matrix:
                        self._distance_matrix[key] = []
                        print(f"[Calibrator] First measurement {key}: {distance} mm ({distance/1000:.3f} m)")
                    self._distance_matrix[key].append(distance)
    
    def get_collection_status(self) -> Dict[str, any]:
        """Get current collection status."""
        with self._lock:
            # Count only pairs within num_anchors
            valid_pairs = {}
            has_d01 = False
            has_d02 = False
            has_d12 = False
            
            for (i, j), distances in self._distance_matrix.items():
                if i < self.num_anchors and j < self.num_anchors:
                    valid_pairs[f"{i}-{j}"] = len(distances)
                    if (i, j) == (0, 1) and len(distances) >= self._min_samples:
                        has_d01 = True
                    if (i, j) == (0, 2) and len(distances) >= self._min_samples:
                        has_d02 = True
                    if (i, j) == (1, 2) and len(distances) >= self._min_samples:
                        has_d12 = True
            
            # For 3 anchors, minimum is 2 pairs (d01 and d02)
            if self.num_anchors == 3:
                min_required = 2  # d01 and d02 are required
                ready = has_d01 and has_d02
            else:
                min_required = self.num_anchors - 1
                ready = len(valid_pairs) >= min_required
            
            status = {
                "collecting": self._collecting,
                "duration": time.time() - self._collection_start_time if self._collecting else 0,
                "pairs": valid_pairs,
                "valid_count": len(valid_pairs),
                "required_count": min_required,
                "ready": ready,
                "has_d01": has_d01,
                "has_d02": has_d02,
                "has_d12": has_d12,
            }
            return status
    
    def has_enough_data(self) -> bool:
        """Check if we have enough measurements to calibrate."""
        with self._lock:
            valid_count = 0
            has_d01 = False
            has_d02 = False
            
            for (i, j), distances in self._distance_matrix.items():
                if i < self.num_anchors and j < self.num_anchors:
                    if len(distances) >= self._min_samples:
                        valid_count += 1
                        if (i, j) == (0, 1):
                            has_d01 = True
                        if (i, j) == (0, 2):
                            has_d02 = True
            
            # For 3 anchors: minimum need d01 and d02 (can estimate d12)
            # Ideally want all 3 pairs, but can work with 2 if from anchor 0
            if self.num_anchors == 3:
                return has_d01 and has_d02  # At least need these two
            else:
                # For more anchors, need at least num_anchors-1 pairs
                return valid_count >= self.num_anchors - 1
    
    def calibrate(self) -> CalibrationResult:
        """
        Perform calibration using collected measurements.
        Uses simple geometric trilateration for 3 anchors.
        
        Returns:
            CalibrationResult with estimated anchor positions.
        """
        with self._lock:
            distance_data = dict(self._distance_matrix)
        
        try:
            # Extract distances between anchor pairs (only first num_anchors)
            n = self.num_anchors
            avg_distances = {}
            
            for (i, j), distances in distance_data.items():
                if i < n and j < n and len(distances) > 0:
                    # Convert mm to meters and calculate average
                    avg_dist = np.mean(distances) / 1000.0
                    avg_distances[(i, j)] = avg_dist
                    print(f"[Calibrator] Distance {i}-{j}: {avg_dist:.3f} m (from {len(distances)} samples)")
            
            # For 3 anchors, use simple geometric calculation
            if n == 3:
                # Get distances
                d01 = avg_distances.get((0, 1), 0)
                d02 = avg_distances.get((0, 2), 0)
                d12 = avg_distances.get((1, 2), 0)
                
                # Check minimum required distances
                if d01 <= 0 or d02 <= 0:
                    return CalibrationResult(
                        success=False,
                        anchors=[],
                        error_message=f"Missing required distances: d01={d01:.3f}, d02={d02:.3f}"
                    )
                
                # If d12 is missing, estimate it
                # This happens when only anchor 0 reports distances
                if d12 <= 0:
                    # Estimate d12: assume roughly similar to average of d01 and d02
                    # This gives a reasonable approximation for typical setups
                    d12 = (d01 + d02) / 2
                    print(f"[Calibrator] Warning: d12 missing, estimated as {d12:.3f} m (avg of d01 and d02)")
                    print(f"[Calibrator] For accurate results, ensure all anchors report inter-anchor distances")
                
                # Calculate positions using geometric method
                anchors = calibrate_3_anchors(d01, d02, d12)
                
                self._last_result = CalibrationResult(
                    success=True,
                    anchors=anchors,
                    timestamp=time.time()
                )
                return self._last_result
            else:
                # For more anchors, use MDS
                return self._calibrate_mds(avg_distances, n)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            return CalibrationResult(
                success=False,
                anchors=[],
                error_message=str(e)
            )
    
    def _calibrate_mds(self, avg_distances: Dict[Tuple[int, int], float], n: int) -> CalibrationResult:
        """Calibrate using MDS for more than 3 anchors."""
        # Build distance matrix
        twr_distance = np.zeros((n, n))
        for (i, j), dist in avg_distances.items():
            twr_distance[i, j] = dist
            twr_distance[j, i] = dist
        
        # Classical MDS
        # 1. Square the distances
        D_sq = np.square(twr_distance)
        
        # 2. Double centering: B = -0.5 * J * D^2 * J
        J = np.eye(n) - np.ones((n, n)) / n
        B = -0.5 * J @ D_sq @ J
        
        # 3. Eigendecomposition
        eigenvalues, eigenvectors = np.linalg.eigh(B)
        
        # Sort by eigenvalues (descending)
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # 4. Take first 2 dimensions
        coords = np.zeros((n, 2))
        for i in range(2):
            if eigenvalues[i] > 0:
                coords[:, i] = eigenvectors[:, i] * np.sqrt(eigenvalues[i])
        
        # 5. Translate so A0 is at origin
        coords = coords - coords[0, :]
        
        # 6. Rotate so A1 is on positive X-axis
        if n > 1:
            angle = np.arctan2(coords[1, 1], coords[1, 0])
            cos_a, sin_a = np.cos(-angle), np.sin(-angle)
            R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
            coords = coords @ R.T
        
        # 7. Ensure A2 has positive Y
        if n > 2 and coords[2, 1] < 0:
            coords[:, 1] = -coords[:, 1]
        
        anchors = [
            AnchorPosition(id=i, x=float(coords[i, 0]), y=float(coords[i, 1]), z=0.0)
            for i in range(n)
        ]
        
        return CalibrationResult(success=True, anchors=anchors, timestamp=time.time())
    
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
    if d01 > 0:
        x2 = (d01**2 + d02**2 - d12**2) / (2 * d01)
    else:
        x2 = 0
    
    # y^2 = d02^2 - x^2
    y2_squared = d02**2 - x2**2
    if y2_squared < 0:
        y2_squared = 0  # Handle numerical errors
    y2 = math.sqrt(y2_squared)
    
    a2 = AnchorPosition(id=2, x=x2, y=y2, z=0.0)
    
    print(f"[Calibrator] 3-anchor result: A0=(0,0), A1=({d01:.3f},0), A2=({x2:.3f},{y2:.3f})")
    
    return [a0, a1, a2]


# Test code
if __name__ == "__main__":
    print("Testing 3-anchor calibration:")
    
    # Test with realistic distances (similar to user's setup: ~3m between anchors)
    # From user's reference: A0=(0,0,2.22), A1=(3.02,0,2.15), A2=(1.47,2.56,2.14)
    # d01 = 3.02, d02 = sqrt(1.47^2 + 2.56^2) = 2.95, d12 = sqrt((3.02-1.47)^2 + 2.56^2) = 2.99
    
    d01 = 3.02  # A0 to A1
    d02 = 2.95  # A0 to A2
    d12 = 2.99  # A1 to A2
    
    anchors = calibrate_3_anchors(d01, d02, d12)
    print("Expected: A0=(0,0), A1=(3.02,0), A2=(1.47,2.56)")
    print("Got:")
    for a in anchors:
        print(f"  Anchor {a.id}: ({a.x:.3f}, {a.y:.3f}, {a.z:.3f})")
