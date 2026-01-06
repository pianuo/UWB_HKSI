"""
Anchor self-calibration module using MDS (Multi-Dimensional Scaling).
Based on the C++ implementation from RTLSClient.cpp.

This module calculates anchor positions from inter-anchor distance measurements.
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any


@dataclass
class AnchorPosition:
    """Anchor position result."""
    id: int
    x: float
    y: float
    z: float = 0.0
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


class AnchorCalibration:
    """
    Anchor self-calibration using MDS algorithm.
    
    Algorithm:
    1. Collect inter-anchor distance measurements
    2. Build distance matrix
    3. Apply MDS to estimate 2D coordinates
    4. Apply rotation to align with reference frame (A0 at origin, A1 on x-axis)
    """
    
    def __init__(self, num_anchors: int = 3, n_dim: int = 2):
        """
        Initialize calibration.
        
        Args:
            num_anchors: Number of anchors (3-8 supported)
            n_dim: Number of dimensions (2 for 2D positioning)
        """
        self.num_anchors = num_anchors
        self.n_dim = n_dim
        self.view_node = 0  # Reference node (A0)
        
        # Distance matrix storage
        self._distance_matrix: Optional[np.ndarray] = None
        self._calibrated_positions: Optional[List[AnchorPosition]] = None
        
        # Raw measurement buffer
        self._twr_distances: Dict[Tuple[int, int], List[float]] = {}
    
    def reset(self):
        """Reset calibration data."""
        self._distance_matrix = None
        self._calibrated_positions = None
        self._twr_distances = {}
    
    def add_distance_measurement(self, from_anchor: int, to_anchor: int, distance_mm: float):
        """
        Add a distance measurement between two anchors.
        
        Args:
            from_anchor: Source anchor ID
            to_anchor: Target anchor ID
            distance_mm: Measured distance in millimeters
        """
        if distance_mm <= 0:
            return
        
        # Store both directions
        key1 = (from_anchor, to_anchor)
        key2 = (to_anchor, from_anchor)
        
        if key1 not in self._twr_distances:
            self._twr_distances[key1] = []
        if key2 not in self._twr_distances:
            self._twr_distances[key2] = []
        
        self._twr_distances[key1].append(distance_mm)
        self._twr_distances[key2].append(distance_mm)
    
    def set_distance_matrix(self, matrix: np.ndarray):
        """
        Set the full distance matrix directly.
        
        Args:
            matrix: NxN symmetric matrix of distances in millimeters.
                    Diagonal should be 0.
        """
        self._distance_matrix = matrix.astype(float)
    
    def build_distance_matrix(self) -> Optional[np.ndarray]:
        """
        Build distance matrix from collected measurements.
        Uses average of multiple measurements for each pair.
        
        Returns:
            Distance matrix or None if insufficient data.
        """
        matrix = np.zeros((self.num_anchors, self.num_anchors))
        
        for (i, j), distances in self._twr_distances.items():
            if i < self.num_anchors and j < self.num_anchors and distances:
                # Use median for robustness
                matrix[i, j] = np.median(distances)
        
        # Check if we have enough measurements
        # Need all pairwise distances for MDS
        for i in range(self.num_anchors):
            for j in range(i + 1, self.num_anchors):
                if matrix[i, j] == 0 and matrix[j, i] == 0:
                    print(f"[Calibration] Missing distance between A{i} and A{j}")
                    return None
                # Ensure symmetry
                if matrix[i, j] == 0:
                    matrix[i, j] = matrix[j, i]
                elif matrix[j, i] == 0:
                    matrix[j, i] = matrix[i, j]
        
        self._distance_matrix = matrix
        return matrix
    
    def _mds(self, twr_distance: np.ndarray) -> np.ndarray:
        """
        Multi-Dimensional Scaling algorithm.
        Estimates coordinates from distance matrix.
        
        Based on RTLSClient::mds() from the C++ code.
        
        Args:
            twr_distance: NxN distance matrix in meters
        
        Returns:
            Nx2 coordinate matrix (relative to centroid)
        """
        n_nodes = twr_distance.shape[0]
        
        # Square the distances
        dist_squared = np.square(twr_distance)
        
        # Centering operator: I - (1/n) * ones
        centering_operator = np.eye(n_nodes) - np.ones((n_nodes, n_nodes)) / n_nodes
        
        # Double-centered distance matrix: B = -0.5 * J * D^2 * J
        centered_dist_squared = -0.5 * centering_operator @ dist_squared @ centering_operator
        
        # SVD decomposition
        u, s, vh = np.linalg.svd(centered_dist_squared)
        
        # Extract coordinates using sqrt of eigenvalues
        est_geom = np.zeros((n_nodes, self.n_dim))
        for i in range(min(self.n_dim, len(s))):
            if s[i] > 0:
                est_geom[:, i] = u[:, i] * np.sqrt(s[i])
        
        # Translate to have view_node at origin
        trans_coord = est_geom - est_geom[self.view_node, :]
        
        return trans_coord
    
    def _angle_rotation(self, trans_coord: np.ndarray) -> np.ndarray:
        """
        Rotate coordinates to align A1 on the positive x-axis.
        
        Based on RTLSClient::angleRotation() from the C++ code.
        
        Args:
            trans_coord: Nx2 coordinate matrix from MDS
        
        Returns:
            Nx2 rotated coordinate matrix
        """
        n_nodes = trans_coord.shape[0]
        
        # Calculate distances from origin for each node
        trans_distance = np.sqrt(np.sum(trans_coord**2, axis=1))
        
        # Calculate current angles
        current_angle = np.zeros(n_nodes)
        for i in range(n_nodes):
            if trans_distance[i] > 0:
                cos_val = trans_coord[i, 0] / trans_distance[i]
                # Clamp to [-1, 1] to avoid numerical errors
                cos_val = np.clip(cos_val, -1, 1)
                current_angle[i] = np.arccos(cos_val)
                # Adjust sign based on y coordinate
                if trans_coord[i, 1] < 0:
                    current_angle[i] = -current_angle[i]
        
        # Rotation angle to align A1 with x-axis
        rotate_angle = current_angle[1]  # Angle of anchor 1
        
        # Build rotation matrix
        rotation_matrix = np.array([
            [np.cos(rotate_angle), -np.sin(rotate_angle)],
            [np.sin(rotate_angle), np.cos(rotate_angle)]
        ])
        
        # Apply rotation
        coord = trans_coord @ rotation_matrix
        
        # Ensure A2 is in positive y (if it exists and should be)
        if n_nodes > 2 and coord[2, 1] < 0:
            # Flip y axis
            coord[:, 1] = -coord[:, 1]
        
        return coord
    
    def calibrate(self) -> Optional[List[AnchorPosition]]:
        """
        Perform calibration and return anchor positions.
        
        Returns:
            List of AnchorPosition objects, or None if calibration failed.
        """
        # Build distance matrix if not set directly
        if self._distance_matrix is None:
            if not self.build_distance_matrix():
                print("[Calibration] Failed to build distance matrix")
                return None
        
        # Convert to meters
        distance_m = self._distance_matrix / 1000.0
        
        print(f"[Calibration] Distance matrix (m):\n{distance_m}")
        
        try:
            # Apply MDS
            trans_coord = self._mds(distance_m)
            print(f"[Calibration] MDS result:\n{trans_coord}")
            
            # Apply rotation alignment
            est_coord = self._angle_rotation(trans_coord)
            print(f"[Calibration] Aligned coordinates:\n{est_coord}")
            
            # Build result
            self._calibrated_positions = []
            for i in range(self.num_anchors):
                pos = AnchorPosition(
                    id=i,
                    x=est_coord[i, 0],
                    y=est_coord[i, 1],
                    z=0.0  # Assume 2D for now
                )
                self._calibrated_positions.append(pos)
            
            return self._calibrated_positions
            
        except Exception as e:
            print(f"[Calibration] Error: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def get_positions(self) -> Optional[List[AnchorPosition]]:
        """Get calibrated positions (must call calibrate() first)."""
        return self._calibrated_positions
    
    def get_distance_info(self) -> Dict[str, Any]:
        """Get information about collected distances."""
        info = {
            "num_pairs": len(self._twr_distances),
            "pairs": {}
        }
        
        for (i, j), distances in self._twr_distances.items():
            if i < j:  # Only show each pair once
                key = f"A{i}-A{j}"
                info["pairs"][key] = {
                    "count": len(distances),
                    "mean_mm": np.mean(distances) if distances else 0,
                    "std_mm": np.std(distances) if len(distances) > 1 else 0
                }
        
        return info


def calibrate_from_ranges(
    ranges_data: List[List[int]], 
    num_anchors: int = 3
) -> Optional[List[AnchorPosition]]:
    """
    Calibrate anchor positions from a list of range measurements.
    
    This function processes measurements where each anchor measures
    distances to all other anchors.
    
    Args:
        ranges_data: List of range measurements. Each entry contains
                     distances from one anchor to all others.
        num_anchors: Number of anchors
    
    Returns:
        List of AnchorPosition or None if calibration failed.
    """
    calibrator = AnchorCalibration(num_anchors=num_anchors)
    
    # Process range data
    for data in ranges_data:
        if len(data) >= num_anchors:
            # Assuming data[i] is distance from sender to anchor i
            # We need to know which anchor sent this data
            # This requires proper data format
            pass
    
    return calibrator.calibrate()


def calibrate_from_distance_matrix(
    distances: List[List[float]], 
    num_anchors: int = 3
) -> Optional[List[AnchorPosition]]:
    """
    Calibrate anchor positions from a distance matrix.
    
    Args:
        distances: NxN list of distances in millimeters
        num_anchors: Number of anchors
    
    Returns:
        List of AnchorPosition or None if calibration failed.
    """
    calibrator = AnchorCalibration(num_anchors=num_anchors)
    matrix = np.array(distances[:num_anchors])[:, :num_anchors]
    calibrator.set_distance_matrix(matrix)
    return calibrator.calibrate()


# Test function
def test_calibration():
    """Test the calibration with known distances."""
    # Test with a simple 3-anchor setup
    # A0 at (0, 0), A1 at (10, 0), A2 at (5, 8.66) - equilateral triangle
    
    # Distances (in mm)
    d01 = 10000  # A0 to A1: 10m
    d02 = 10000  # A0 to A2: 10m
    d12 = 10000  # A1 to A2: 10m
    
    distance_matrix = [
        [0, d01, d02],
        [d01, 0, d12],
        [d02, d12, 0]
    ]
    
    result = calibrate_from_distance_matrix(distance_matrix, num_anchors=3)
    
    if result:
        print("\nCalibration Result:")
        for pos in result:
            print(f"  Anchor {pos.id}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    else:
        print("Calibration failed")


if __name__ == "__main__":
    test_calibration()

