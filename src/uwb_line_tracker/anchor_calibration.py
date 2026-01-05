"""
UWB Anchor Self-Calibration Module.

Based on the C++ RTLSClient implementation:
- MDS (Multi-Dimensional Scaling) for shape estimation from distance matrix
- Angle rotation to align coordinates (A0 at origin, A1 on X-axis)

This module allows automatic calibration of anchor positions from 
inter-anchor range measurements.
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class AnchorPosition:
    """Anchor position in 3D space."""
    x: float
    y: float
    z: float
    anchor_id: int = 0


@dataclass
class CalibrationResult:
    """Result of anchor calibration."""
    anchors: List[AnchorPosition]
    success: bool
    error_message: str = ""
    

class AnchorCalibrator:
    """
    Anchor self-calibration using MDS (Multi-Dimensional Scaling).
    
    Algorithm based on RTLSClient.cpp lines 1046-1179:
    1. Collect inter-anchor range measurements
    2. Use MDS to estimate relative positions from distance matrix
    3. Apply angle rotation to align with standard coordinate system
       (A0 at origin, A1 on positive X-axis, A2 in first quadrant)
    """
    
    def __init__(self, num_anchors: int = 3, n_dim: int = 2):
        """
        Initialize calibrator.
        
        Args:
            num_anchors: Number of anchors (typically 3)
            n_dim: Number of dimensions (2 for 2D, 3 for 3D)
        """
        self.num_anchors = num_anchors
        self.n_dim = n_dim
        self.distance_matrix = np.zeros((num_anchors, num_anchors))
        self.calibrated_positions: Optional[List[AnchorPosition]] = None
        
    def set_distance(self, anchor_i: int, anchor_j: int, distance: float):
        """
        Set distance between two anchors.
        
        Args:
            anchor_i: First anchor ID
            anchor_j: Second anchor ID
            distance: Distance in meters
        """
        if 0 <= anchor_i < self.num_anchors and 0 <= anchor_j < self.num_anchors:
            self.distance_matrix[anchor_i, anchor_j] = distance
            self.distance_matrix[anchor_j, anchor_i] = distance  # Symmetric
    
    def set_distance_matrix(self, matrix: np.ndarray):
        """Set the full distance matrix."""
        self.distance_matrix = matrix.copy()
        
    def mds(self, distance_matrix: np.ndarray, view_node: int = 0) -> np.ndarray:
        """
        Multi-Dimensional Scaling to estimate positions from distance matrix.
        
        Based on RTLSClient::mds() function.
        
        Args:
            distance_matrix: n x n matrix of inter-anchor distances
            view_node: Reference node (coordinates relative to this node)
            
        Returns:
            n x n_dim array of estimated coordinates
        """
        n_nodes = distance_matrix.shape[0]
        
        # Step 1: Square the distances
        dist_squared = distance_matrix ** 2
        
        # Step 2: Double centering
        # J = I - (1/n) * ones(n,n)
        centering_operator = np.eye(n_nodes) - np.ones((n_nodes, n_nodes)) / n_nodes
        
        # B = -0.5 * J * D^2 * J
        centered_dist_squared = -0.5 * centering_operator @ dist_squared @ centering_operator
        
        # Step 3: SVD decomposition
        u, s, vh = np.linalg.svd(centered_dist_squared)
        
        # Step 4: Estimate geometry from eigenvalues
        # Take first n_dim dimensions
        est_geom = np.zeros((n_nodes, self.n_dim))
        for i in range(min(self.n_dim, len(s))):
            if s[i] > 0:
                est_geom[:, i] = u[:, i] * np.sqrt(s[i])
        
        # Step 5: Translate so view_node is at origin
        trans_coord = est_geom - est_geom[view_node, :]
        
        return trans_coord
    
    def angle_rotation(self, trans_coord: np.ndarray) -> np.ndarray:
        """
        Rotate coordinates so that:
        - A0 is at origin
        - A1 is on the positive X-axis
        - A2 is in the first quadrant (positive Y)
        
        Based on RTLSClient::angleRotation() function.
        
        Args:
            trans_coord: n x n_dim array of translated coordinates
            
        Returns:
            n x n_dim array of rotated coordinates
        """
        n_nodes = trans_coord.shape[0]
        
        # Calculate distance from origin for each node
        trans_distance = np.sqrt(np.sum(trans_coord ** 2, axis=1))
        
        # Calculate current angle for each node
        # Avoid division by zero
        current_angle = np.zeros(n_nodes)
        for i in range(n_nodes):
            if trans_distance[i] > 1e-10:
                angle = np.arccos(np.clip(trans_coord[i, 0] / trans_distance[i], -1, 1))
                if trans_coord[i, 1] < 0:
                    angle = -angle
                current_angle[i] = angle
        
        # Rotation angle is based on anchor 1's position
        # We want anchor 1 to be on the positive X-axis (angle = 0)
        rotate_angle = current_angle[1] if len(current_angle) > 1 else 0
        
        # Build rotation matrix (2D)
        cos_a = np.cos(rotate_angle)
        sin_a = np.sin(rotate_angle)
        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a, cos_a]
        ])
        
        # Apply rotation
        if self.n_dim == 2:
            coord = trans_coord @ rotation_matrix
        else:
            # For 3D, only rotate X-Y plane
            coord = trans_coord.copy()
            coord[:, :2] = trans_coord[:, :2] @ rotation_matrix
        
        # Ensure A2 is in positive Y (first quadrant)
        if n_nodes > 2 and coord[2, 1] < 0:
            coord[:, 1] = -coord[:, 1]
        
        return coord
    
    def calibrate(self) -> CalibrationResult:
        """
        Perform anchor calibration using stored distance matrix.
        
        Returns:
            CalibrationResult with calibrated anchor positions
        """
        try:
            # Validate distance matrix
            if not self._validate_distances():
                return CalibrationResult(
                    anchors=[],
                    success=False,
                    error_message="Invalid distance matrix: some distances are zero or negative"
                )
            
            # Step 1: MDS to get relative positions
            trans_coord = self.mds(self.distance_matrix, view_node=0)
            
            # Step 2: Angle rotation to align coordinate system
            est_coord = self.angle_rotation(trans_coord)
            
            # Step 3: Create anchor positions
            anchors = []
            for i in range(self.num_anchors):
                pos = AnchorPosition(
                    x=float(est_coord[i, 0]),
                    y=float(est_coord[i, 1]),
                    z=0.0,  # Assuming 2D for now
                    anchor_id=i
                )
                anchors.append(pos)
            
            self.calibrated_positions = anchors
            
            return CalibrationResult(
                anchors=anchors,
                success=True
            )
            
        except Exception as e:
            return CalibrationResult(
                anchors=[],
                success=False,
                error_message=str(e)
            )
    
    def _validate_distances(self) -> bool:
        """Validate that all necessary distances are positive."""
        # Check that off-diagonal elements are positive
        for i in range(self.num_anchors):
            for j in range(i + 1, self.num_anchors):
                if self.distance_matrix[i, j] <= 0:
                    return False
        return True
    
    def calibrate_from_ranges(self, ranges: List[List[float]]) -> CalibrationResult:
        """
        Calibrate from a list of range measurements.
        
        Args:
            ranges: List of [anchor_i, anchor_j, distance] tuples
            
        Returns:
            CalibrationResult
        """
        for r in ranges:
            if len(r) >= 3:
                self.set_distance(int(r[0]), int(r[1]), r[2])
        return self.calibrate()


class RealTimeCalibrator:
    """
    Real-time anchor calibration that accumulates measurements.
    
    Designed to work with UWB hardware that provides inter-anchor
    range measurements periodically.
    """
    
    def __init__(self, num_anchors: int = 3):
        self.num_anchors = num_anchors
        self.calibrator = AnchorCalibrator(num_anchors=num_anchors)
        
        # Measurement accumulator with averaging
        self.measurement_counts = np.zeros((num_anchors, num_anchors))
        self.measurement_sums = np.zeros((num_anchors, num_anchors))
        
        self.last_result: Optional[CalibrationResult] = None
        self.is_calibrated = False
    
    def add_measurement(self, anchor_i: int, anchor_j: int, distance: float):
        """
        Add a single distance measurement.
        
        Args:
            anchor_i: First anchor ID
            anchor_j: Second anchor ID  
            distance: Measured distance in meters
        """
        if distance > 0:
            self.measurement_counts[anchor_i, anchor_j] += 1
            self.measurement_counts[anchor_j, anchor_i] += 1
            self.measurement_sums[anchor_i, anchor_j] += distance
            self.measurement_sums[anchor_j, anchor_i] += distance
    
    def get_average_distances(self) -> np.ndarray:
        """Get averaged distance matrix."""
        with np.errstate(divide='ignore', invalid='ignore'):
            avg = np.where(
                self.measurement_counts > 0,
                self.measurement_sums / self.measurement_counts,
                0
            )
        return avg
    
    def get_measurement_status(self) -> dict:
        """Get status of measurements for each anchor pair."""
        status = {}
        for i in range(self.num_anchors):
            for j in range(i + 1, self.num_anchors):
                key = f"A{i}-A{j}"
                count = int(self.measurement_counts[i, j])
                avg = 0.0
                if count > 0:
                    avg = self.measurement_sums[i, j] / count
                status[key] = {
                    "count": count,
                    "average_distance": avg
                }
        return status
    
    def has_sufficient_data(self) -> bool:
        """Check if we have at least one measurement for each anchor pair."""
        for i in range(self.num_anchors):
            for j in range(i + 1, self.num_anchors):
                if self.measurement_counts[i, j] < 1:
                    return False
        return True
    
    def perform_calibration(self) -> CalibrationResult:
        """
        Perform calibration using accumulated measurements.
        
        Returns:
            CalibrationResult
        """
        if not self.has_sufficient_data():
            return CalibrationResult(
                anchors=[],
                success=False,
                error_message="Insufficient data: need at least one measurement for each anchor pair"
            )
        
        # Set averaged distances
        avg_distances = self.get_average_distances()
        self.calibrator.set_distance_matrix(avg_distances)
        
        # Perform calibration
        result = self.calibrator.calibrate()
        self.last_result = result
        self.is_calibrated = result.success
        
        return result
    
    def reset(self):
        """Reset all measurements."""
        self.measurement_counts = np.zeros((self.num_anchors, self.num_anchors))
        self.measurement_sums = np.zeros((self.num_anchors, self.num_anchors))
        self.last_result = None
        self.is_calibrated = False


def test_calibration():
    """Test the calibration with known anchor positions."""
    # Known anchor positions (for testing)
    # A0: (0, 0), A1: (10, 0), A2: (5, 8.66) - equilateral triangle
    true_positions = [
        (0, 0),
        (10, 0),
        (5, 8.66)
    ]
    
    # Calculate true distances
    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    d01 = dist(true_positions[0], true_positions[1])
    d02 = dist(true_positions[0], true_positions[2])
    d12 = dist(true_positions[1], true_positions[2])
    
    print(f"True distances: A0-A1={d01:.2f}m, A0-A2={d02:.2f}m, A1-A2={d12:.2f}m")
    
    # Create calibrator and set distances
    calibrator = AnchorCalibrator(num_anchors=3)
    calibrator.set_distance(0, 1, d01)
    calibrator.set_distance(0, 2, d02)
    calibrator.set_distance(1, 2, d12)
    
    # Calibrate
    result = calibrator.calibrate()
    
    if result.success:
        print("\nCalibration successful!")
        print("Estimated anchor positions:")
        for anchor in result.anchors:
            print(f"  A{anchor.anchor_id}: ({anchor.x:.2f}, {anchor.y:.2f})")
    else:
        print(f"Calibration failed: {result.error_message}")
    
    return result


if __name__ == "__main__":
    test_calibration()

