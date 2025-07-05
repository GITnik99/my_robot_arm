#!/usr/bin/env python3
"""
Surgical Robot Constraint System
Implements realistic constraints for a wall-mounted neurosurgical robot
operating above a patient's head on a surgical table.
"""

import numpy as np
import math
from typing import Tuple, List, Optional

class SurgicalConstraints:
    """
    Constraint system for neurosurgical robot operations.
    
    Coordinate system (robot base at origin):
    - X: left/right (positive = right when facing away from wall)
    - Y: forward from wall (positive = away from wall) 
    - Z: up/down (positive = up)
    """
    
    def __init__(self):
        # Table dimensions and position
        self.table_width = 0.60      # 60cm wide
        self.table_length = 2.00     # 200cm long  
        self.table_height = 1.00     # 100cm high
        
        # Table position relative to robot base (origin)
        self.table_x_min = -self.table_width / 2   # -0.30m (left edge)
        self.table_x_max = self.table_width / 2    # +0.30m (right edge)
        self.table_y_min = 0.20                    # 0.20m (head end - 20cm from wall)
        self.table_y_max = 2.20                    # 2.20m (foot end)
        self.table_z_surface = 0.00                # Table surface at robot base level
        self.table_z_bottom = -self.table_height  # -1.00m (bottom of table)
        
        # Patient head (sphere)
        self.head_radius = 0.10      # 10cm radius (20cm diameter)
        self.head_center = np.array([0.0, 0.20, 0.05])  # Centered, at head end, 5cm elevated
        
        print("Surgical Constraint System Initialized")
        print(f"Table bounds: X[{self.table_x_min:.2f}, {self.table_x_max:.2f}] "
              f"Y[{self.table_y_min:.2f}, {self.table_y_max:.2f}] "
              f"Z[{self.table_z_bottom:.2f}, {self.table_z_surface:.2f}]")
        print(f"Head sphere: center {self.head_center}, radius {self.head_radius:.2f}m")
    
    def is_above_table(self, point: np.ndarray) -> bool:
        """
        Check if a point is above the table surface within table boundaries.
        
        Args:
            point: 3D point [x, y, z] in robot base coordinates
            
        Returns:
            True if point is in valid workspace above table
        """
        x, y, z = point
        
        # Check if within table X-Y boundaries
        within_table_xy = (self.table_x_min <= x <= self.table_x_max and 
                          self.table_y_min <= y <= self.table_y_max)
        
        # Check if above table surface
        above_surface = z >= self.table_z_surface
        
        return within_table_xy and above_surface
    
    def is_colliding_with_head(self, point: np.ndarray) -> bool:
        """
        Check if a point collides with (is inside) the patient's head sphere.
        
        Args:
            point: 3D point [x, y, z] in robot base coordinates
            
        Returns:
            True if point is inside head sphere (collision detected)
        """
        distance_to_head_center = np.linalg.norm(point - self.head_center)
        return distance_to_head_center <= self.head_radius
    
    def is_valid_position(self, point: np.ndarray) -> bool:
        """
        Check if a point satisfies all constraints (main constraint function).
        
        Args:
            point: 3D point [x, y, z] in robot base coordinates
            
        Returns:
            True if position is valid (above table AND not colliding with head)
        """
        return self.is_above_table(point) and not self.is_colliding_with_head(point)
    
    def check_trajectory(self, trajectory: List[np.ndarray]) -> Tuple[bool, List[int]]:
        """
        Check if an entire trajectory satisfies constraints.
        
        Args:
            trajectory: List of 3D points representing robot path
            
        Returns:
            Tuple of (all_valid: bool, violation_indices: List[int])
        """
        violation_indices = []
        
        for i, point in enumerate(trajectory):
            if not self.is_valid_position(point):
                violation_indices.append(i)
        
        all_valid = len(violation_indices) == 0
        return all_valid, violation_indices
    
    def get_violation_info(self, point: np.ndarray) -> dict:
        """
        Get detailed information about constraint violations for a point.
        
        Args:
            point: 3D point [x, y, z] in robot base coordinates
            
        Returns:
            Dictionary with violation details
        """
        info = {
            'point': point.copy(),
            'is_valid': self.is_valid_position(point),
            'above_table': self.is_above_table(point),
            'head_collision': self.is_colliding_with_head(point),
            'distance_to_head': np.linalg.norm(point - self.head_center),
            'head_penetration_depth': max(0, self.head_radius - np.linalg.norm(point - self.head_center))
        }
        
        # Add specific violation reasons
        info['violations'] = []
        if not info['above_table']:
            x, y, z = point
            if not (self.table_x_min <= x <= self.table_x_max):
                info['violations'].append('Outside table X bounds')
            if not (self.table_y_min <= y <= self.table_y_max):
                info['violations'].append('Outside table Y bounds')
            if z < self.table_z_surface:
                info['violations'].append('Below table surface')
        
        if info['head_collision']:
            info['violations'].append('Collision with patient head')
        
        return info
    
    def visualize_workspace(self) -> str:
        """
        Generate a text description of the workspace setup.
        
        Returns:
            String description of the constraint setup
        """
        description = f"""
Neurosurgical Robot Workspace Configuration:
==========================================

Robot Base (Origin): [0.0, 0.0, 0.0]

Surgical Table:
- Dimensions: {self.table_width:.1f}m × {self.table_length:.1f}m × {self.table_height:.1f}m
- Position: X[{self.table_x_min:.2f}, {self.table_x_max:.2f}] Y[{self.table_y_min:.2f}, {self.table_y_max:.2f}] Z[{self.table_z_bottom:.2f}, {self.table_z_surface:.2f}]
- Distance from wall: {self.table_y_min:.2f}m

Patient Head:
- Shape: Sphere with radius {self.head_radius:.2f}m (diameter {2*self.head_radius:.2f}m)
- Position: [{self.head_center[0]:.2f}, {self.head_center[1]:.2f}, {self.head_center[2]:.2f}]
- Elevation above table: {self.head_center[2] - self.table_z_surface:.2f}m

Valid Workspace:
- Must be above table surface (Z ≥ {self.table_z_surface:.2f}m)
- Must be within table boundaries
- Must not penetrate head sphere
        """
        return description


def demo_constraint_system():
    """
    Demonstration of the constraint system with example points.
    """
    print("=== Surgical Robot Constraint System Demo ===\n")
    
    # Initialize constraint system
    constraints = SurgicalConstraints()
    print(constraints.visualize_workspace())
    
    # Test points
    test_points = [
        np.array([0.0, 0.5, 0.1]),    # Valid: above table, away from head
        np.array([0.0, 0.2, 0.05]),   # Invalid: inside head sphere
        np.array([0.5, 0.5, 0.1]),    # Invalid: outside table X bounds
        np.array([0.0, 3.0, 0.1]),    # Invalid: outside table Y bounds  
        np.array([0.0, 0.5, -0.1]),   # Invalid: below table surface
        np.array([0.0, 0.32, 0.1]),   # Valid: just outside head sphere
    ]
    
    print("\n=== Testing Individual Points ===")
    for i, point in enumerate(test_points):
        info = constraints.get_violation_info(point)
        status = "✓ VALID" if info['is_valid'] else "✗ INVALID"
        print(f"\nPoint {i+1}: [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}] - {status}")
        
        if not info['is_valid']:
            print(f"  Violations: {', '.join(info['violations'])}")
        
        print(f"  Distance to head: {info['distance_to_head']:.3f}m")
        if info['head_penetration_depth'] > 0:
            print(f"  Head penetration: {info['head_penetration_depth']:.3f}m")
    
    # Test trajectory
    print("\n=== Testing Sample Trajectory ===")
    # Simple linear trajectory from safe point toward head
    start = np.array([0.0, 0.5, 0.1])
    end = np.array([0.0, 0.15, 0.05])
    trajectory = [start + t * (end - start) for t in np.linspace(0, 1, 10)]
    
    is_safe, violations = constraints.check_trajectory(trajectory)
    print(f"Trajectory safety: {'✓ SAFE' if is_safe else '✗ VIOLATIONS DETECTED'}")
    if violations:
        print(f"Violation points: {violations}")


if __name__ == "__main__":
    demo_constraint_system()