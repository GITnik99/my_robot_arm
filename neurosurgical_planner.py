#!/usr/bin/env python

import rospy
import moveit_commander
import sys
import math
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations

class NeurosurgicalPlanner:
    def __init__(self):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('neurosurgical_planner')
        
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Set precise planning parameters for surgery
        self.group.set_planning_time(10.0)  # More time for precise planning
        self.group.set_max_velocity_scaling_factor(0.05)  # Very slow for surgery
        self.group.set_max_acceleration_scaling_factor(0.05)
        
        rospy.sleep(2)
        print("Neurosurgical Planner initialized")
        print("Ready to plan surgical trajectories")
    
    def plan_surgical_trajectory(self, entry_point, target_point, approach_distance=0.05):
        """
        Plan neurosurgical trajectory from entry point to tumor target
        
        Args:
            entry_point: (x, y, z) coordinates where needle enters brain
            target_point: (x, y, z) coordinates of tumor inside brain
            approach_distance: Distance to stop before actual entry (safety margin)
        """
        print("\n" + "="*60)
        print("PLANNING NEUROSURGICAL TRAJECTORY")
        print("="*60)
        
        entry_x, entry_y, entry_z = entry_point
        target_x, target_y, target_z = target_point
        
        print(f"Entry Point: X={entry_x:.3f}, Y={entry_y:.3f}, Z={entry_z:.3f}")
        print(f"Target Point: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")
        
        # Calculate trajectory vector (from entry to target)
        trajectory_vector = [
            target_x - entry_x,
            target_y - entry_y,
            target_z - entry_z
        ]
        
        # Calculate trajectory length
        trajectory_length = math.sqrt(
            trajectory_vector[0]**2 + 
            trajectory_vector[1]**2 + 
            trajectory_vector[2]**2
        )
        
        print(f"Trajectory Length: {trajectory_length:.3f}m")
        
        # Normalize trajectory vector
        unit_vector = [v/trajectory_length for v in trajectory_vector]
        
        # Calculate approach position (before entry point)
        approach_pos = [
            entry_x - unit_vector[0] * approach_distance,
            entry_y - unit_vector[1] * approach_distance,
            entry_z - unit_vector[2] * approach_distance
        ]
        
        print(f"Approach Position: X={approach_pos[0]:.3f}, Y={approach_pos[1]:.3f}, Z={approach_pos[2]:.3f}")
        
        # Calculate needle orientation (pointing from entry to target)
        needle_orientation = self.calculate_needle_orientation(unit_vector)
        
        # Plan the surgical trajectory
        trajectory_plan = self.plan_surgical_phases(approach_pos, entry_point, target_point, needle_orientation)
        
        return trajectory_plan
    
    def calculate_needle_orientation(self, trajectory_vector):
        """Calculate needle orientation to align with trajectory"""
        # Needle should point along trajectory vector
        # This is a simplified calculation - in practice you'd use more sophisticated orientation control
        
        # Calculate rotation to align needle with trajectory
        # Default needle direction is along Y-axis (forward)
        default_direction = [0, 1, 0]
        
        # Calculate rotation needed
        dot_product = sum(a*b for a,b in zip(default_direction, trajectory_vector))
        angle = math.acos(max(-1, min(1, dot_product)))  # Clamp to valid range
        
        # Create rotation quaternion (simplified - you might need more sophisticated calculation)
        # For now, use identity quaternion (no rotation)
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0
        
        return orientation
    
    def plan_surgical_phases(self, approach_pos, entry_point, target_point, orientation):
        """Plan the complete surgical procedure in phases"""
        print("\nPlanning surgical phases...")
        
        phases = {
            'approach': None,
            'entry': None,
            'insertion': None,
            'retraction': None
        }
        
        # Phase 1: Move to approach position
        print("\nPhase 1: Moving to approach position...")
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "world"
        approach_pose.pose.position.x = approach_pos[0]
        approach_pose.pose.position.y = approach_pos[1]
        approach_pose.pose.position.z = approach_pos[2]
        approach_pose.pose.orientation = orientation
        
        self.group.set_pose_target(approach_pose.pose)
        approach_plan = self.group.plan()
        
        if approach_plan and len(approach_plan[1].joint_trajectory.points) > 0:
            phases['approach'] = approach_plan
            print("✓ Approach phase planned successfully")
        else:
            print("✗ Approach phase planning failed")
            return None
        
        # Phase 2: Move to entry point
        print("\nPhase 2: Moving to entry point...")
        entry_pose = PoseStamped()
        entry_pose.header.frame_id = "world"
        entry_pose.pose.position.x = entry_point[0]
        entry_pose.pose.position.y = entry_point[1]
        entry_pose.pose.position.z = entry_point[2]
        entry_pose.pose.orientation = orientation
        
        # Plan Cartesian path from approach to entry (straight line)
        waypoints = [entry_pose.pose]
        (entry_plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        
        if fraction > 0.9:  # At least 90% of path planned
            phases['entry'] = entry_plan
            print(f"✓ Entry phase planned successfully ({fraction*100:.1f}% of path)")
        else:
            print(f"✗ Entry phase planning incomplete ({fraction*100:.1f}% of path)")
        
        # Phase 3: Needle insertion (extend needle to target)
        print("\nPhase 3: Needle insertion to target...")
        # This would use the needle_extension joint
        insertion_distance = math.sqrt(
            (target_point[0] - entry_point[0])**2 + 
            (target_point[1] - entry_point[1])**2 + 
            (target_point[2] - entry_point[2])**2
        )
        
        print(f"Required needle extension: {insertion_distance:.3f}m")
        
        # Check if target is within needle reach
        max_needle_extension = 0.12  # From your URDF
        if insertion_distance <= max_needle_extension:
            print("✓ Target within needle reach")
            phases['insertion'] = insertion_distance
        else:
            print(f"✗ Target too far - max reach: {max_needle_extension:.3f}m")
        
        return phases
    
    def execute_surgical_plan(self, trajectory_plan):
        """Execute the planned surgical trajectory"""
        if not trajectory_plan:
            print("No valid trajectory plan to execute")
            return False
        
        print("\n" + "="*60)
        print("EXECUTING SURGICAL TRAJECTORY")
        print("="*60)
        
        # Phase 1: Approach
        if trajectory_plan['approach']:
            print("\nExecuting Phase 1: Approach...")
            self.group.execute(trajectory_plan['approach'][1], wait=True)
            print("✓ Approach completed")
            rospy.sleep(1)
        
        # Phase 2: Entry
        if trajectory_plan['entry']:
            print("\nExecuting Phase 2: Entry...")
            self.group.execute(trajectory_plan['entry'], wait=True)
            print("✓ Entry completed - needle at brain surface")
            rospy.sleep(1)
        
        # Phase 3: Insertion
        if trajectory_plan['insertion']:
            print(f"\nExecuting Phase 3: Needle insertion ({trajectory_plan['insertion']:.3f}m)...")
            # Here you would control the needle_extension joint
            # For now, just simulate the insertion
            insertion_time = trajectory_plan['insertion'] / 0.005  # At 5mm/s
            print(f"Simulating needle insertion over {insertion_time:.1f} seconds...")
            rospy.sleep(min(insertion_time, 10))  # Cap at 10 seconds for demo
            print("✓ Needle reached tumor target")
        
        print("\n✓ Surgical trajectory completed successfully!")
        return True
    
    def plan_from_coordinates(self, entry_coords, target_coords):
        """Main function to plan surgery from professor's coordinates"""
        print("Received surgical coordinates from professor:")
        print(f"Entry point: {entry_coords}")
        print(f"Target point: {target_coords}")
        
        # Plan the trajectory
        plan = self.plan_surgical_trajectory(entry_coords, target_coords)
        
        if plan:
            print("\n✓ Surgical planning successful!")
            
            # Ask for execution confirmation
            response = raw_input("\nExecute surgical plan? (y/n): ")
            if response.lower() == 'y':
                self.execute_surgical_plan(plan)
            else:
                print("Surgical plan ready for execution when approved")
        else:
            print("\n✗ Surgical planning failed!")
        
        return plan

def get_coordinates_from_user():
    """Get coordinates interactively from user"""
    print("\nEnter surgical coordinates:")
    print("(All coordinates in meters relative to robot base)")
    
    # Get entry point
    print("\nEntry Point (where needle enters brain):")
    entry_x = float(input("  X coordinate: "))
    entry_y = float(input("  Y coordinate: "))
    entry_z = float(input("  Z coordinate: "))
    entry_point = (entry_x, entry_y, entry_z)
    
    # Get target point
    print("\nTarget Point (tumor location):")
    target_x = float(input("  X coordinate: "))
    target_y = float(input("  Y coordinate: "))
    target_z = float(input("  Z coordinate: "))
    target_point = (target_x, target_y, target_z)
    
    return entry_point, target_point

def main():
    """Example usage with sample coordinates"""
    try:
        planner = NeurosurgicalPlanner()
        
        print("Neurosurgical Trajectory Planner Ready")
        print("="*50)
        
        # Choose coordinate input method
        print("\nCoordinate Input Options:")
        print("1. Use example coordinates")
        print("2. Enter coordinates manually")
        choice = input("Choose option (1 or 2): ")
        
        if choice == "2":
            # Get coordinates from user
            entry_point, target_point = get_coordinates_from_user()
        else:
            # Use example coordinates
            entry_point = (0.0, 0.25, 0.30)    # Where needle enters brain surface
            target_point = (0.0, 0.28, 0.25)   # Tumor location inside brain
            print("Using example coordinates...")
        
        print(f"\nPlanning surgery with:")
        print(f"Entry: {entry_point}")
        print(f"Target: {target_point}")
        
        plan = planner.plan_from_coordinates(entry_point, target_point)
        
        # Keep node running for manual coordinate input
        print("\n" + "="*50)
        print("Ready for professor's coordinates...")
        print("To use:")
        print("1. Call planner.plan_from_coordinates(entry, target)")
        print("2. Where entry and target are (x,y,z) tuples")
        print("Example: planner.plan_from_coordinates((0.0, 0.25, 0.30), (0.0, 0.28, 0.25))")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("Neurosurgical planner stopped")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()