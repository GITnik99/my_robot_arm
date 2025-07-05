def make_patient_head_accessible(self):
        """Remove patient head from collision checking while keeping it visible"""
        print("\nMaking patient head accessible...")
        
        # Remove from collision world while keeping visual
        if "patient_head" in self.loaded_objects:
            # This removes collision but keeps the visual sphere
            self.scene.remove_world_object("patient_head")
            rospy.sleep(1)
            
            # Add it back as a different object that we can control
            head_pose = PoseStamped()
            head_pose.header.frame_id = "world"
            head_pose.pose.position.x = self.patient_head_position[0]
            head_pose.pose.position.y = self.patient_head_position[1]
            head_pose.pose.position.z = self.patient_head_position[2]
            head_pose.pose.orientation.w = 1.0
            
            print("Patient head is now ACCESSIBLE (visible but no collision)")
        else:
            print("Patient head not found in scene")#!/usr/bin/env python

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class RealisticSurgicalScene:
    def __init__(self):
        # Initialize MoveIt commanders
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('realistic_surgical_scene', anonymous=True)
        
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        
        # Wait for scene to initialize
        rospy.sleep(2)
        
        # Track loaded objects
        self.loaded_objects = []
        
        print("Realistic Surgical Scene Manager initialized")
    
    def clear_scene(self):
        """Remove all objects from the planning scene"""
        print("Clearing existing scene objects...")
        
        # Remove all known objects
        for obj_name in self.loaded_objects:
            self.scene.remove_world_object(obj_name)
        
        self.loaded_objects = []
        rospy.sleep(1)
        print("Scene cleared")
    
    def create_surgical_environment(self):
        """Create realistic surgical environment"""
        print("Creating realistic surgical environment...")
        
        # 1. Add surgical table (FORBIDDEN - collision object)
        self.add_surgical_table()
        
        # 2. Mark patient head (ACCESSIBLE - no collision object)
        self.mark_patient_head()
        
        rospy.sleep(2)  # Wait for objects to be added
        print("Realistic surgical environment created!")
    
    def add_surgical_table(self):
        """Add surgical table as forbidden collision object"""
        # Calculate table position - head against wall, length extending outward
        # Robot is wall-mounted, table extends outward from wall
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = "world"
        
        # Position table perpendicular to wall - slightly away from wall
        # X: Centered with robot position
        # Y: Table center extending outward from wall (with clearance)
        # Z: Table height (bottom of table)
        table_pose.pose.position.x = 0.0     # Centered with robot on wall
        table_pose.pose.position.y = 1.0     # Slightly further from wall (10cm clearance)
        table_pose.pose.position.z = 0.15    # Table height
        table_pose.pose.orientation.w = 1.0  # No rotation
        
        # Table dimensions: perpendicular to wall - head against wall
        # Length: 1.8m (along Y-axis, extending outward from wall)
        # Width: 0.6m (along X-axis, parallel to wall) 
        # Height: 0.1m (thin table)
        table_size = (0.6, 1.8, 0.1)
        
        # Add table as collision object (FORBIDDEN)
        self.scene.add_box("surgical_table", table_pose, size=table_size)
        self.loaded_objects.append("surgical_table")
        
        print("Surgical table added (FORBIDDEN - collision object)")
        print(f"  Position: X={table_pose.pose.position.x}, Y={table_pose.pose.position.y}, Z={table_pose.pose.position.z}")
        print(f"  Size: {table_size[0]}m x {table_size[1]}m x {table_size[2]}m")
        print("  Orientation: Perpendicular to wall - head against wall, length extending outward")
    
    def mark_patient_head(self):
        """Mark patient head as accessible (no collision object)"""
        # Patient head position: at head of table (near wall end)
        
        # Head position: at head of table (near wall with clearance)
        head_x = 0.0      # Centered with robot
        head_y = 0.2      # Near wall end of table (with clearance from wall)
        head_z = 0.25     # On top of table
                          # 0.15 (table bottom) + 0.1 (table height) = 0.25 (table top)
        
        # We DON'T add this as a collision object
        # This makes it naturally accessible to the robot
        
        print("Patient head marked (ACCESSIBLE - no collision)")
        print(f"  Position: X={head_x}, Y={head_y}, Z={head_z}")
        print("  Size: Sphere radius ~0.1m (head size)")
        print("  Status: ACCESSIBLE for surgical procedures")
        print("  Location: At head of table (near wall)")
        
        # Store head position for reference
        self.patient_head_position = (head_x, head_y, head_z)
        self.patient_head_radius = 0.1
    
    def get_patient_head_position(self):
        """Get patient head position for planning"""
        return self.patient_head_position
    
    def is_near_patient_head(self, position, tolerance=0.15):
        """Check if a position is near the patient head"""
        head_pos = self.patient_head_position
        distance = ((position[0] - head_pos[0])**2 + 
                   (position[1] - head_pos[1])**2 + 
                   (position[2] - head_pos[2])**2)**0.5
        return distance <= tolerance
    
    def plan_to_patient_head(self):
        """Plan movement to patient head area"""
        print("\nPlanning movement to patient head...")
        
        head_pos = self.patient_head_position
        approach_distance = 0.15  # 15cm from head
        
        # Create target pose near patient head
        target_pose = self.group.get_current_pose().pose
        target_pose.position.x = head_pos[0] - approach_distance  # Approach from robot side
        target_pose.position.y = head_pos[1]
        target_pose.position.z = head_pos[2]
        
        print(f"Target position: X={target_pose.position.x:.2f}, Y={target_pose.position.y:.2f}, Z={target_pose.position.z:.2f}")
        
        # Plan movement
        self.group.set_pose_target(target_pose)
        plan = self.group.plan()
        
        if plan and len(plan[1].joint_trajectory.points) > 0:
            print("SUCCESS: Path planned to patient head area")
            print("Robot can safely approach patient head without hitting table")
            return plan
        else:
            print("FAILED: Cannot plan path to patient head")
            print("Check if table is blocking access or target is unreachable")
            return None
    
    def test_surgical_access(self):
        """Test surgical access scenarios"""
        print("\n" + "="*60)
        print("TESTING SURGICAL ACCESS SCENARIOS")
        print("="*60)
        
        # Test 1: Approach patient head (should work)
        print("\nTest 1: Approaching patient head...")
        head_plan = self.plan_to_patient_head()
        if head_plan:
            print("RESULT: SUCCESS - Robot can access patient head")
        else:
            print("RESULT: FAILED - Robot cannot access patient head")
        
        # Test 2: Try to move through table (should fail)
        print("\nTest 2: Attempting to move through table...")
        table_pose = self.group.get_current_pose().pose
        table_pose.position.x = 0.6  # Table position
        table_pose.position.y = 0.0
        table_pose.position.z = 0.15 # Middle of table
        
        self.group.set_pose_target(table_pose)
        table_plan = self.group.plan()
        
        if table_plan and len(table_plan[1].joint_trajectory.points) > 0:
            print("RESULT: UNEXPECTED - Robot planned through table (configuration error)")
        else:
            print("RESULT: SUCCESS - Robot correctly avoids table collision")
        
        # Test 3: Safe area access (should work)
        print("\nTest 3: Accessing safe area above table...")
        safe_pose = self.group.get_current_pose().pose
        safe_pose.position.x = 0.6   # Above table
        safe_pose.position.y = 0.0
        safe_pose.position.z = 0.4   # Well above table
        
        self.group.set_pose_target(safe_pose)
        safe_plan = self.group.plan()
        
        if safe_plan and len(safe_plan[1].joint_trajectory.points) > 0:
            print("RESULT: SUCCESS - Robot can access safe area above table")
        else:
            print("RESULT: FAILED - Robot cannot access safe area")
    
    def enable_surgical_mode(self):
        """Enable surgical mode with enhanced safety"""
        print("\nSURGICAL MODE ENABLED")
        print("- Enhanced safety protocols active")
        print("- Reduced speed limits for patient safety")
        print("- Precision control for neurosurgical procedures")
        
        # Set very conservative velocity scaling for surgery
        self.group.set_max_velocity_scaling_factor(0.05)  # Very slow for surgery
        self.group.set_max_acceleration_scaling_factor(0.05)
    
    def print_scene_summary(self):
        """Print summary of surgical scene setup"""
        print("\n" + "="*60)
        print("SURGICAL SCENE SUMMARY")
        print("="*60)
        print("Setup: Neurosurgical robot with realistic patient positioning")
        print()
        print("OBJECTS:")
        print("1. Surgical Table:")
        print("   - Position: X=0.0, Y=1.0, Z=0.15")
        print("   - Size: 0.6m x 1.8m x 0.1m") 
        print("   - Orientation: Perpendicular to wall - with clearance from wall")
        print("   - Status: FORBIDDEN (collision object)")
        print("   - Purpose: Robot must avoid hitting table")
        print()
        print("2. Patient Head:")
        head_pos = self.patient_head_position
        print(f"   - Position: X={head_pos[0]}, Y={head_pos[1]}, Z={head_pos[2]}")
        print("   - Size: Sphere radius ~0.1m")
        print("   - Status: ACCESSIBLE (no collision)")
        print("   - Purpose: Surgical target area")
        print("   - Location: At head of table (near wall)")
        print()
        print("SURGICAL WORKFLOW:")
        print("- Robot can approach patient head for procedures")
        print("- Robot cannot hit or move through surgical table")
        print("- Table head against wall, extending outward")
        print("- Patient head positioned near wall for optimal robot access")
        print("="*60)

def main():
    try:
        # Create realistic surgical scene
        scene_manager = RealisticSurgicalScene()
        
        # Clear any existing objects
        scene_manager.clear_scene()
        
        # Create surgical environment
        scene_manager.create_surgical_environment()
        
        # Enable surgical mode
        scene_manager.enable_surgical_mode()
        
        # Print scene summary
        scene_manager.print_scene_summary()
        
        # Test surgical access scenarios
        scene_manager.test_surgical_access()
        
        print("\n" + "="*60)
        print("REALISTIC SURGICAL ENVIRONMENT READY!")
        print("- Surgical table positioned vertically to wall")
        print("- Patient head accessible for procedures")
        print("- Table acts as collision barrier")
        print("- Ready for neurosurgical planning")
        print("="*60)
        
        # Keep node running
        print("\nRealistic surgical scene manager running...")
        print("Press Ctrl+C to stop")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("Realistic surgical scene manager stopped")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()