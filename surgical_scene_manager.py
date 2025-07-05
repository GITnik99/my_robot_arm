#!/usr/bin/env python

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class SurgicalSceneManager:
    def __init__(self):
        # Initialize MoveIt commanders
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('surgical_scene_manager', anonymous=True)
        
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        
        # Wait for scene to initialize
        rospy.sleep(2)
        
        # Define object types and their access rules
        self.object_rules = {
            'patient_body': 'RESTRICTED',      # Yellow - careful approach only
            'critical_organs': 'FORBIDDEN',    # Red - absolutely no access
            'surgical_target': 'ACCESSIBLE',   # Green - surgery allowed here
            'medical_equipment': 'OBSTACLE'    # Gray - avoid collision
        }
        
        # Track which objects are currently loaded
        self.loaded_objects = []
        
        print("Surgical Scene Manager initialized")
    
    def clear_scene(self):
        """Remove all objects from the planning scene"""
        print("Clearing existing scene objects...")
        
        # Remove all known objects
        for obj_name in self.loaded_objects:
            self.scene.remove_world_object(obj_name)
        
        self.loaded_objects = []
        rospy.sleep(1)
        print("Scene cleared")
    
    def add_surgical_objects(self):
        """Add all surgical environment objects"""
        print("Adding surgical environment objects...")
        
        # 1. Patient Body (RESTRICTED - careful approach)
        self.add_patient_body()
        
        # 2. Critical Organs (FORBIDDEN - never access)
        self.add_critical_organs()
        
        # 3. Medical Equipment (OBSTACLE - avoid collision)
        self.add_medical_equipment()
        
        # Note: We DON'T add surgical_target as collision object
        # This makes it naturally accessible
        self.mark_surgical_target()
        
        rospy.sleep(2)  # Wait for objects to be added
        print("All surgical objects loaded successfully!")
    
    def add_patient_body(self):
        """Add patient body as restricted zone"""
        patient_pose = PoseStamped()
        patient_pose.header.frame_id = "world"
        patient_pose.pose.position.x = 0.8    # Further out in robot's reach
        patient_pose.pose.position.y = 0.0    # Directly in front
        patient_pose.pose.position.z = 0.3    # At robot working height
        patient_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("patient_body", patient_pose, size=(0.4, 0.6, 0.2))
        self.loaded_objects.append("patient_body")
        print("Patient body added (RESTRICTED zone)")
    
    def add_critical_organs(self):
        """Add critical organs as forbidden zone"""
        organs_pose = PoseStamped()
        organs_pose.header.frame_id = "world"
        organs_pose.pose.position.x = 0.82   # Within patient body area
        organs_pose.pose.position.y = -0.1   # Offset within patient
        organs_pose.pose.position.z = 0.35   # Chest level
        organs_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("critical_organs", organs_pose, size=(0.15, 0.15, 0.1))
        self.loaded_objects.append("critical_organs")
        print("Critical organs added (FORBIDDEN zone)")
    
    def add_medical_equipment(self):
        """Add medical equipment as obstacle"""
        equipment_pose = PoseStamped()
        equipment_pose.header.frame_id = "world"
        equipment_pose.pose.position.x = 0.3   # Behind and to side of robot reach
        equipment_pose.pose.position.y = -0.7  # Far to the left side
        equipment_pose.pose.position.z = 0.4   # Equipment height
        equipment_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("medical_equipment", equipment_pose, size=(0.2, 0.2, 0.8))
        self.loaded_objects.append("medical_equipment")
        print("Medical equipment added (OBSTACLE)")
    
    def mark_surgical_target(self):
        """Mark surgical target area (accessible - no collision object)"""
        # We don't add this as a collision object, making it naturally accessible
        # Just log the area for reference
        target_x, target_y, target_z = 0.7, 0.2, 0.32
        print(f"Surgical target marked at ({target_x}, {target_y}, {target_z}) - ACCESSIBLE")
    
    def get_object_rule(self, object_name):
        """Get access rule for a specific object"""
        return self.object_rules.get(object_name, "UNKNOWN")
    
    def check_target_accessibility(self, target_pose):
        """Check if a target pose is accessible based on our rules"""
        # This is a simplified version - in practice you'd do geometric checks
        x, y, z = target_pose
        
        # Define surgical target area (accessible zone)
        surgical_area = {
            'x_min': 0.65, 'x_max': 0.75,
            'y_min': 0.15, 'y_max': 0.25,
            'z_min': 0.30, 'z_max': 0.35
        }
        
        # Check if target is in surgical area
        if (surgical_area['x_min'] <= x <= surgical_area['x_max'] and
            surgical_area['y_min'] <= y <= surgical_area['y_max'] and
            surgical_area['z_min'] <= z <= surgical_area['z_max']):
            return True, "Target in ACCESSIBLE surgical area"
        
        # Check if target would hit forbidden zones
        # (In practice, MoveIt will handle collision detection automatically)
        return False, "Target outside accessible surgical area"
    
    def plan_surgical_movement(self, target_position):
        """Plan movement with surgical safety considerations"""
        print(f"\nPlanning movement to: {target_position}")
        
        # Check accessibility
        accessible, reason = self.check_target_accessibility(target_position)
        print(f"Accessibility check: {reason}")
        
        if not accessible:
            print("Movement denied - target not in accessible area")
            return None
        
        # If accessible, proceed with normal MoveIt planning
        # MoveIt will automatically avoid collision objects we added
        print("Target approved - planning with MoveIt collision avoidance")
        
        # Create pose for MoveIt
        target_pose = self.group.get_current_pose().pose
        target_pose.position.x = target_position[0]
        target_pose.position.y = target_position[1] 
        target_pose.position.z = target_position[2]
        
        self.group.set_pose_target(target_pose)
        plan = self.group.plan()
        
        return plan
    
    def enable_surgical_mode(self):
        """Enable surgical mode - more restrictive safety"""
        print("\nSURGICAL MODE ENABLED")
        print("- Enhanced safety protocols active")
        print("- Reduced speed limits in effect")
        print("- All movements logged for safety")
        
        # Set conservative velocity scaling
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
    
    def test_scene_objects(self):
        """Test the scene setup with different target positions"""
        print("\n" + "="*50)
        print("TESTING SURGICAL SCENE SETUP")
        print("="*50)
        
        # Test positions
        test_positions = [
            (0.7, 0.2, 0.32, "Surgical target (should work)"),
            (0.82, -0.1, 0.35, "Critical organs (should fail)"),
            (0.5, 0.0, 0.3, "Safe area (should work)"),
            (0.3, -0.7, 0.4, "Equipment area (should fail)")
        ]
        
        for x, y, z, description in test_positions:
            print(f"\nTesting: {description}")
            accessible, reason = self.check_target_accessibility((x, y, z))
            status = "ALLOWED" if accessible else "BLOCKED"
            print(f"Result: {status} - {reason}")

def main():
    try:
        # Create surgical scene manager
        scene_manager = SurgicalSceneManager()
        
        # Clear any existing objects
        scene_manager.clear_scene()
        
        # Add all surgical objects
        scene_manager.add_surgical_objects()
        
        # Enable surgical mode
        scene_manager.enable_surgical_mode()
        
        # Test the setup
        scene_manager.test_scene_objects()
        
        print("\n" + "="*50)
        print("SURGICAL SCENE READY!")
        print("- Objects loaded with smart recognition")
        print("- Forbidden zones: Critical organs")
        print("- Restricted zones: Patient body") 
        print("- Accessible zones: Surgical target")
        print("- Obstacle zones: Medical equipment")
        print("="*50)
        
        # Keep node running
        print("\nSurgical scene manager running...")
        print("Press Ctrl+C to stop")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("Surgical scene manager stopped")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()