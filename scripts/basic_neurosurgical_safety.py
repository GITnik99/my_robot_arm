#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import math

class OperatingTable:
    def __init__(self):
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('operating_table', anonymous=True)
        
        # Initialize the planning scene interface
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Wait for the scene to be ready
        rospy.sleep(2.0)
        
        rospy.loginfo("Creating operating table...")
        
        # Create operating table
        self.create_operating_table()
        
        rospy.loginfo("Operating table created successfully")

    def create_operating_table(self):
        """Create realistic operating table positioned beside the robot"""
        
        # Operating table main surface
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.8  # Much further away from robot
        table_pose.pose.position.y = 0.0  # Centered
        table_pose.pose.position.z = 0.15  # Table height
        
        # Rotate 90 degrees around Z-axis (yaw rotation)
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.707  # sin(90°/2)
        table_pose.pose.orientation.w = 0.707  # cos(90°/2)
        
        # Realistic operating table dimensions: 1.5m x 0.4m x 0.1m
        # After rotation: length becomes Y-direction, width becomes X-direction
        table_dimensions = (0.4, 1.5, 0.1)  # much narrower width
        
        self.scene.add_box("operating_table", table_pose, size=table_dimensions)
        
        rospy.loginfo("Operating table positioned in front of robot")
        rospy.loginfo("Table dimensions: 1.5m x 0.4m x 0.1m")
        rospy.loginfo("Table position: (0.8, 0.0, 0.15)")
        rospy.loginfo("Robot base should now be completely clear")
        
        # Add target positions above the table for robot planning
        self.add_table_targets()
        
        # Add realistic patient head
        self.add_patient_head()
    
    def add_table_targets(self):
        """Add target positions above the operating table"""
        
        # Target position above center of table
        center_target = geometry_msgs.msg.PoseStamped()
        center_target.header.frame_id = "base_link"
        center_target.pose.position.x = 0.8  # Above table center
        center_target.pose.position.y = 0.0
        center_target.pose.pose.position.z = 0.35  # 20cm above table surface
        center_target.pose.orientation.w = 1.0
        
        # Small sphere to mark the target
        self.scene.add_sphere("table_center_target", center_target, radius=0.02)
        
        # Target position above near edge of table
        near_target = geometry_msgs.msg.PoseStamped()
        near_target.header.frame_id = "base_link"
        near_target.pose.position.x = 0.6  # Near edge of table
        near_target.pose.position.y = 0.0
        near_target.pose.position.z = 0.35
        near_target.pose.orientation.w = 1.0
        
        self.scene.add_sphere("table_near_target", near_target, radius=0.02)
        
        # Target position above far edge of table
        far_target = geometry_msgs.msg.PoseStamped()
        far_target.header.frame_id = "base_link"
        far_target.pose.position.x = 1.0  # Far edge of table
        far_target.pose.position.y = 0.0
        far_target.pose.position.z = 0.35
        far_target.pose.orientation.w = 1.0
        
        self.scene.add_sphere("table_far_target", far_target, radius=0.02)
        
        rospy.loginfo("Added target positions above operating table")
        rospy.loginfo("- table_center_target: (0.8, 0.0, 0.35)")
        rospy.loginfo("- table_near_target: (0.6, 0.0, 0.35)")  
        rospy.loginfo("- table_far_target: (1.0, 0.0, 0.35)")
        rospy.loginfo("Try planning to these positions in RViz")
    
    def add_patient_head(self):
        """Add realistic patient head positioned on the operating table"""
        
        # Patient head positioned on table surface
        head_pose = geometry_msgs.msg.PoseStamped()
        head_pose.header.frame_id = "base_link"
        head_pose.pose.position.x = 0.8  # Center of table
        head_pose.pose.position.y = 0.0  # Centered on table
        head_pose.pose.position.z = 0.25  # On table surface (0.15 + 0.05 + 0.05 for head radius)
        head_pose.pose.orientation.w = 1.0
        
        # Adult head radius approximately 9.5cm
        head_radius = 0.095
        
        self.scene.add_sphere("patient_head", head_pose, radius=head_radius)
        
        rospy.loginfo("Added patient head on operating table")
        rospy.loginfo("Head position: (0.8, 0.0, 0.25)")
        rospy.loginfo("Head radius: 9.5cm")
        rospy.loginfo("Patient head is now a collision object - robot must avoid it")

if __name__ == '__main__':
    try:
        table = OperatingTable()
        rospy.loginfo("Operating table system active")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass