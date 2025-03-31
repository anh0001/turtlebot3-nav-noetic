#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import math

class PatrolRobot:
    def __init__(self):
        rospy.init_node('turtlebot_patrol')
        
        # Create action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Define patrol points (x, y coordinates)
        self.patrol_points = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]
        
        self.current_point = 0
        self.total_points = len(self.patrol_points)
        
        # Start patrolling
        self.patrol()
    
    def create_goal(self, x, y):
        """Create a MoveBaseGoal with the given x, y coordinates"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        return goal
    
    def patrol(self):
        """Main patrol loop"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # Get the next patrol point
            x, y = self.patrol_points[self.current_point]
            
            # Create and send goal
            goal = self.create_goal(x, y)
            rospy.loginfo(f"Sending goal: Point {self.current_point + 1}/{self.total_points} ({x}, {y})")
            self.client.send_goal(goal)
            
            # Wait for result
            self.client.wait_for_result()
            
            # Get the result
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached patrol point {self.current_point + 1}")
                # Move to next point
                self.current_point = (self.current_point + 1) % self.total_points
            else:
                rospy.logwarn(f"Failed to reach patrol point {self.current_point + 1}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        PatrolRobot()
    except rospy.ROSInterruptException:
        pass
