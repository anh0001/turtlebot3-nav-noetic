#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

class PatrolRobot:
    def __init__(self):
        rospy.init_node('turtlebot_patrol')
        
        # Parameters
        self.loop_forever = rospy.get_param('~loop_forever', True)
        self.wait_time = rospy.get_param('~wait_time', 1.0)  # Time to wait between goals in seconds
        
        # Create action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait_result = self.client.wait_for_server(rospy.Duration(10.0))
        if not wait_result:
            rospy.logerr("Could not connect to move_base action server! Make sure navigation is running.")
            rospy.signal_shutdown("No move_base action server")
            return
        rospy.loginfo("Connected to move_base action server")
        
        # Define patrol points with orientation (x, y, yaw)
        self.patrol_points = [
            {'x': -1.5, 'y': 1.5, 'yaw': 0.0},         # Forward
            {'x': 1.5, 'y': -1.5, 'yaw': -math.pi/2},  # -90 degrees (facing right)
            {'x': 1.5, 'y': 1.5, 'yaw': math.pi},     # 180 degrees (backward)
            {'x': -1.5, 'y': -1.5, 'yaw': math.pi/2}  # 90 degrees (facing left)
        ]
        
        self.current_point = 0
        self.total_points = len(self.patrol_points)
        self.completed_loops = 0
        
        # Start patrolling
        self.patrol()
    
    def create_goal(self, x, y, yaw):
        """Create a MoveBaseGoal with the given x, y coordinates and yaw orientation"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Set orientation from yaw
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        return goal
    
    def patrol(self):
        """Main patrol loop"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # Get the next patrol point
            point = self.patrol_points[self.current_point]
            x, y, yaw = point['x'], point['y'], point['yaw']
            
            # Create and send goal
            goal = self.create_goal(x, y, yaw)
            rospy.loginfo(f"Sending goal: Point {self.current_point + 1}/{self.total_points} ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
            self.client.send_goal(goal)
            
            # Wait for result
            self.client.wait_for_result()
            
            # Get the result
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached patrol point {self.current_point + 1}")
                # Move to next point
                self.current_point = (self.current_point + 1) % self.total_points
                
                # If we've completed a loop, increment the counter
                if self.current_point == 0:
                    self.completed_loops += 1
                    rospy.loginfo(f"Completed patrol loop {self.completed_loops}")
                    
                    # If we're not set to loop forever and we've done at least one loop, stop
                    if not self.loop_forever and self.completed_loops >= 1:
                        rospy.loginfo("Completed requested number of loops. Stopping patrol.")
                        break
                
                # Wait a bit before moving to the next goal
                rospy.sleep(self.wait_time)
            else:
                rospy.logwarn(f"Failed to reach patrol point {self.current_point + 1}. Status: {state}")
                # We'll still try to move to the next point even if this one failed
                self.current_point = (self.current_point + 1) % self.total_points
                rospy.sleep(self.wait_time)
            
            rate.sleep()
        
        rospy.loginfo("Patrol complete or interrupted.")

if __name__ == '__main__':
    try:
        PatrolRobot()
    except rospy.ROSInterruptException:
        pass