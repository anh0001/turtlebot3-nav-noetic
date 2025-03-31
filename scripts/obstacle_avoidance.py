#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class TurtlebotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        
        # Initialize variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.laser_ranges = []
        self.min_distance = float('inf')
        self.obstacle_direction = None
        
        # Create move_base client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # Create velocity publisher for manual control if needed
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Set rate
        self.rate = rospy.Rate(10)
        
        # Goals
        self.goals = [
            {'x': 1.0, 'y': 0.0},
            {'x': 1.0, 'y': 1.0},
            {'x': 0.0, 'y': 1.0},
            {'x': 0.0, 'y': 0.0}
        ]
        self.current_goal_index = 0
        
        rospy.loginfo("Turtlebot Navigator initialized")
        
    def odom_callback(self, msg):
        # Extract position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.orientation['roll'], self.orientation['pitch'], self.orientation['yaw']) = euler_from_quaternion(orientation_list)

    def scan_callback(self, msg):
        # Process laser scan data
        self.laser_ranges = np.array(msg.ranges)
        self.laser_ranges[np.isinf(self.laser_ranges)] = 10.0  # Replace inf with 10.0
        self.min_distance = np.min(self.laser_ranges)
        self.obstacle_direction = np.argmin(self.laser_ranges) * msg.angle_increment + msg.angle_min

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"Sending goal: x={x}, y={y}")
        self.move_base_client.send_goal(goal)

    def navigate(self):
        while not rospy.is_shutdown():
            # Check if the robot reached the current goal
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
                self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
                current_goal = self.goals[self.current_goal_index]
                self.send_goal(current_goal['x'], current_goal['y'])
            elif state == actionlib.GoalStatus.ACTIVE:
                rospy.loginfo(f"Navigating to goal... Current position: x={self.position['x']:.2f}, y={self.position['y']:.2f}, Min distance to obstacle: {self.min_distance:.2f}")
            else:
                # If no goal is active, send the first goal
                if self.current_goal_index == 0 and state != actionlib.GoalStatus.PENDING:
                    current_goal = self.goals[self.current_goal_index]
                    self.send_goal(current_goal['x'], current_goal['y'])
            
            self.rate.sleep()

    def manual_obstacle_avoidance(self):
        """
        Manual obstacle avoidance method using raw velocity commands.
        This is a backup in case move_base is not working properly.
        """
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            
            if self.min_distance < 0.5:  # If obstacle is closer than 0.5 meters
                # Calculate avoidance direction (turn away from obstacle)
                if self.obstacle_direction < 0:  # Obstacle on the left
                    cmd_vel.angular.z = -0.5  # Turn right
                else:  # Obstacle on the right
                    cmd_vel.angular.z = 0.5   # Turn left
                cmd_vel.linear.x = 0.1
            else:
                # No obstacle, go forward
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(f"Min distance: {self.min_distance}, Direction: {self.obstacle_direction}, Linear: {cmd_vel.linear.x}, Angular: {cmd_vel.angular.z}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = TurtlebotNavigator()
        # Use navigate() for move_base navigation
        navigator.navigate()
        # Or use manual_obstacle_avoidance() for direct velocity control
        # navigator.manual_obstacle_avoidance()
    except rospy.ROSInterruptException:
        pass