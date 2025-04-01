#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, sqrt, pi
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import actionlib
import sys

class TurtlebotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        
        # Parameters (can be set from launch file)
        self.use_move_base = rospy.get_param('~use_move_base', True)  # Use move_base by default
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)  # Meters
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # Meters
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s
        
        # Status variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.laser_ranges = []
        self.min_distance = float('inf')
        self.obstacle_direction = 0.0
        self.goal_reached = False
        self.navigation_active = False
        
        # TF Listener for transformations
        self.tf_listener = tf.TransformListener()
        
        # Create move_base client
        if self.use_move_base:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            server_available = self.move_base_client.wait_for_server(rospy.Duration(5.0))
            if not server_available:
                rospy.logwarn("move_base action server not available, falling back to manual control")
                self.use_move_base = False
            else:
                rospy.loginfo("Connected to move_base action server")
                
                # Subscribe to move_base result to handle failures
                self.move_base_result_sub = rospy.Subscriber(
                    '/move_base/result',
                    MoveBaseActionResult,
                    self.move_base_result_callback
                )
        
        # Create velocity publisher for manual control
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Set rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Define patrol goals in map frame
        # These should be adjusted to match your map
        self.goals = [
            {'x': 0.5, 'y': 0.0, 'yaw': 0.0},
            {'x': 1.5, 'y': -0.5, 'yaw': pi/2},
            {'x': -0.5, 'y': 1.5, 'yaw': pi},
            {'x': 0.5, 'y': -1.5, 'yaw': -pi/2}
        ]
        self.current_goal_index = 0
        
        rospy.loginfo("Turtlebot Navigator initialized. Use move_base: %s", self.use_move_base)
        
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
        
        # Replace NaN and inf values with a large number
        self.laser_ranges[np.isnan(self.laser_ranges)] = 10.0
        self.laser_ranges[np.isinf(self.laser_ranges)] = 10.0
        
        # Find the minimum distance and its direction
        if len(self.laser_ranges) > 0:
            self.min_distance = np.min(self.laser_ranges)
            min_idx = np.argmin(self.laser_ranges)
            self.obstacle_direction = min_idx * msg.angle_increment + msg.angle_min

    def move_base_result_callback(self, msg):
        result_status = msg.status.status
        if result_status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            self.goal_reached = True
        elif result_status == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("Goal aborted! Trying to recover...")
            # You could implement recovery behavior here
            self.goal_reached = True  # Mark as reached to move to the next goal
        elif result_status == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("Goal rejected! Trying to recover...")
            self.goal_reached = True  # Mark as reached to move to the next goal

    def create_move_base_goal(self, x, y, yaw):
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

    def send_goal(self, x, y, yaw):
        if not self.use_move_base:
            rospy.logwarn("move_base is not enabled. Cannot send goal.")
            return
            
        goal = self.create_move_base_goal(x, y, yaw)
        self.goal_reached = False
        
        rospy.loginfo(f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        self.move_base_client.send_goal(goal)
        self.navigation_active = True

    def navigate(self):
        """Main navigation method using move_base"""
        rospy.loginfo("Starting navigation using move_base...")
        
        # Send the first goal
        current_goal = self.goals[self.current_goal_index]
        self.send_goal(current_goal['x'], current_goal['y'], current_goal['yaw'])
        
        while not rospy.is_shutdown():
            if self.goal_reached:
                # Move to the next goal
                self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
                current_goal = self.goals[self.current_goal_index]
                self.send_goal(current_goal['x'], current_goal['y'], current_goal['yaw'])
            
            # Show current status
            if self.navigation_active:
                distance_to_obstacle = self.min_distance if self.min_distance < 10.0 else "None"
                rospy.loginfo(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                            f"Yaw: {self.orientation['yaw']:.2f}, "
                            f"Nearest obstacle: {distance_to_obstacle}")
            
            self.rate.sleep()

    def manual_obstacle_avoidance(self):
        """
        Manual obstacle avoidance method using raw velocity commands.
        This is a backup in case move_base is not working properly.
        """
        rospy.loginfo("Starting manual obstacle avoidance...")
        
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            
            # Get current goal
            current_goal = self.goals[self.current_goal_index]
            goal_x, goal_y = current_goal['x'], current_goal['y']
            
            # Calculate distance to goal
            distance_to_goal = sqrt((goal_x - self.position['x'])**2 + 
                                   (goal_y - self.position['y'])**2)
            
            # Calculate angle to goal
            angle_to_goal = atan2(goal_y - self.position['y'], 
                                 goal_x - self.position['x'])
            
            # Calculate angle difference
            angle_diff = angle_to_goal - self.orientation['yaw']
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > pi:
                angle_diff -= 2 * pi
            while angle_diff < -pi:
                angle_diff += 2 * pi
            
            if distance_to_goal < self.goal_tolerance:
                # Goal reached, move to next goal
                rospy.loginfo(f"Goal {self.current_goal_index + 1} reached!")
                self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
                
                # Brief stop
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.sleep(1.0)  # Pause for 1 second
                
            elif self.min_distance < self.obstacle_threshold:
                # Obstacle detected, avoid it
                rospy.loginfo("Obstacle detected! Avoiding...")
                
                # Determine turn direction based on the obstacle location
                if self.obstacle_direction < 0:
                    # Obstacle on the left, turn right
                    cmd_vel.angular.z = -self.angular_speed
                else:
                    # Obstacle on the right, turn left
                    cmd_vel.angular.z = self.angular_speed
                
                # Slow forward movement
                cmd_vel.linear.x = 0.1
                
            else:
                # No obstacle, head towards the goal
                if abs(angle_diff) > 0.2:
                    # Turn towards goal
                    cmd_vel.angular.z = 0.3 * angle_diff
                    # Limit angular velocity
                    cmd_vel.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd_vel.angular.z))
                    cmd_vel.linear.x = 0.1
                else:
                    # Move forward
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = 0.1 * angle_diff  # Small correction
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Log status
            rospy.loginfo(f"Goal: ({goal_x:.2f}, {goal_y:.2f}), "
                         f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                         f"Distance: {distance_to_goal:.2f}, "
                         f"Angle diff: {angle_diff:.2f}, "
                         f"Obstacle dist: {self.min_distance:.2f}")
            
            self.rate.sleep()

    def run(self):
        """Main entry point that decides which navigation method to use"""
        try:
            if self.use_move_base:
                self.navigate()
            else:
                self.manual_obstacle_avoidance()
        except Exception as e:
            rospy.logerr(f"Navigation error: {e}")
            # Emergency stop
            self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        navigator = TurtlebotNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass