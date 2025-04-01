#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, sqrt, pi, cos, sin
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import actionlib
from actionlib_msgs.msg import GoalStatus

class SimpleTurtlebotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        
        # Get parameters from the parameter server
        self.use_move_base = rospy.get_param('~use_move_base', True)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # Increased for better responsiveness
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed = rospy.get_param('~angular_speed', 0.6)
        
        # Status variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.laser_ranges = []
        self.is_obstacle_front = False
        self.is_obstacle_left = False
        self.is_obstacle_right = False
        self.goal_reached = False
        self.navigation_active = False
        self.goals_completed = False
        self.turning = False
        self.turn_direction = 1  # 1 for left, -1 for right
        
        # TF Listener for transformations
        self.tf_listener = tf.TransformListener()
        
        # Create move_base client if enabled
        if self.use_move_base:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            server_available = self.move_base_client.wait_for_server(rospy.Duration(5.0))
            if not server_available:
                rospy.logwarn("move_base action server not available, falling back to manual control")
                self.use_move_base = False
            else:
                rospy.loginfo("Connected to move_base action server")
                
                # Subscribe to move_base result
                self.move_base_result_sub = rospy.Subscriber(
                    '/move_base/result',
                    MoveBaseActionResult,
                    self.move_base_result_callback
                )
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Set rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Define predefined goals in map frame
        self.goals = [
            {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
            {'x': 1.5, 'y': -0.5, 'yaw': pi/2},
            {'x': -0.5, 'y': 1.5, 'yaw': pi},
            {'x': 0.5, 'y': -1.5, 'yaw': -pi/2}
        ]
        self.current_goal_index = 0
        
        rospy.loginfo("Simple Turtlebot Navigator initialized. Use move_base: %s", self.use_move_base)
        
    def odom_callback(self, msg):
        # Extract position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.orientation['roll'], self.orientation['pitch'], self.orientation['yaw']) = euler_from_quaternion(orientation_list)

    def scan_callback(self, msg):
        # Process laser scan data - simplified to just check front, left, right
        self.laser_ranges = np.array(msg.ranges)
        
        # Replace NaN and inf values with a large number
        self.laser_ranges[np.isnan(self.laser_ranges)] = 10.0
        self.laser_ranges[np.isinf(self.laser_ranges)] = 10.0
        
        # Simple sector division - front, left, right
        num_points = len(self.laser_ranges)
        
        if num_points > 0:
            # Define sectors (assuming 360 degree scan)
            front_indices = list(range(num_points - 45, num_points)) + list(range(0, 45))
            left_indices = list(range(45, 135))
            right_indices = list(range(num_points - 135, num_points - 45))
            
            # Get minimum distance in each sector
            front_dist = np.min(self.laser_ranges[front_indices]) if front_indices else 10.0
            left_dist = np.min(self.laser_ranges[left_indices]) if left_indices else 10.0
            right_dist = np.min(self.laser_ranges[right_indices]) if right_indices else 10.0
            
            # Check if obstacles are present
            self.is_obstacle_front = front_dist < self.obstacle_threshold
            self.is_obstacle_left = left_dist < self.obstacle_threshold
            self.is_obstacle_right = right_dist < self.obstacle_threshold
            
            # Simple logging of obstacle detection
            if self.is_obstacle_front or self.is_obstacle_left or self.is_obstacle_right:
                rospy.logdebug(f"Obstacles: Front={self.is_obstacle_front}, Left={self.is_obstacle_left}, Right={self.is_obstacle_right}")
                rospy.logdebug(f"Distances: Front={front_dist:.2f}, Left={left_dist:.2f}, Right={right_dist:.2f}")

    def move_base_result_callback(self, msg):
        result_status = msg.status.status
        if result_status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            self.goal_reached = True
        elif result_status == GoalStatus.ABORTED:
            rospy.logwarn("Goal aborted! Moving to next goal.")
            self.goal_reached = True
        elif result_status == GoalStatus.REJECTED:
            rospy.logwarn("Goal rejected! Moving to next goal.")
            self.goal_reached = True

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
                self.current_goal_index += 1
                
                # Check if all goals have been reached
                if self.current_goal_index >= len(self.goals):
                    rospy.loginfo("All goals have been reached! Navigation complete.")
                    self.goals_completed = True
                    # Stop the robot
                    self.cmd_vel_pub.publish(Twist())
                    if self.navigation_active:
                        self.move_base_client.cancel_all_goals()
                        self.navigation_active = False
                    break
                else:
                    # Send the next goal
                    current_goal = self.goals[self.current_goal_index]
                    self.send_goal(current_goal['x'], current_goal['y'], current_goal['yaw'])
            
            # Show current status
            if self.navigation_active:
                rospy.loginfo(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), Yaw: {self.orientation['yaw']:.2f}")
            
            self.rate.sleep()
        
        rospy.loginfo("Navigation complete. Node will remain active but idle.")
        rospy.spin()

    def manual_obstacle_avoidance(self):
        """Simple manual obstacle avoidance: move straight when clear, turn when obstacles detected"""
        rospy.loginfo("Starting simple manual obstacle avoidance...")
        
        # State for the current goal
        current_goal_index = 0
        current_goal = self.goals[current_goal_index]
        heading_to_goal = False
        
        while not rospy.is_shutdown():
            # Create empty velocity command
            cmd_vel = Twist()
            
            # Get current goal
            current_goal = self.goals[current_goal_index]
            goal_x, goal_y = current_goal['x'], current_goal['y']
            
            # Calculate distance to goal
            distance_to_goal = sqrt((goal_x - self.position['x'])**2 + 
                                   (goal_y - self.position['y'])**2)
            
            # Calculate angle to goal
            angle_to_goal = atan2(goal_y - self.position['y'], 
                                 goal_x - self.position['x'])
            
            # Calculate angle difference (how much we need to turn to face the goal)
            angle_diff = self.normalize_angle(angle_to_goal - self.orientation['yaw'])
            
            # Check if goal is reached
            if distance_to_goal < self.goal_tolerance and not self.turning:
                rospy.loginfo(f"Goal {current_goal_index + 1} reached! Distance: {distance_to_goal:.2f}m")
                current_goal_index = (current_goal_index + 1) % len(self.goals)
                heading_to_goal = False
                
                # Stop briefly at goal
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.sleep(1.0)
                continue
            
            # Simple obstacle avoidance logic
            if self.is_obstacle_front:
                # Obstacle in front - stop and turn
                cmd_vel.linear.x = 0.0
                
                # If we weren't turning before, decide which way to turn
                # Choose the side with more space
                if not self.turning:
                    self.turning = True
                    if self.is_obstacle_left and not self.is_obstacle_right:
                        self.turn_direction = -1  # Turn right
                        rospy.loginfo("Obstacle ahead! Turning right.")
                    elif self.is_obstacle_right and not self.is_obstacle_left:
                        self.turn_direction = 1   # Turn left
                        rospy.loginfo("Obstacle ahead! Turning left.")
                    else:
                        # If both sides have obstacles or are clear, choose randomly
                        # but with preference to the side closest to goal direction
                        if angle_diff > 0:
                            self.turn_direction = 1  # Turn left
                        else:
                            self.turn_direction = -1  # Turn right
                        rospy.loginfo(f"Obstacle ahead! Turning {'left' if self.turn_direction > 0 else 'right'}.")
                
                # Apply the turn
                cmd_vel.angular.z = self.angular_speed * self.turn_direction
            
            elif self.turning:
                # Continue turning until path is clear
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.angular_speed * self.turn_direction
                
                # Check if we've cleared the obstacle
                if not (self.is_obstacle_front or 
                       (self.turn_direction > 0 and self.is_obstacle_left) or 
                       (self.turn_direction < 0 and self.is_obstacle_right)):
                    self.turning = False
                    heading_to_goal = False
                    rospy.loginfo("Path clear! Resuming navigation.")
            
            elif not heading_to_goal:
                # Align with goal direction before moving forward
                if abs(angle_diff) > 0.3:  # If we're not facing the goal
                    cmd_vel.linear.x = 0.0
                    # Turn towards the goal
                    cmd_vel.angular.z = 0.5 * angle_diff
                    rospy.loginfo(f"Aligning to goal: angle_diff={angle_diff:.2f}")
                else:
                    heading_to_goal = True
                    rospy.loginfo("Aligned with goal, moving forward.")
            
            else:
                # Path is clear and we're aligned with goal - move forward
                cmd_vel.linear.x = self.linear_speed
                
                # Small correction to keep on course
                cmd_vel.angular.z = 0.3 * angle_diff
                
                # Check for obstacles on sides - slight adjustment to avoid them
                if self.is_obstacle_left:
                    cmd_vel.angular.z -= 0.2  # Slight right turn
                elif self.is_obstacle_right:
                    cmd_vel.angular.z += 0.2  # Slight left turn
            
            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Log status 
            rospy.loginfo(f"Goal: ({goal_x:.2f}, {goal_y:.2f}), Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), Distance: {distance_to_goal:.2f}m")
            
            self.rate.sleep()
    
    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

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
        navigator = SimpleTurtlebotNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass