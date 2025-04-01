#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, sqrt, pi, cos, sin
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import actionlib
import sys
import random
import time

class TurtlebotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        
        # Parameters (can be set from launch file)
        self.use_move_base = rospy.get_param('~use_move_base', True)  # Use move_base by default
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)  # Meters
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.3)  # Meters
        self.danger_threshold = rospy.get_param('~danger_threshold', 0.4)
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.6)  # rad/s - increased from 0.5
        self.scan_angle_front = rospy.get_param('~scan_angle_front', pi/2)  # Increased from pi/3 (90 degrees now)
        
        # Status variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.laser_ranges = []
        self.min_distance = float('inf')
        self.obstacle_direction = 0.0
        self.goal_reached = False
        self.navigation_active = False
        self.goals_completed = False
        self.obstacle_sectors = {'front': float('inf'), 'front_left': float('inf'), 'front_right': float('inf'),
                               'left': float('inf'), 'right': float('inf')}
        self.scan_ranges = {'min': 0, 'max': 0, 'increment': 0}
        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_duration = 5.0  # Increased from 3.0 seconds
        self.stuck_detection_buffer = []
        self.is_stuck = False
        self.stuck_counter = 0
        self.last_cmd_vel = Twist()
        self.recovery_strategy = 0  # To cycle through different recovery strategies
        
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
            {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
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
        
        # Store scan parameters
        self.scan_ranges['min'] = msg.angle_min
        self.scan_ranges['max'] = msg.angle_max
        self.scan_ranges['increment'] = msg.angle_increment
        
        # Replace NaN and inf values with a large number
        self.laser_ranges[np.isnan(self.laser_ranges)] = 10.0
        self.laser_ranges[np.isinf(self.laser_ranges)] = 10.0
        
        # Find the minimum distance and its direction
        if len(self.laser_ranges) > 0:
            self.min_distance = np.min(self.laser_ranges)
            min_idx = np.argmin(self.laser_ranges)
            self.obstacle_direction = min_idx * msg.angle_increment + msg.angle_min
            
            # Process laser data into sectors for better obstacle avoidance
            self.process_laser_sectors(msg)

    def process_laser_sectors(self, scan_msg):
        """Process laser scan data into meaningful sectors for navigation"""
        num_points = len(self.laser_ranges)
        
        if num_points == 0:
            return
            
        # Define sectors (angle ranges in the scan)
        front_range = int(self.scan_angle_front / scan_msg.angle_increment / 2)
        quarter_range = int(front_range / 2)
        center_idx = num_points // 2  # Center of the scan (typically straight ahead)
        
        # Front sector (center of scan)
        front_start = max(0, center_idx - quarter_range)
        front_end = min(num_points - 1, center_idx + quarter_range)
        front_sector = self.laser_ranges[front_start:front_end+1]
        
        # Front-left sector (between front and left)
        front_left_start = max(0, center_idx - front_range)
        front_left_end = front_start - 1
        front_left_sector = self.laser_ranges[front_left_start:max(0, front_left_end+1)]
        
        # Front-right sector (between front and right)
        front_right_start = front_end + 1
        front_right_end = min(num_points - 1, center_idx + front_range)
        front_right_sector = self.laser_ranges[min(front_right_start, num_points-1):min(front_right_end+1, num_points)]
        
        # Left sector (left portion of scan)
        left_sector = self.laser_ranges[:front_left_start]
        
        # Right sector (right portion of scan)
        right_sector = self.laser_ranges[front_right_end+1:] if front_right_end < num_points - 1 else []
        
        # Get minimum distances for each sector with safety checks
        self.obstacle_sectors['front'] = np.min(front_sector) if len(front_sector) > 0 else 10.0
        self.obstacle_sectors['front_left'] = np.min(front_left_sector) if len(front_left_sector) > 0 else 10.0
        self.obstacle_sectors['front_right'] = np.min(front_right_sector) if len(front_right_sector) > 0 else 10.0
        self.obstacle_sectors['left'] = np.min(left_sector) if len(left_sector) > 0 else 10.0
        self.obstacle_sectors['right'] = np.min(right_sector) if len(right_sector) > 0 else 10.0
        
        # Check for extremely close obstacles in any sector - emergency stop condition
        if self.min_distance < 0.15:  # If anything is extremely close, set all sectors to danger
            rospy.logwarn("CRITICAL OBSTACLE PROXIMITY DETECTED! Possible collision!")
            for sector in self.obstacle_sectors:
                self.obstacle_sectors[sector] = min(self.obstacle_sectors[sector], 0.15)
        
        # For debugging
        rospy.loginfo(f"Obstacle distances - Front: {self.obstacle_sectors['front']:.2f}, "
                     f"Front-L: {self.obstacle_sectors['front_left']:.2f}, "
                     f"Front-R: {self.obstacle_sectors['front_right']:.2f}, "
                     f"Left: {self.obstacle_sectors['left']:.2f}, "
                     f"Right: {self.obstacle_sectors['right']:.2f}")

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
                self.current_goal_index += 1
                
                # Check if all goals have been reached
                if self.current_goal_index >= len(self.goals):
                    rospy.loginfo("All goals have been reached! Navigation complete.")
                    self.goals_completed = True
                    # Stop the robot by sending a zero velocity command
                    self.cmd_vel_pub.publish(Twist())
                    # Cancel any active goals
                    if self.navigation_active:
                        self.move_base_client.cancel_all_goals()
                        self.navigation_active = False
                    break  # Exit the navigation loop
                else:
                    # Send the next goal
                    current_goal = self.goals[self.current_goal_index]
                    self.send_goal(current_goal['x'], current_goal['y'], current_goal['yaw'])
            
            # Show current status
            if self.navigation_active:
                distance_to_obstacle = self.min_distance if self.min_distance < 10.0 else "None"
                rospy.loginfo(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                            f"Yaw: {self.orientation['yaw']:.2f}, "
                            f"Nearest obstacle: {distance_to_obstacle}")
            
            self.rate.sleep()
        
        # Keep the node running but inactive after completing all goals
        rospy.loginfo("Navigation complete. Node will remain active but idle.")
        rospy.spin()

    def check_if_stuck(self):
        """Check if the robot is stuck by monitoring position changes"""
        # Record current position in buffer
        current_pos = (self.position['x'], self.position['y'], self.orientation['yaw'])
        
        # Initialize buffer if empty
        if len(self.stuck_detection_buffer) < 10:
            self.stuck_detection_buffer.append(current_pos)
            return False
        
        # Add new position and remove oldest
        self.stuck_detection_buffer.append(current_pos)
        self.stuck_detection_buffer.pop(0)
        
        # Calculate position variance - if very small, robot might be stuck
        x_positions = [pos[0] for pos in self.stuck_detection_buffer]
        y_positions = [pos[1] for pos in self.stuck_detection_buffer]
        
        x_variance = np.var(x_positions)
        y_variance = np.var(y_positions)
        
        # If the robot barely moved in the last several readings, it might be stuck
        if x_variance < 0.0001 and y_variance < 0.0001 and self.last_cmd_vel.linear.x > 0.05:
            self.stuck_counter += 1
            if self.stuck_counter > 3:  # Require multiple consecutive detections
                rospy.logwarn("ROBOT APPEARS TO BE STUCK! Initiating aggressive recovery...")
                self.is_stuck = True
                self.stuck_counter = 0
                return True
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)  # Decrease counter if moving
            self.is_stuck = False
        
        return False
    
    def manual_obstacle_avoidance(self):
        """
        Enhanced manual obstacle avoidance method using raw velocity commands.
        This is a backup in case move_base is not working properly.
        """
        rospy.loginfo("Starting enhanced manual obstacle avoidance...")
        last_time = rospy.Time.now()
        
        # State management to ensure complete rotations
        rotating_to_clear_path = False
        rotation_start_time = None
        rotation_direction = 0
        rotation_duration = 0
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time
            
            # Check if all goals have been completed
            if self.goals_completed:
                # Just sleep to keep the node running but not doing anything
                self.rate.sleep()
                continue
                
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
            
            # First, check if stuck
            if self.check_if_stuck():
                rospy.logwarn("Stuck detection triggered - entering aggressive recovery")
                self.recovery_mode = True
                self.recovery_start_time = rospy.Time.now()
                self.recovery_strategy = (self.recovery_strategy + 1) % 4  # Cycle through strategies
                rotating_to_clear_path = False  # Reset rotation state
                
            # Check if we're in recovery mode
            if self.recovery_mode:
                cmd_vel = self.execute_recovery_behavior()
                # Check if we should exit recovery mode
                if rospy.Time.now() - self.recovery_start_time > rospy.Duration(self.recovery_duration):
                    rospy.loginfo("Exiting recovery mode")
                    self.recovery_mode = False
                    rotating_to_clear_path = False  # Reset rotation state
            
            # EMERGENCY STOP - if any obstacle is extremely close (< 0.15m)
            elif self.min_distance < 0.15:
                rospy.logwarn("EMERGENCY STOP! Obstacle extremely close.")
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)  # Send stop command immediately
                
                # Then enter recovery mode
                self.recovery_mode = True
                self.recovery_start_time = rospy.Time.now()
                cmd_vel = self.execute_recovery_behavior()
                rotating_to_clear_path = False  # Reset rotation state
            
            # Goal reached check
            elif distance_to_goal < self.goal_tolerance:
                # Goal reached, move to next goal
                rospy.loginfo(f"Goal {self.current_goal_index + 1} reached!")
                self.current_goal_index += 1
                rotating_to_clear_path = False  # Reset rotation state
                
                # Check if all goals have been reached
                if self.current_goal_index >= len(self.goals):
                    rospy.loginfo("All goals have been reached! Navigation complete.")
                    self.goals_completed = True
                    # Stop the robot
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    continue  # Continue the loop but with goals_completed = True
                
                # Brief stop
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.sleep(1.0)  # Pause for 1 second
            
            # If we're currently in a rotation-to-clear-path state
            elif rotating_to_clear_path:
                # Pure rotation - no linear movement
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = rotation_direction * self.angular_speed
                
                # Check if we've been rotating long enough
                current_rotation_time = (rospy.Time.now() - rotation_start_time).to_sec()
                
                # Check if rotation is complete or if we now have a clear path in front
                if (current_rotation_time >= rotation_duration or 
                    self.obstacle_sectors['front'] > self.obstacle_threshold):
                    rospy.loginfo(f"Rotation complete or clear path found after {current_rotation_time:.1f}s of rotation")
                    rotating_to_clear_path = False  # Exit rotation state
                else:
                    rospy.loginfo(f"Continuing rotation to find clear path: {current_rotation_time:.1f}/{rotation_duration:.1f}s")
            
            # Critical obstacle detection - front obstacle detection
            elif self.obstacle_sectors['front'] < self.obstacle_threshold:
                # Front obstacle detected! Stop moving forward and rotate in place
                rospy.loginfo("Obstacle detected in front! Starting pure rotation to clear path...")
                
                # Start rotating in place (zero linear velocity)
                cmd_vel.linear.x = 0.0
                
                # Determine rotation direction based on which side has more space
                if self.obstacle_sectors['left'] > self.obstacle_sectors['right']:
                    rotation_direction = 1.0  # Rotate left (positive angular velocity)
                    rospy.loginfo("More space on left - rotating counterclockwise")
                else:
                    rotation_direction = -1.0  # Rotate right (negative angular velocity)
                    rospy.loginfo("More space on right - rotating clockwise")
                
                cmd_vel.angular.z = rotation_direction * self.angular_speed
                
                # Set up the rotation state
                rotating_to_clear_path = True
                rotation_start_time = rospy.Time.now()
                
                # Set a minimum rotation duration to prevent oscillation
                # (longer rotation for more severe obstacle detection)
                closeness_factor = 1.0 - (self.obstacle_sectors['front'] / self.obstacle_threshold)
                rotation_duration = 1.0 + (2.0 * closeness_factor)  # Between 1-3 seconds based on obstacle closeness
                
                rospy.loginfo(f"Beginning {rotation_duration:.1f}s rotation to find clear path")
            
            # Side obstacle avoidance - also rotate in place if too close to side obstacles
            elif min(self.obstacle_sectors['front_left'], self.obstacle_sectors['front_right']) < self.obstacle_threshold * 0.7:
                # Obstacle in forward arc but not directly in front - still rotate in place
                rospy.loginfo("Obstacle detected in forward arc! Rotating in place...")
                
                cmd_vel.linear.x = 0.0  # No forward movement
                
                if self.obstacle_sectors['front_left'] < self.obstacle_sectors['front_right']:
                    # Obstacle in front-left, rotate right
                    rotation_direction = -1.0
                    rospy.loginfo("Obstacle in front-left - rotating clockwise")
                else:
                    # Obstacle in front-right, rotate left
                    rotation_direction = 1.0
                    rospy.loginfo("Obstacle in front-right - rotating counterclockwise")
                
                cmd_vel.angular.z = rotation_direction * self.angular_speed
                
                # Set up the rotation state with a shorter duration
                rotating_to_clear_path = True
                rotation_start_time = rospy.Time.now()
                rotation_duration = 1.0  # Shorter rotation for side obstacles
                
                rospy.loginfo("Beginning 1.0s rotation to clear side obstacles")
            
            # Side obstacle avoidance - slight adjustments when clear ahead but obstacles on sides
            elif min(self.obstacle_sectors['left'], self.obstacle_sectors['right']) < self.obstacle_threshold:
                # Side obstacle detected but path ahead is clear
                if self.obstacle_sectors['left'] < self.obstacle_sectors['right']:
                    # Obstacle on left, slight right drift but keep moving
                    cmd_vel.angular.z = -0.3
                    rospy.loginfo("Side obstacle on left, drifting right")
                else:
                    # Obstacle on right, slight left drift but keep moving
                    cmd_vel.angular.z = 0.3
                    rospy.loginfo("Side obstacle on right, drifting left")
                
                # Continue forward movement at reduced speed
                cmd_vel.linear.x = self.linear_speed * 0.7
            
            # No obstacles, head towards the goal
            else:
                if abs(angle_diff) > 0.3:
                    # Large angle difference to goal - rotate in place first
                    cmd_vel.linear.x = 0.0  # No forward movement during significant rotation
                    cmd_vel.angular.z = 0.6 * angle_diff  # Proportional rotation
                    
                    # Limit angular velocity
                    cmd_vel.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd_vel.angular.z))
                    rospy.loginfo(f"Aligning to goal direction: angle_diff={angle_diff:.2f}")
                else:
                    # Aligned with goal direction - move forward
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = 0.2 * angle_diff  # Small correction while moving
                    rospy.loginfo("Moving toward goal with minor course correction")
            
            # Store command for stuck detection
            self.last_cmd_vel = cmd_vel
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Log status
            rospy.loginfo(f"Goal: ({goal_x:.2f}, {goal_y:.2f}), "
                        f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                        f"Distance: {distance_to_goal:.2f}, "
                        f"Obstacles - Front: {self.obstacle_sectors['front']:.2f}, "
                        f"Front-L: {self.obstacle_sectors['front_left']:.2f}, "
                        f"Front-R: {self.obstacle_sectors['front_right']:.2f}")
            
            self.rate.sleep()
        
        # Keep the node running but inactive after completing all goals
        rospy.loginfo("Navigation complete. Node will remain active but idle.")
        rospy.spin()
        
    def execute_recovery_behavior(self):
        """Execute advanced recovery behavior when stuck or in danger"""
        cmd_vel = Twist()
        
        # If we're stuck, use more aggressive recovery
        if self.is_stuck:
            # Use a cycling strategy to avoid getting permanently stuck
            if self.recovery_strategy == 0:
                # Strategy 1: Just rotate in place (no linear movement)
                cmd_vel.linear.x = 0.0
                # Choose random direction to avoid repeatedly trying the same approach
                turn_direction = 1 if random.random() > 0.5 else -1
                cmd_vel.angular.z = turn_direction * self.angular_speed
                rospy.logwarn("Stuck Recovery: Pure rotation to find clear path")
                
            elif self.recovery_strategy == 1:
                # Strategy 2: Aggressive backup with no turning
                cmd_vel.linear.x = -0.15
                cmd_vel.angular.z = 0.0
                rospy.logwarn("Stuck Recovery: Pure backward movement")
                
            elif self.recovery_strategy == 2:
                # Strategy 3: Backup with turn
                cmd_vel.linear.x = -0.12
                # Random direction
                turn_direction = 1 if random.random() > 0.5 else -1
                cmd_vel.angular.z = turn_direction * (self.angular_speed * 0.7)
                rospy.logwarn(f"Stuck Recovery: Backing up with {'left' if turn_direction > 0 else 'right'} turn")
                
            else:
                # Strategy 4: "Wiggle" rotation - oscillating angular velocity
                cmd_vel.linear.x = 0.0
                oscillation = sin(time.time() * 3.0) 
                cmd_vel.angular.z = oscillation * self.angular_speed * 1.2
                rospy.logwarn("Stuck Recovery: Oscillating rotation to escape")
                
            return cmd_vel
        
        # Normal recovery (not stuck, but obstacle detected)
        # For normal recovery, prioritize in-place rotation
        cmd_vel.linear.x = 0.0  # No forward/backward movement
        
        # Determine which way has more space
        if self.obstacle_sectors['left'] > self.obstacle_sectors['right']:
            # More space to the left
            cmd_vel.angular.z = self.angular_speed  # Pure left rotation
            rospy.loginfo("Recovery: Pure left rotation to find clear path")
        else:
            # More space to the right
            cmd_vel.angular.z = -self.angular_speed  # Pure right rotation
            rospy.loginfo("Recovery: Pure right rotation to find clear path")
            
        return cmd_vel

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