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
from actionlib_msgs.msg import GoalStatus

class TurtlebotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        
        # Enhanced Parameters (can be set from launch file)
        self.use_move_base = rospy.get_param('~use_move_base', True)  # Use move_base by default
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)  # Meters
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.4)  # Increased from 0.3
        self.danger_threshold = rospy.get_param('~danger_threshold', 0.25)  # Reduced from 0.4
        self.critical_threshold = rospy.get_param('~critical_threshold', 0.15)  # New parameter for critical proximity
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.7)  # rad/s - increased from 0.6
        self.scan_angle_front = rospy.get_param('~scan_angle_front', pi/2)  # 90 degrees
        self.rotation_cooldown = rospy.get_param('~rotation_cooldown', 1.0)  # New parameter - seconds to wait after rotation
        self.slow_zone_distance = rospy.get_param('~slow_zone_distance', 0.7)  # New parameter - distance to slow down
        self.rotation_precision = rospy.get_param('~rotation_precision', 0.1)  # New parameter - radians of precision for in-place rotation
        
        # Status variables
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.laser_ranges = []
        self.min_distance = float('inf')
        self.obstacle_direction = 0.0
        self.goal_reached = False
        self.navigation_active = False
        self.goals_completed = False
        
        # Enhanced obstacle detection
        self.obstacle_sectors = {
            'front': float('inf'), 
            'front_left': float('inf'), 
            'front_right': float('inf'),
            'left': float('inf'), 
            'right': float('inf'),
            'rear_left': float('inf'),  # New sector
            'rear_right': float('inf'), # New sector
            'rear': float('inf')        # New sector
        }
        
        self.scan_ranges = {'min': 0, 'max': 0, 'increment': 0}
        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_duration = 5.0  # 5 seconds
        self.stuck_detection_buffer = []
        self.is_stuck = False
        self.stuck_counter = 0
        self.last_cmd_vel = Twist()
        self.recovery_strategy = 0  # To cycle through different recovery strategies
        
        # New variables for improved rotation control
        self.last_rotation_time = rospy.Time.now()  # Initialize to allow immediate rotation
        self.rotation_in_progress = False
        self.target_rotation_angle = None
        self.rotation_start_yaw = None
        self.rotation_start_time = None
        self.last_obstacle_check_time = rospy.Time.now()
        self.obstacle_check_frequency = 0.2  # seconds between full obstacle checks
        
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
        
        rospy.loginfo("Enhanced Turtlebot Navigator initialized. Use move_base: %s", self.use_move_base)
        
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
            
            # Only do full processing periodically to reduce computational load
            current_time = rospy.Time.now()
            if (current_time - self.last_obstacle_check_time).to_sec() >= self.obstacle_check_frequency:
                self.process_laser_sectors(msg)
                self.last_obstacle_check_time = current_time

    def process_laser_sectors(self, scan_msg):
        """
        Enhanced process laser scan data into meaningful sectors for navigation
        Divides the 360° scan into 8 sectors for better spatial awareness
        """
        num_points = len(self.laser_ranges)
        
        if num_points == 0:
            return
            
        # Calculate indices for a complete 360° division into 8 sectors
        sector_size = num_points // 8  # Each sector covers 45 degrees
        
        # Define the start and end indices for each sector
        sector_indices = {}
        
        # Front sector (centered on 0 degrees / straight ahead)
        front_center = num_points // 2  # Assuming 0 rad is at the middle of the scan
        sector_indices['front'] = (front_center - sector_size//2, front_center + sector_size//2)
        
        # Front-right sector (45 degrees to the right)
        sector_indices['front_right'] = (front_center + sector_size//2, front_center + 3*sector_size//2)
        
        # Right sector (90 degrees to the right)
        sector_indices['right'] = (front_center + 3*sector_size//2, front_center + 5*sector_size//2)
        
        # Rear-right sector (135 degrees to the right / 225 degrees)
        sector_indices['rear_right'] = (front_center + 5*sector_size//2, min(front_center + 7*sector_size//2, num_points-1))
        
        # Rear sector (180 degrees / behind the robot)
        rear_start = (front_center + 7*sector_size//2) % num_points
        rear_end = (front_center - 7*sector_size//2) % num_points
        if rear_start > rear_end:  # Wrapping around the end of the array
            rear_sector = np.concatenate((self.laser_ranges[rear_start:], self.laser_ranges[:rear_end+1]))
        else:
            rear_sector = self.laser_ranges[rear_start:rear_end+1]
        
        # Front-left sector (45 degrees to the left)
        sector_indices['front_left'] = (max(0, front_center - 3*sector_size//2), front_center - sector_size//2)
        
        # Left sector (90 degrees to the left)
        sector_indices['left'] = (max(0, front_center - 5*sector_size//2), front_center - 3*sector_size//2)
        
        # Rear-left sector (135 degrees to the left / 225 degrees)
        sector_indices['rear_left'] = (max(0, front_center - 7*sector_size//2), front_center - 5*sector_size//2)
        
        # Extract the minimum distance for each sector
        for sector, (start, end) in sector_indices.items():
            if start < end:  # Normal case
                sector_data = self.laser_ranges[start:end+1]
            else:  # Wrapping around the end of the array
                sector_data = np.concatenate((self.laser_ranges[start:], self.laser_ranges[:end+1]))
            
            self.obstacle_sectors[sector] = np.min(sector_data) if len(sector_data) > 0 else 10.0
        
        # Handle the rear sector specially (since it might wrap around)
        self.obstacle_sectors['rear'] = np.min(rear_sector) if len(rear_sector) > 0 else 10.0
        
        # Check for extremely close obstacles in any sector - emergency stop condition
        if self.min_distance < self.critical_threshold:
            rospy.logwarn(f"CRITICAL OBSTACLE PROXIMITY DETECTED! Distance: {self.min_distance:.2f}m")
            # Find which sector has the critical obstacle
            critical_sector = min(self.obstacle_sectors.items(), key=lambda x: x[1])[0]
            rospy.logwarn(f"Critical obstacle detected in {critical_sector} sector")
        
        # Log sector data at a lower frequency to avoid flooding the terminal
        if random.random() < 0.2:  # Only log ~20% of the time
            rospy.loginfo(f"Obstacle distances - Front: {self.obstacle_sectors['front']:.2f}, "
                         f"FL: {self.obstacle_sectors['front_left']:.2f}, "
                         f"FR: {self.obstacle_sectors['front_right']:.2f}, "
                         f"Left: {self.obstacle_sectors['left']:.2f}, "
                         f"Right: {self.obstacle_sectors['right']:.2f}")

    def move_base_result_callback(self, msg):
        result_status = msg.status.status
        if result_status == GoalStatus.SUCCEEDED:  # Changed from actionlib.GoalStatus.SUCCEEDED
            rospy.loginfo("Goal reached successfully!")
            self.goal_reached = True
        elif result_status == GoalStatus.ABORTED:  # Changed from actionlib.GoalStatus.ABORTED
            rospy.logwarn("Goal aborted! Trying to recover...")
            # You could implement recovery behavior here
            self.goal_reached = True  # Mark as reached to move to the next goal
        elif result_status == GoalStatus.REJECTED:  # Changed from actionlib.GoalStatus.REJECTED
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
        """Enhanced check if the robot is stuck by monitoring position changes"""
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
        yaw_positions = [pos[2] for pos in self.stuck_detection_buffer]
        
        x_variance = np.var(x_positions)
        y_variance = np.var(y_positions)
        yaw_variance = np.var(yaw_positions)
        
        # Also check if we're trying to move but not moving
        linear_command = abs(self.last_cmd_vel.linear.x) > 0.05
        angular_command = abs(self.last_cmd_vel.angular.z) > 0.05
        
        # Enhanced stuck detection with motion intent analysis
        small_position_change = x_variance < 0.0001 and y_variance < 0.0001
        intended_linear_motion = linear_command and small_position_change
        intended_angular_motion = angular_command and yaw_variance < 0.001
        
        # If we're commanding motion but not moving as expected
        if (intended_linear_motion or intended_angular_motion) and not self.recovery_mode:
            self.stuck_counter += 1
            if self.stuck_counter > 3:  # Require multiple consecutive detections
                rospy.logwarn(f"ROBOT APPEARS TO BE STUCK! Position variance: x={x_variance:.6f}, y={y_variance:.6f}, yaw={yaw_variance:.6f}")
                self.is_stuck = True
                self.stuck_counter = 0
                return True
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)  # Decrease counter if moving
            self.is_stuck = False
        
        return False
    
    def perform_precise_rotation(self, cmd_vel, target_angle_diff, max_angular_speed=None):
        """
        Performs a precise rotation to a target angle difference.
        Returns the updated cmd_vel object.
        """
        if max_angular_speed is None:
            max_angular_speed = self.angular_speed
            
        # If this is the start of a new rotation
        if not self.rotation_in_progress:
            self.rotation_in_progress = True
            self.rotation_start_yaw = self.orientation['yaw']
            self.rotation_start_time = rospy.Time.now()
            self.target_rotation_angle = target_angle_diff
            rospy.loginfo(f"Starting precise rotation by {target_angle_diff:.2f} radians")
        
        # Calculate how much we've already rotated
        current_rotation = self.normalize_angle(self.orientation['yaw'] - self.rotation_start_yaw)
        remaining_rotation = self.normalize_angle(self.target_rotation_angle - current_rotation)
        
        # Calculate time spent rotating
        rotation_time = (rospy.Time.now() - self.rotation_start_time).to_sec()
        
        # Adjust angular velocity based on how much rotation remains
        # Use a proportional control for smoother deceleration as we approach the target
        kp = 2.0  # Proportional gain
        angular_velocity = kp * remaining_rotation
        
        # Limit the angular velocity to max_angular_speed
        if abs(angular_velocity) > max_angular_speed:
            angular_velocity = max_angular_speed * (1 if angular_velocity > 0 else -1)
        
        # Set a minimum angular velocity to overcome friction
        min_angular_vel = 0.2
        if 0 < abs(angular_velocity) < min_angular_vel:
            angular_velocity = min_angular_vel * (1 if angular_velocity > 0 else -1)
        
        # Set the angular velocity
        cmd_vel.angular.z = angular_velocity
        
        # Check if we've reached the target angle (within tolerance)
        if abs(remaining_rotation) < self.rotation_precision:
            rospy.loginfo(f"Rotation complete! Rotated by {current_rotation:.2f} radians in {rotation_time:.2f} seconds")
            self.rotation_in_progress = False
            cmd_vel.angular.z = 0.0  # Stop rotation
            self.last_rotation_time = rospy.Time.now()  # Update the last rotation time
            
        # Safety check for stuck rotations
        if rotation_time > 5.0 and abs(current_rotation) < 0.2:
            rospy.logwarn("Rotation appears stuck! Aborting rotation.")
            self.rotation_in_progress = False
            cmd_vel.angular.z = 0.0  # Stop rotation
            self.recovery_mode = True  # Enter recovery mode
            self.recovery_start_time = rospy.Time.now()
            
        return cmd_vel
        
    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
    
    def manual_obstacle_avoidance(self):
        """
        Enhanced manual obstacle avoidance method using raw velocity commands.
        This is a backup in case move_base is not working properly.
        """
        rospy.loginfo("Starting enhanced manual obstacle avoidance...")
        last_time = rospy.Time.now()
        
        # State variables for obstacle avoidance
        danger_detected = False
        danger_start_time = None
        danger_sector = None
        awaiting_rotation_cooldown = False
        cooldown_start_time = None
        
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
            angle_diff = self.normalize_angle(angle_to_goal - self.orientation['yaw'])
            
            # First, check if stuck
            if self.check_if_stuck():
                rospy.logwarn("Stuck detection triggered - entering aggressive recovery")
                self.recovery_mode = True
                self.recovery_start_time = rospy.Time.now()
                self.recovery_strategy = (self.recovery_strategy + 1) % 4  # Cycle through strategies
                danger_detected = False  # Reset danger state
                awaiting_rotation_cooldown = False  # Reset cooldown state
            
            # Check if we're in recovery mode
            if self.recovery_mode:
                cmd_vel = self.execute_recovery_behavior()
                # Check if we should exit recovery mode
                if rospy.Time.now() - self.recovery_start_time > rospy.Duration(self.recovery_duration):
                    rospy.loginfo("Exiting recovery mode")
                    self.recovery_mode = False
                    danger_detected = False  # Reset danger state
                    awaiting_rotation_cooldown = False  # Reset cooldown state
            
            # EMERGENCY STOP - if any obstacle is extremely close (< critical_threshold)
            elif self.min_distance < self.critical_threshold:
                rospy.logwarn(f"EMERGENCY STOP! Obstacle extremely close: {self.min_distance:.2f}m")
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)  # Send stop command immediately
                
                # Then enter recovery mode
                self.recovery_mode = True
                self.recovery_start_time = rospy.Time.now()
                cmd_vel = self.execute_recovery_behavior()
                danger_detected = False  # Reset danger state
                awaiting_rotation_cooldown = False  # Reset cooldown state
            
            # Goal reached check
            elif distance_to_goal < self.goal_tolerance:
                # Goal reached, move to next goal
                rospy.loginfo(f"Goal {self.current_goal_index + 1} reached! Distance: {distance_to_goal:.2f}m")
                self.current_goal_index += 1
                danger_detected = False  # Reset danger state
                awaiting_rotation_cooldown = False  # Reset cooldown state
                
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
            
            # If we're in a precise rotation (already in progress)
            elif self.rotation_in_progress:
                cmd_vel = self.perform_precise_rotation(cmd_vel, self.target_rotation_angle)
                
            # If we're awaiting rotation cooldown
            elif awaiting_rotation_cooldown:
                # Check if cooldown period has elapsed
                if (rospy.Time.now() - cooldown_start_time).to_sec() > self.rotation_cooldown:
                    rospy.loginfo("Rotation cooldown complete, resuming navigation")
                    awaiting_rotation_cooldown = False
                else:
                    # During cooldown, keep still
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    rospy.loginfo("In rotation cooldown...")
            
            # DANGER state - obstacle detected within danger threshold
            elif self.obstacle_sectors['front'] < self.danger_threshold or \
                 self.obstacle_sectors['front_left'] < self.danger_threshold or \
                 self.obstacle_sectors['front_right'] < self.danger_threshold:
                
                # First time detecting danger
                if not danger_detected:
                    danger_detected = True
                    danger_start_time = rospy.Time.now()
                    
                    # Find which sector has the closest obstacle
                    front_sectors = {
                        'front': self.obstacle_sectors['front'],
                        'front_left': self.obstacle_sectors['front_left'],
                        'front_right': self.obstacle_sectors['front_right']
                    }
                    danger_sector = min(front_sectors.items(), key=lambda x: x[1])[0]
                    rospy.logwarn(f"DANGER detected in {danger_sector}! Distance: {front_sectors[danger_sector]:.2f}m")
                
                # Stop immediately
                cmd_vel.linear.x = 0.0
                
                # Determine rotation direction based on where the danger is
                rotation_direction = 0
                if danger_sector == 'front':
                    # Choose direction based on which side has more space
                    if self.obstacle_sectors['left'] > self.obstacle_sectors['right']:
                        rotation_direction = pi/2  # Rotate 90 degrees left
                        rospy.loginfo("Front obstacle - rotating left 90 degrees")
                    else:
                        rotation_direction = -pi/2  # Rotate 90 degrees right
                        rospy.loginfo("Front obstacle - rotating right 90 degrees")
                elif danger_sector == 'front_left':
                    rotation_direction = -pi/3  # Rotate 60 degrees right
                    rospy.loginfo("Front-left obstacle - rotating right 60 degrees")
                elif danger_sector == 'front_right':
                    rotation_direction = pi/3  # Rotate 60 degrees left
                    rospy.loginfo("Front-right obstacle - rotating left 60 degrees")
                
                # Start precise rotation
                self.target_rotation_angle = rotation_direction
                self.rotation_in_progress = True
                self.rotation_start_yaw = self.orientation['yaw']
                self.rotation_start_time = rospy.Time.now()
                
                # Update cmd_vel for the first step of rotation
                cmd_vel = self.perform_precise_rotation(cmd_vel, rotation_direction)
                
                # Reset danger detection state
                danger_detected = False
            
            # Obstacle avoidance - if close to obstacle but not in immediate danger
            elif self.obstacle_sectors['front'] < self.obstacle_threshold or \
                 self.obstacle_sectors['front_left'] < self.obstacle_threshold or \
                 self.obstacle_sectors['front_right'] < self.obstacle_threshold:
                
                rospy.loginfo("Obstacle detected within avoidance threshold, slowing down and adjusting")
                
                # Reduce speed based on proximity to obstacle
                proximity_factor = 1.0 - (min(
                    self.obstacle_sectors['front'],
                    self.obstacle_sectors['front_left'],
                    self.obstacle_sectors['front_right']
                ) / self.obstacle_threshold)
                
                # Slow down more as we get closer to obstacles
                speed_reduction = 0.5 + (0.5 * proximity_factor)  # Ranges from 0.5 to 1.0
                cmd_vel.linear.x = self.linear_speed * (1.0 - speed_reduction)
                
                # Determine turn direction based on obstacle locations
                if self.obstacle_sectors['front'] < self.obstacle_threshold:
                    # Obstacle directly ahead, choose direction based on more open space
                    if self.obstacle_sectors['left'] > self.obstacle_sectors['right']:
                        cmd_vel.angular.z = self.angular_speed * 0.7  # Gentle left turn
                        rospy.loginfo("Front obstacle - gentle left turn")
                    else:
                        cmd_vel.angular.z = -self.angular_speed * 0.7  # Gentle right turn
                        rospy.loginfo("Front obstacle - gentle right turn")
                elif self.obstacle_sectors['front_left'] < self.obstacle_threshold:
                    # Obstacle on front-left, turn right
                    cmd_vel.angular.z = -self.angular_speed * 0.5
                    rospy.loginfo("Front-left obstacle - gentle right turn")
                elif self.obstacle_sectors['front_right'] < self.obstacle_threshold:
                    # Obstacle on front-right, turn left
                    cmd_vel.angular.z = self.angular_speed * 0.5
                    rospy.loginfo("Front-right obstacle - gentle left turn")
            
            # Side obstacle avoidance - adjust trajectory when obstacles on sides
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
                
                # Continue forward movement at slightly reduced speed
                cmd_vel.linear.x = self.linear_speed * 0.8
            
            # No obstacles in danger zone, head towards the goal
            else:
                if abs(angle_diff) > 0.3:
                    # Large angle difference to goal - first align
                    rospy.loginfo(f"Aligning to goal direction: angle_diff={angle_diff:.2f}")
                    
                    # Start precise rotation to goal orientation
                    self.rotation_in_progress = True
                    self.rotation_start_yaw = self.orientation['yaw']
                    self.rotation_start_time = rospy.Time.now()
                    self.target_rotation_angle = angle_diff
                    
                    # Initial rotation command
                    cmd_vel = self.perform_precise_rotation(cmd_vel, angle_diff)
                else:
                    # Aligned with goal direction - move forward
                    # Adjust speed based on distance to goal for smoother approach
                    if distance_to_goal < self.slow_zone_distance:
                        deceleration_factor = max(0.4, distance_to_goal / self.slow_zone_distance)
                        cmd_vel.linear.x = self.linear_speed * deceleration_factor
                        rospy.loginfo(f"Approaching goal, reduced speed: {cmd_vel.linear.x:.2f}")
                    else:
                        cmd_vel.linear.x = self.linear_speed
                        
                    # Small correction while moving
                    cmd_vel.angular.z = 0.3 * angle_diff
                    rospy.loginfo("Moving toward goal with minor course correction")
            
            # Store command for stuck detection
            self.last_cmd_vel = cmd_vel
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Check if a precise rotation just completed
            if not self.rotation_in_progress and (rospy.Time.now() - self.last_rotation_time).to_sec() < 0.2:
                # Enter cooldown state
                awaiting_rotation_cooldown = True
                cooldown_start_time = rospy.Time.now()
                rospy.loginfo(f"Entering rotation cooldown for {self.rotation_cooldown} seconds")
            
            # Log status (only log detailed status occasionally to avoid flooding)
            if random.random() < 0.3:  # ~30% chance each iteration
                rospy.loginfo(f"Goal: ({goal_x:.2f}, {goal_y:.2f}), "
                            f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                            f"Distance to goal: {distance_to_goal:.2f}m, "
                            f"Angle diff: {angle_diff:.2f}rad, "
                            f"Nearest obstacle: {self.min_distance:.2f}m")
            
            self.rate.sleep()
        
        # Keep the node running but inactive after completing all goals
        rospy.loginfo("Navigation complete. Node will remain active but idle.")
        rospy.spin()
        
    def execute_recovery_behavior(self):
        """Enhanced recovery behavior when stuck or in danger"""
        cmd_vel = Twist()
        
        # If we're stuck, use more aggressive recovery
        if self.is_stuck:
            # Use a cycling strategy to avoid getting permanently stuck
            if self.recovery_strategy == 0:
                # Strategy 1: Precise rotation by 180 degrees
                if not self.rotation_in_progress:
                    rospy.logwarn("Stuck Recovery: Rotating 180 degrees to find clear path")
                    self.rotation_in_progress = True
                    self.rotation_start_yaw = self.orientation['yaw']
                    self.rotation_start_time = rospy.Time.now()
                    self.target_rotation_angle = pi  # 180 degrees
                
                cmd_vel = self.perform_precise_rotation(cmd_vel, pi, max_angular_speed=self.angular_speed * 1.2)
                
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
        # First, check where obstacles are not present (find escape direction)
        safe_directions = []
        sector_angles = {
            'front': 0.0,
            'front_right': -pi/4,
            'right': -pi/2,
            'rear_right': -3*pi/4,
            'rear': pi,
            'rear_left': 3*pi/4,
            'left': pi/2,
            'front_left': pi/4
        }
        
        # Find sectors with no obstacles (or obstacles far away)
        for sector, angle in sector_angles.items():
            if self.obstacle_sectors.get(sector, float('inf')) > self.obstacle_threshold:
                safe_directions.append((sector, angle))
        
        if safe_directions:
            # If we have safe directions, choose the one closest to our current orientation
            # or the one that would require the least rotation
            safe_directions.sort(key=lambda x: abs(self.normalize_angle(x[1])))
            safest_sector, escape_angle = safe_directions[0]
            
            # Start precise rotation to the escape direction
            if not self.rotation_in_progress:
                rospy.loginfo(f"Recovery: Rotating to safe direction '{safest_sector}' ({escape_angle:.2f} radians)")
                self.rotation_in_progress = True
                self.rotation_start_yaw = self.orientation['yaw']
                self.rotation_start_time = rospy.Time.now()
                self.target_rotation_angle = escape_angle
            
            # Perform the rotation
            cmd_vel = self.perform_precise_rotation(cmd_vel, escape_angle)
            
        else:
            # No clear safe direction, rotate in place slowly
            cmd_vel.linear.x = 0.0  # No forward/backward movement
            cmd_vel.angular.z = self.angular_speed  # Default to left rotation
            rospy.loginfo("Recovery: No clear safe direction, rotating left to explore")
            
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