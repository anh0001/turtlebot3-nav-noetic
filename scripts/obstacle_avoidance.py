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
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.2)  # Meters - increased from 0.4
        self.danger_threshold = rospy.get_param('~danger_threshold', 0.4)  # Critical distance - increased from 0.25
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
                
            # Check if we're in recovery mode
            if self.recovery_mode:
                cmd_vel = self.execute_recovery_behavior()
                # Check if we should exit recovery mode
                if rospy.Time.now() - self.recovery_start_time > rospy.Duration(self.recovery_duration):
                    rospy.loginfo("Exiting recovery mode")
                    self.recovery_mode = False
            
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
            
            # Goal reached check
            elif distance_to_goal < self.goal_tolerance:
                # Goal reached, move to next goal
                rospy.loginfo(f"Goal {self.current_goal_index + 1} reached!")
                self.current_goal_index += 1
                
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
            
            # Critical obstacle detection - enter recovery mode
            elif min(self.obstacle_sectors['front'], 
                    self.obstacle_sectors['front_left'], 
                    self.obstacle_sectors['front_right']) < self.danger_threshold:
                rospy.logwarn("Critical obstacle detected ahead! Entering recovery mode...")
                self.recovery_mode = True
                self.recovery_start_time = rospy.Time.now()
                cmd_vel = self.execute_recovery_behavior()
            
            # Regular obstacle avoidance - front obstacle
            elif self.obstacle_sectors['front'] < self.obstacle_threshold:
                # Front obstacle detected, need to avoid
                rospy.loginfo("Obstacle detected directly in front! Stopping forward motion...")
                
                # First, stop forward motion
                cmd_vel.linear.x = 0.0
                
                # Determine which way to turn based on front-left/front-right sector distances
                if self.obstacle_sectors['front_left'] > self.obstacle_sectors['front_right']:
                    # More space on the front-left, turn left
                    cmd_vel.angular.z = self.angular_speed
                    rospy.loginfo("Turning left to avoid obstacle")
                else:
                    # More space on the front-right, turn right
                    cmd_vel.angular.z = -self.angular_speed
                    rospy.loginfo("Turning right to avoid obstacle")
            
            # Regular obstacle avoidance - front-left or front-right obstacle
            elif min(self.obstacle_sectors['front_left'], self.obstacle_sectors['front_right']) < self.obstacle_threshold:
                # Obstacle in forward arc but not directly in front
                rospy.loginfo("Obstacle detected in forward arc! Adjusting course...")
                
                if self.obstacle_sectors['front_left'] < self.obstacle_sectors['front_right']:
                    # Obstacle in front-left sector, turn right
                    cmd_vel.angular.z = -self.angular_speed * 0.8
                    rospy.loginfo("Turning right to avoid front-left obstacle")
                else:
                    # Obstacle in front-right sector, turn left
                    cmd_vel.angular.z = self.angular_speed * 0.8
                    rospy.loginfo("Turning left to avoid front-right obstacle")
                
                # Slow forward movement
                cmd_vel.linear.x = self.linear_speed * 0.3
            
            # Side obstacle avoidance - make more aggressive adjustments if obstacles are close on sides
            elif min(self.obstacle_sectors['left'], self.obstacle_sectors['right']) < self.obstacle_threshold:
                # Side obstacle detected
                if self.obstacle_sectors['left'] < self.obstacle_sectors['right']:
                    # Obstacle on left, drift right more aggressively
                    cmd_vel.angular.z = -self.angular_speed * 0.5
                    rospy.loginfo("Side obstacle on left, turning right")
                else:
                    # Obstacle on right, drift left more aggressively
                    cmd_vel.angular.z = self.angular_speed * 0.5
                    rospy.loginfo("Side obstacle on right, turning left")
                
                # Continue forward movement if path is clear ahead
                if self.obstacle_sectors['front'] > self.obstacle_threshold:
                    cmd_vel.linear.x = self.linear_speed * 0.5  # Half speed
            
            # No obstacles, head towards the goal
            else:
                if abs(angle_diff) > 0.3:
                    # Turn towards goal
                    cmd_vel.angular.z = 0.5 * angle_diff  # More aggressive turning
                    # Limit angular velocity
                    cmd_vel.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd_vel.angular.z))
                    
                    # Move slower when turning
                    turn_ratio = 1.0 - min(1.0, abs(angle_diff) / pi)
                    cmd_vel.linear.x = self.linear_speed * turn_ratio
                else:
                    # Move forward
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = 0.2 * angle_diff  # More aggressive correction
            
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
                # Strategy 1: Aggressive backup and random turn
                cmd_vel.linear.x = -0.15  # More aggressive backup
                # Random turn direction to escape potential local minima
                turn_direction = 1 if random.random() > 0.5 else -1
                cmd_vel.angular.z = turn_direction * self.angular_speed * 0.8
                rospy.logwarn("Stuck Recovery: Aggressive backup with random turn")
                
            elif self.recovery_strategy == 1:
                # Strategy 2: Rotate in place for a more complete view
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.angular_speed * 1.2  # Faster rotation
                rospy.logwarn("Stuck Recovery: Fast rotation to scan environment")
                
            elif self.recovery_strategy == 2:
                # Strategy 3: Wiggle - alternating left/right turns while backing up
                cmd_vel.linear.x = -0.12
                # Use time-based oscillation for the wiggle
                oscillation = sin(time.time() * 5.0)  # 5.0 controls the frequency
                cmd_vel.angular.z = oscillation * self.angular_speed
                rospy.logwarn("Stuck Recovery: Wiggle backing up")
                
            else:
                # Strategy 4: Forward burst with random direction (last resort)
                # This is risky but sometimes necessary to escape
                cmd_vel.linear.x = 0.15  # Forward burst
                cmd_vel.angular.z = (random.random() - 0.5) * self.angular_speed * 2.0
                rospy.logwarn("Stuck Recovery: Forward burst with random direction (risky)")
                
            return cmd_vel
        
        # Normal recovery (not stuck, but obstacle detected)
        # Determine recovery strategy based on obstacle configuration
        if self.obstacle_sectors['front'] < self.danger_threshold:
            # Obstacle very close in front - back up more aggressively
            cmd_vel.linear.x = -0.15  # Faster backward movement
            
            # Pick the direction with more space to turn while backing up
            if self.obstacle_sectors['front_left'] > self.obstacle_sectors['front_right']:
                cmd_vel.angular.z = self.angular_speed * 0.7  # More aggressive turn while backing up
                rospy.loginfo("Recovery: Backing up and turning left")
            else:
                cmd_vel.angular.z = -self.angular_speed * 0.7
                rospy.loginfo("Recovery: Backing up and turning right")
                
        elif min(self.obstacle_sectors['front_left'], self.obstacle_sectors['front_right']) < self.danger_threshold:
            # Obstacle in forward arc - rotate away from it
            cmd_vel.linear.x = -0.05  # Slight backup while rotating
            
            if self.obstacle_sectors['front_left'] < self.obstacle_sectors['front_right']:
                # Obstacle in front-left, turn right
                cmd_vel.angular.z = -self.angular_speed
                rospy.loginfo("Recovery: Backing slightly and rotating right")
            else:
                # Obstacle in front-right, turn left
                cmd_vel.angular.z = self.angular_speed
                rospy.loginfo("Recovery: Backing slightly and rotating left")
                
        elif min(self.obstacle_sectors['left'], self.obstacle_sectors['right']) < self.danger_threshold:
            # Tight space on sides - rotate toward the more open direction
            cmd_vel.linear.x = 0.0
            
            if self.obstacle_sectors['left'] > self.obstacle_sectors['right']:
                cmd_vel.angular.z = self.angular_speed  # Rotate left
                rospy.loginfo("Recovery: Rotating left in place")
            else:
                cmd_vel.angular.z = -self.angular_speed  # Rotate right
                rospy.loginfo("Recovery: Rotating right in place")
        else:
            # General recovery - do a full spin with randomized direction
            cmd_vel.linear.x = 0.0
            # Random direction to avoid getting stuck in back-and-forth motions
            if random.random() > 0.5:
                cmd_vel.angular.z = self.angular_speed
                rospy.loginfo("Recovery: Scanning by rotating left")
            else:
                cmd_vel.angular.z = -self.angular_speed
                rospy.loginfo("Recovery: Scanning by rotating right")
            
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