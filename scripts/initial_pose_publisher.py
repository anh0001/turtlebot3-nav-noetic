#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import math

class InitialPosePublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('initial_pose_publisher')
        
        # Get parameters
        self.initial_pose_x = rospy.get_param('~initial_pose_x', -2.0)
        self.initial_pose_y = rospy.get_param('~initial_pose_y', 0.0)
        self.initial_pose_z = rospy.get_param('~initial_pose_z', 0.0)
        self.initial_pose_a = rospy.get_param('~initial_pose_a', 0.0)  # yaw in radians
        self.publication_duration = rospy.get_param('~publication_duration', 5.0)  # seconds to publish
        
        # Create publisher for initial pose
        self.initial_pose_pub = rospy.Publisher('/initialpose', 
                                            PoseWithCovarianceStamped, 
                                            queue_size=1)
        
        rospy.loginfo("Initial pose publisher started")
        rospy.loginfo("Will publish pose (x=%f, y=%f, a=%f) for %f seconds", 
                     self.initial_pose_x, self.initial_pose_y, self.initial_pose_a,
                     self.publication_duration)
                     
        # Wait a moment to ensure ROS is fully initialized
        rospy.sleep(1.0)
        
        # Publish the initial pose
        self.publish_initial_pose()
        
    def create_initial_pose_msg(self):
        """Create the initial pose message with the specified coordinates."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        
        # Set the position
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.position.z = self.initial_pose_z
        
        # Set the orientation from yaw
        q = quaternion_from_euler(0, 0, self.initial_pose_a)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Set the covariance matrix
        # We'll use a relatively small covariance to indicate high confidence
        covariance = [0.1, 0, 0, 0, 0, 0,  # x
                      0, 0.1, 0, 0, 0, 0,  # y
                      0, 0, 0.1, 0, 0, 0,  # z
                      0, 0, 0, 0.1, 0, 0,  # roll
                      0, 0, 0, 0, 0.1, 0,  # pitch
                      0, 0, 0, 0, 0, 0.1]  # yaw
                     
        msg.pose.covariance = covariance
        
        return msg
        
    def publish_initial_pose(self):
        """Publish the initial pose repeatedly for the specified duration."""
        initial_pose_msg = self.create_initial_pose_msg()
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(1)  # 1 Hz
        
        while (rospy.Time.now() - start_time).to_sec() < self.publication_duration:
            rospy.loginfo("Publishing initial pose...")
            self.initial_pose_pub.publish(initial_pose_msg)
            rate.sleep()
            
        rospy.loginfo("Finished publishing initial pose")

if __name__ == '__main__':
    try:
        InitialPosePublisher()
    except rospy.ROSInterruptException:
        pass