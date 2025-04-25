#!/usr/bin/env python3
# filepath: /home/ubuntu/34763-autonomous-marine-robotics/Training_Sessions/TS6_Perception/ts6_ws/src/sonar-SLAM/bruce_slam/src/bruce_slam/mavros_odom_processor.py

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans

class OdometryProcessor:
    def __init__(self):
        rospy.init_node('mavros_odom_processor')
        
        # Flag to determine if we've received the initial position
        self.initialized = False
        
        # Store the initial odometry transform
        self.initial_position = None
        self.initial_orientation = None
        self.initial_orientation_inverse = None
        
        # ROS subscribers and publishers
        self.odom_sub = rospy.Subscriber('/bluerov2/mavros/local_position/odom', 
                                        Odometry, 
                                        self.odom_callback)
        
        self.odom_pub = rospy.Publisher('/bluerov2/mavros/local_position/odom/recentered', 
                                        Odometry, 
                                        queue_size=10)
        
        rospy.loginfo("Odometry processor initialized. Waiting for first odometry message...")

    def odom_callback(self, msg):
        if not self.initialized:
            # Store the initial position and orientation
            self.initial_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # Get the initial orientation as a quaternion
            initial_quat = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            self.initial_orientation = initial_quat
            
            # Calculate the inverse of the initial quaternion
            self.initial_orientation_inverse = tf_trans.quaternion_inverse(initial_quat)
            
            self.initialized = True
            rospy.loginfo("Initial position set: [{:.2f}, {:.2f}, {:.2f}]".format(
                self.initial_position[0], self.initial_position[1], self.initial_position[2]))
            
        # Create a new odometry message
        transformed_odom = Odometry()
        transformed_odom.header = msg.header
        transformed_odom.child_frame_id = msg.child_frame_id
        
        # Get the current position
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Calculate position relative to initial position
        rel_pos = current_pos - self.initial_position
        
        # Get the current orientation as quaternion
        current_quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Calculate the relative orientation
        # This rotates the orientation by the inverse of the initial orientation
        rel_quat = tf_trans.quaternion_multiply(current_quat, self.initial_orientation_inverse)
        
        # Set the transformed position and orientation
        transformed_odom.pose.pose.position.x = rel_pos[0]
        transformed_odom.pose.pose.position.y = rel_pos[1]
        transformed_odom.pose.pose.position.z = rel_pos[2]
        
        transformed_odom.pose.pose.orientation.x = rel_quat[0]
        transformed_odom.pose.pose.orientation.y = rel_quat[1]
        transformed_odom.pose.pose.orientation.z = rel_quat[2]
        transformed_odom.pose.pose.orientation.w = rel_quat[3]
        
        # Copy the twist information (velocities)
        transformed_odom.twist = msg.twist
        
        # Publish the transformed odometry
        self.odom_pub.publish(transformed_odom)

if __name__ == '__main__':
    try:
        odometry_processor = OdometryProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass