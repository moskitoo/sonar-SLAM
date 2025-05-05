#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import tf2_ros
import tf.transformations

class BlueRovRePublisher:
    def __init__(self):
        rospy.init_node('bluerov_republisher')
        
        # Parameters
        self.input_odom_topic = rospy.get_param("~input_odom_topic", "/bluerov2/mavros/global_position/local")
        self.output_odom_topic = rospy.get_param("~output_odom_topic", "/mavros_odom")
        self.output_frame_id = rospy.get_param("~output_frame_id", "odom")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "base_link")
        self.apply_drift = rospy.get_param("~apply_drift", False)
        
        # Drift parameters
        self.drift_vector = np.array([0.01, 0.01, 0.0])
        self.drift_increment = rospy.get_param("~drift_increment", 0.001)
        self.drift_period = rospy.get_param("~drift_period", 10.0)  # Period for oscillating drift in seconds
        
        # TF broadcaster for transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher(self.output_odom_topic, Odometry, queue_size=10)
        
        # Subscribe to the odometry topic you want to republish
        rospy.Subscriber(self.input_odom_topic, Odometry, self.odometry_callback)
        
        # Set publish rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Publish a static transform between map and odom if needed
        # self.publish_static_transform()
        
        rospy.loginfo("BlueRov Republisher Node initialized")
        rospy.loginfo(f"Input topic: {self.input_odom_topic}")
        rospy.loginfo(f"Output topic: {self.output_odom_topic}")
        rospy.loginfo(f"Apply drift: {self.apply_drift}")

    def publish_static_transform(self):
        """Publish a static transform that aligns the map and odom frames"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = self.output_frame_id
        # Identity transform - placing the odom frame at the same origin as map
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(transform)
        rospy.loginfo(f"Published static transform between map and {self.output_frame_id} frames")

    def odometry_callback(self, msg):
        # Create a new odometry message
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.output_frame_id
        odom.child_frame_id = self.output_child_frame_id
        
        # Copy values from the input message
        odom.pose = msg.pose
        odom.twist = msg.twist
        
        # Get the current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Get the current orientation
        orientation = msg.pose.pose.orientation
        
        if self.apply_drift:
            # Apply drift to position
            x += self.drift_vector[0]
            y += self.drift_vector[1]
            z += self.drift_vector[2]
            
            # Update the drift vector (simple linear increase)
            self.drift_vector[0] += self.drift_increment
            self.drift_vector[1] += self.drift_increment
            
            # Apply drift to orientation
            q_orig = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ]
            
            # Convert quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(q_orig)
            
            # Add drift to yaw (rotation around z-axis)
            drift_angle = 0.1 * np.sin(rospy.Time.now().to_sec() / self.drift_period)
            euler = (euler[0], euler[1], euler[2] + drift_angle)
            
            # Convert back to quaternion
            q_with_drift = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
            
            # Set the orientation with drift
            orientation.x = q_with_drift[0]
            orientation.y = q_with_drift[1]
            orientation.z = q_with_drift[2]
            orientation.w = q_with_drift[3]
        
        # Update the odometry message with potentially drifted values
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = orientation
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        # Optionally publish the transform
        self.publish_transform(x, y, z, orientation)

    def publish_transform(self, x, y, z, orientation):
        """Publish transform from output_frame_id to output_child_frame_id"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.output_frame_id
        transform.child_frame_id = self.output_child_frame_id
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        transform.transform.rotation = orientation
        
        self.tf_broadcaster.sendTransform(transform)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = BlueRovRePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass