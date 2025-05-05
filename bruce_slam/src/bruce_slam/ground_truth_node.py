#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import pymap3d as pm
import tf2_ros
import tf.transformations

class GroundTruthNode:
    def __init__(self):
        rospy.init_node('ground_truth_node')
        
        # Origin for ENU conversion (will be set to first GPS reading)
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = 0.0

        self.drift_vector = np.array([0.01, 0.01, 0.0])
        self.drift_increment = 0.01


        self.initial_drift_vector = np.array([2.5, -25.3821, 0.0])
        
        # Latest data
        self.current_depth = 0.0
        self.current_heading = None
        self.current_orientation = None

        self.use_heading = rospy.get_param("~use_heading")
        
        # TF broadcaster for transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('ground_truth', Odometry, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/bluerov2/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/bluerov2/mavros/global_position/rel_alt', Float64, self.depth_callback)
        rospy.Subscriber('/bluerov2/mavros/global_position/compass_hdg', Float64, self.compass_callback)
        rospy.Subscriber('/bluerov2/mavros/imu/data', Imu, self.imu_callback)
        
        # Also subscribe to the odometry topic you want to compare with
        rospy.Subscriber('/your_odometry_topic', Odometry, self.odometry_callback)
        
        # Set publish rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Publish a static transform between map and odom
        # self.publish_static_transform()
        
        rospy.loginfo("Ground Truth Node initialized")

    def publish_static_transform(self):
        """Publish a static transform that aligns the map and odom frames"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        # Identity transform - placing the odom frame at the same origin as map
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(transform)
        rospy.loginfo("Published static transform between map and odom frames")

    def gps_callback(self, msg):
        # Set origin point if not set
        if self.origin_lat is None and self.origin_lon is None and msg.latitude != 0 and msg.longitude != 0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            rospy.loginfo(f"Origin set to: {self.origin_lat}, {self.origin_lon}, {self.origin_alt}")
            return
        
        # Skip if we have invalid data
        if msg.latitude == 0 and msg.longitude == 0:
            return
        
        # Convert GPS to ENU coordinates
        e, n, u = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, 
                                    self.origin_lat, self.origin_lon, self.origin_alt)
        
        # Create odometry message
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        # odom.header.frame_id = "map"
        odom.header.frame_id = "bluerov2/map"
        odom.child_frame_id = "gt_base_link"  # Changed to distinguish from odometry base_link


        e += self.initial_drift_vector[0]
        n += self.initial_drift_vector[1]

        
        if self.current_orientation is not None and self.use_heading is False:

            # Apply drift vector
            e += self.drift_vector[0]
            n += self.drift_vector[1]

            self.drift_vector[0] += self.drift_increment
            self.drift_vector[1] += self.drift_increment
            
            # Set the position
            odom.pose.pose.position = Point(e, n, self.current_depth)
            # Set the orientation with drift
            # Start with the current orientation
            q_orig = [
                self.current_orientation.x,
                self.current_orientation.y,
                self.current_orientation.z,
                self.current_orientation.w
            ]
            
            # Convert quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(q_orig)
            
            # Add drift to yaw (rotation around z-axis)
            drift_angle = 2 * np.sin(rospy.Time.now().to_sec() / 10.0)  # Small oscillating drift
            euler = (euler[0], euler[1], euler[2] + drift_angle)
            
            # Convert back to quaternion
            q_with_drift = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
            
            # Set the orientation with drift
            odom.pose.pose.orientation.x = q_with_drift[0]
            odom.pose.pose.orientation.y = q_with_drift[1]
            odom.pose.pose.orientation.z = q_with_drift[2]
            odom.pose.pose.orientation.w = q_with_drift[3]
            
            # Publish the message
            self.odom_pub.publish(odom)
            
            # Also publish the transform for visualization
            self.broadcast_transform(odom)

        if self.current_heading is not None and self.use_heading is True:
            print("Using heading")
            
            # Set the position
            odom.pose.pose.position = Point(e, n, self.current_depth)
            # odom.pose.pose.orientation = self.current_orientation
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_heading)

            # Assign quaternion to odom_msg
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]
            
            # Publish the message
            self.odom_pub.publish(odom)
            
            # Also publish the transform for visualization
            self.broadcast_transform(odom)

    def broadcast_transform(self, odom_msg):
        """Broadcast a transform from the odometry message"""
        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.child_frame_id = odom_msg.child_frame_id
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation.x = odom_msg.pose.pose.orientation.x
        transform.transform.rotation.y = odom_msg.pose.pose.orientation.y
        transform.transform.rotation.z = odom_msg.pose.pose.orientation.z
        transform.transform.rotation.w = odom_msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(transform)

    def odometry_callback(self, msg):
        """
        Process the odometry message from your SLAM system
        You may want to broadcast transforms or process this data
        """
        pass  # Implement if needed

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def compass_callback(self, msg):
        self.current_heading = msg.data

    def imu_callback(self, msg):
        self.current_orientation = msg.orientation

    def run(self):
        # Republish static transform periodically to ensure it's available
        timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_static_transform())
        
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GroundTruthNode()
        node.run()
    except rospy.ROSInterruptException:
        pass