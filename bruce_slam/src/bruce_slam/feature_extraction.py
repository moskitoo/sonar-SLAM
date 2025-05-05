#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, Image
import cv_bridge
import ros_numpy

from bruce_slam.utils.io import *
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.visualization import apply_custom_colormap
from bruce_slam import pcl
import matplotlib.pyplot as plt
from sonar_oculus.msg import OculusPing, OculusPingUncompressed
from scipy.interpolate import interp1d

from .utils import *
from .sonar import *

from bruce_slam.CFAR import CFAR

class FeatureExtraction(object):
    '''Class to handle extracting features from Sonar images using CFAR
    subsribes to the sonar driver and publishes a point cloud
    '''

    def __init__(self):
        '''Class constructor, no args required all read from yaml file
        '''

        #oculus info
        self.oculus = OculusProperty()

        #default parameters for CFAR
        self.Ntc = 40
        self.Ngc = 10
        self.Pfa = 1e-2
        self.rank = None
        self.alg = "SOCA"
        self.detector = None
        self.threshold = 0
        self.cimg = None

        #default parameters for point cloud 
        self.colormap = "RdBu_r"
        self.pub_rect = True
        self.resolution = 0.5
        self.outlier_filter_radius = 1.0
        self.outlier_filter_min_points = 5
        self.skip = 5

        # for offline visualization
        self.feature_img = None

        # Parameters for Cartesian image dimensions
        self.range_max = None  # Maximum range in meters
        self.range_resolution = None  # Meters per pixel
        self.n_ranges = None  # Number of range bins
        self.n_beams = None  # Number of beams
        self.image_width = None  # Width of the image in pixels
        self.image_height = None  # Height of the image in pixels

        #which vehicle is being used
        self.compressed_images = False  # Set to False for preprocessed images

        # place holder for the multi-robot system
        self.rov_id = ""


    def configure(self):
        '''Calls the CFAR class constructor for the featureExtraction class
        '''
        self.detector = CFAR(self.Ntc, self.Ngc, self.Pfa, self.rank)

    def init_node(self, ns="~"):

        #read in CFAR parameters
        self.Ntc = rospy.get_param(ns + "CFAR/Ntc")
        self.Ngc = rospy.get_param(ns + "CFAR/Ngc")
        self.Pfa = rospy.get_param(ns + "CFAR/Pfa")
        self.rank = rospy.get_param(ns + "CFAR/rank")
        self.alg = rospy.get_param(ns + "CFAR/alg", "SOCA")
        self.threshold = rospy.get_param(ns + "filter/threshold")

        #read in PCL downsampling parameters
        self.resolution = rospy.get_param(ns + "filter/resolution")
        self.outlier_filter_radius = rospy.get_param(ns + "filter/radius")
        self.outlier_filter_min_points = rospy.get_param(ns + "filter/min_points")

        #parameter to decide how often to skip a frame
        self.skip = rospy.get_param(ns + "filter/skip")

        #are the incoming images compressed?
        self.compressed_images = False  # Always use preprocessed images

        #cv bridge
        self.BridgeInstance = cv_bridge.CvBridge()
        
        #read in the format
        self.coordinates = rospy.get_param(
            ns + "visualization/coordinates", "cartesian"
        )

        #vis parameters
        self.radius = rospy.get_param(ns + "visualization/radius")
        self.color = rospy.get_param(ns + "visualization/color")

        # Subscribe to the preprocessed sonar image
        self.sonar_sub = rospy.Subscriber(
            SONAR_TOPIC_UNCOMPRESSED, Image, self.callback, queue_size=10)
            
        # Subscribe to metadata to get sonar parameters
        self.metadata_sub = rospy.Subscriber(
            SONAR_TOPIC_METADATA, OculusPingUncompressed, self.metadata_callback, queue_size=10)

        #feature publish topic
        self.feature_pub = rospy.Publisher(
            SONAR_FEATURE_TOPIC, PointCloud2, queue_size=10)

        #vis publish topic
        self.feature_img_pub = rospy.Publisher(
            SONAR_FEATURE_IMG_TOPIC, Image, queue_size=10)

        self.configure()
        
    def metadata_callback(self, metadata_msg):
        """Callback to get sonar metadata parameters needed for coordinate conversion"""
        self.range_max = metadata_msg.range
        self.range_resolution = metadata_msg.range_resolution
        self.n_ranges = metadata_msg.n_ranges
        self.n_beams = metadata_msg.n_beams
        
        # Update image dimensions based on metadata
        self.image_width = self.n_beams
        self.image_height = self.n_ranges

    def publish_features(self, msg, points):
        '''Publish the feature message using the provided parameters
        msg: Message with header information
        points: points to be converted to a ros point cloud, in cartisian meters
        '''

        #shift the axis
        points = np.c_[points[:,0], np.zeros(len(points)), points[:,1]]
        # points = np.c_[points[:,0], -points[:,1], np.zeros(len(points))]
        # points = np.c_[points[:,0], points[:,1], np.zeros(len(points))]

        #convert to a pointcloud
        feature_msg = n2r(points, "PointCloudXYZ")

        #give the feature message the same time stamp as the source sonar image
        #this is CRITICAL to good time sync downstream
        feature_msg.header.stamp = msg.header.stamp
        feature_msg.header.frame_id = "base_link"

        #publish the point cloud, to be used by SLAM
        self.feature_pub.publish(feature_msg)

    def cartesian_to_meters(self, locs):
        """Convert image coordinates to meters
        
        locs: Nx2 array of [row, col] coordinates in the image
        
        Returns Nx2 array of [x, y] coordinates in meters
        """
        if self.range_max is None or self.range_resolution is None:
            rospy.logwarn("Sonar metadata not received yet. Using default values.")
            self.range_max = 40.0
            self.range_resolution = 0.076
            self.image_width = 512
            self.image_height = 526
        
        # Calculate physical dimensions of the image in meters
        width_meters = self.range_max * 2  # Full width of the image in meters
        height_meters = self.range_max     # Height of the image in meters
        
        # Center of the image is typically at the bottom-center
        # Convert from image coordinates to meters
        # Assuming image origin (0,0) is at top-left
        
        # X coordinates (left-right)
        # Map columns from [0, image_width] to [-width_meters/2, width_meters/2]
        x = ((locs[:, 1] / float(self.image_width)) * width_meters) - (width_meters / 2)
        
        # Y coordinates (up-down)
        # Map rows from [0, image_height] to [0, height_meters]
        # Flip Y because image coordinates start from top-left, but sonar starts from bottom
        y = (1.0 - (locs[:, 0] / float(self.image_height))) * height_meters
        
        return np.column_stack((x, y))

    def callback(self, sonar_msg):
        '''Feature extraction callback
        sonar_msg: an Image message containing the preprocessed sonar image
        '''
        if not hasattr(sonar_msg, 'ping_id'):
            # Use header seq as ping_id for preprocessed images
            ping_id = sonar_msg.header.seq
        else:
            ping_id = sonar_msg.ping_id
            
        if ping_id % self.skip != 0:
            self.feature_img = None
            # Don't extract features in every frame.
            # But we still need empty point cloud for synchronization in SLAM node.
            nan = np.array([[np.nan, np.nan]])
            self.publish_features(sonar_msg, nan)
            return

        # Convert image message to numpy array
        img = ros_numpy.image.image_to_numpy(sonar_msg)
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect targets and check against threshold using CFAR
        peaks = self.detector.detect(img, self.alg)
        peaks &= img > self.threshold

        # Visualization
        vis_img = cv2.applyColorMap(img, 2)
        self.feature_img_pub.publish(ros_numpy.image.numpy_to_image(vis_img, "bgr8"))

        # Get feature locations
        locs = np.c_[np.nonzero(peaks)]
        
        if len(locs) > 0:
            # Convert from image coordinates to meters
            points = self.cartesian_to_meters(locs)
            
            # Filter the cloud using PCL
            if len(points) and self.resolution > 0:
                points = pcl.downsample(points, self.resolution)

            # Remove some outliers
            if self.outlier_filter_min_points > 1 and len(points) > 0:
                points = pcl.remove_outlier(
                    points, self.outlier_filter_radius, self.outlier_filter_min_points
                )
        else:
            points = np.zeros((0, 2))

        # Publish the feature message
        self.publish_features(sonar_msg, points)