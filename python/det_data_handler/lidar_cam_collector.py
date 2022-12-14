#!/usr/bin/python3

"""
This script is used to collect data from the camera and lidar.
TODO:
1. Implement canny edge detection to the camera image and publish it to a topic
2. Run optical flow calculation to the image
3. Create a service to bag record the data for specified time
4. Extract one frame of image from the bag file
5. Extract all the lidar data from the bag file
"""

import os
import cv2
import rospy
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from det_data_handler.srv import Record 
from det_data_handler.util import record_bag, extract_image_from_bag, extract_pcd_from_bag
from det_data_handler.data_handler import DataHandler


class LidarCamCollector(DataHandler):
    def __init__(self, image_topic, lidar_topic):
        super(LidarCamCollector, self).__init__(image_topic)
        self.lidar_topic = lidar_topic
        self.bridge = cv_bridge.CvBridge()
        self.prev_image = None
        self.bag_name = None

    def run(self):
        self.bag_file = "/tmp/detection/collect_lidar_cam/bags"
        if not os.path.exists(self.bag_file):
            os.makedirs(self.bag_file)
        self.image_file = "/tmp/detection/collect_lidar_cam/images"
        if not os.path.exists(self.image_file):
            os.makedirs(self.image_file)
        self.pcd_file = "/tmp/detection/collect_lidar_cam/pcds"
        if not os.path.exists(self.pcd_file):
            os.makedirs(self.pcd_file)
        self.image_sub = rospy.Subscriber(self.image_topic,
                                          Image,
                                          self.image_callback)
        self.image_edge_pub = rospy.Publisher("/camera/image_edge",
                                              Image,
                                              queue_size=1)
        self.record_bag_service = rospy.Service("/record_bag", 
                                                Record, 
                                                self.record_bag_callback)

    def image_edge_detection(self, cv_image):
        """
        Detect edges in the image
        """
        # convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # apply canny edge detection
        edges = cv2.Canny(gray, 100, 300, apertureSize=3)
        edges = cv2.dilate(edges, None)
        return edges

    def image_callback(self, msg):
        """
        Callback function for image topic
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        edges = self.image_edge_detection(cv_image)
        # change the edge background to raw image background
        cv_image[edges > 0] = (0, 0, 255)
        # roatate image 90 degree anti-clockwise
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # calculate optical flow
        if self.prev_image is not None:
            mag, _, _ = self.compute_optical_flow(cv_image, self.prev_image)
            mag_avg = np.round(np.average(mag), 3)
            cv2.putText(cv_image, "Optical Flow: " + str(mag_avg), (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4)
        self.prev_image = cv_image
        # publish the image with edges
        self.image_edge_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def extract_pcd(self):
        """
        Extract the pcd from the bag file
        """
        # check if pcd_file is empty
        if len(os.listdir(self.pcd_file)) == 0:
            pcd_name = os.path.join(self.pcd_file, "0.pcd")
        else:
            pcd_name = os.path.join(self.pcd_file, str(len(os.listdir(self.pcd_file))) + ".pcd")
        if self.bag_name is not None:
            extract_pcd_from_bag(self.bag_name, self.lidar_topic, pcd_name)
            rospy.loginfo("Extracted pcd from bag file: " + pcd_name)
            self.bag_name = None

    def record_bag_callback(self, req):
        """
        Callback function for record bag service
        """
        duration = req.duration
        # record bag
        current_time = rospy.get_time()
        time_str = str(rospy.Time(current_time).to_sec())
        time_str = time_str.split(".")[0]
        time_str = time_str.split(" ")[0]
        time_str = time_str.replace(":", "-")
        bag_name = self.bag_file + "/bag_" + str(time_str) + ".bag"
        topics = [self.image_topic, self.lidar_topic]
        # duration = 10.0
        record_bag(bag_name, topics, duration)
        self.bag_name = bag_name
        rospy.loginfo("Bag recorded to: " + bag_name)
        # extract image from bag
        if len(os.listdir(self.image_file)) == 0:
            image_name = os.path.join(self.image_file, "0.bmp")
        else:
            image_name = os.path.join(self.image_file, str(len(os.listdir(self.image_file))) + ".bmp")
        extract_image_from_bag(bag_name, self.image_topic, image_name)
        rospy.loginfo("Image extracted to: " + image_name)
        return EmptyResponse()
