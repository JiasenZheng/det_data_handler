#!/usr/bin/python3

"""
This script is used to handle both raw images and point clouds
TODO:
1. compute optical flow of images both real-time and offline (bag file)
"""

import cv2
import rospy
import rosbag
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import det_data_handler.utils

class DataHandler():
    def __init__(self,
                 image_topic,
                 real_time=True,):
        self.image_topic = image_topic
        self.real_time = real_time
        self.bridge = cv_bridge.CvBridge()
        self.first_image = True
        self.image_sub = rospy.Subscriber(self.image_topic,
                                          Image,
                                          self.image_callback)
        self.optical_flow_image_pub = rospy.Publisher("/optical_flow_image",
                                                      Image,
                                                      queue_size=1)

    def compute_optical_flow(self, image1, image2):
        """
        Compute optical flow of two images
        """
        # resize images
        image1 = cv2.resize(image1, (640, 480))
        image2 = cv2.resize(image2, (640, 480))
        # convert images to gray scale
        img1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(image1)
        hsv[..., 1] = 255
        # compute optical flow
        flow = cv2.calcOpticalFlowFarneback(img1, img2, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        hsv[..., 0] = ang * 180 / np.pi / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return mag, ang, bgr

    def pub_optical_flow_image(self, cv_img):
        """
        Publish optical flow image
        """
        self.optical_flow_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))

    def image_callback(self, msg):
        """
        Callback function for image topic
        """
        if "image_raw" in self.image_topic:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            try:
                cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            except cv_bridge.CvBridgeError as e:
                rospy.logerr(e)
        if self.first_image:
            self.prev_image = cv_img
            self.first_image = False
            rospy.loginfo("First image received")
        else:
            # time the computation of optical flow
            start_time = rospy.get_time()
            _, _, bgr = self.compute_optical_flow(self.prev_image, cv_img)
            end_time = rospy.get_time()
            rospy.loginfo("Time to compute optical flow: {}".format(end_time - start_time))
            self.pub_optical_flow_image(bgr)
            self.prev_image = cv_img

if __name__ == '__main__':
    rospy.init_node('data_handler_node')
    image_topic = "/camera/color/image_raw"
    data_handler = DataHandler(image_topic)
    rospy.spin()
