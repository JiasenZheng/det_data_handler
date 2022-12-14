#!/usr/bin/python3

"""
DataHandler class to collect data using optical flow
TODO:
1. compute optical flow of images in real time
2. get stationary image frames
3. save stationary images to disk
4. set max number of images to save
"""

import os
import cv2
import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class DataHandler():
    def __init__(self,
                 image_topic,
                 tmp_dir="/tmp/detection",
                 max_images=50):
        self.tmp_dir = tmp_dir
        # create tmp directory if not exist
        if not os.path.exists(self.tmp_dir):
            os.makedirs(self.tmp_dir)
        self.image_topic = image_topic
        self.camera_name = image_topic.split("/")[1]
        self.max_images = max_images

    def run(self):
        self.img_dir = os.path.join(self.tmp_dir, "train_images")
        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)
        # clear images in tmp directory
        for file in os.listdir(self.img_dir):
            os.remove(os.path.join(self.img_dir, file))
        self.bridge = cv_bridge.CvBridge()
        self.first_image = True
        self.first_save = True
        self.image_sub = rospy.Subscriber(self.image_topic,
                                          Image,
                                          self.image_callback)
        ns = rospy.get_namespace()
        self.optical_flow_image_pub = rospy.Publisher(ns + "/optical_flow_image",
                                                      Image,
                                                      queue_size=1)
        self.captured_image_pub = rospy.Publisher(ns + "/captured_image",
                                                    Image,
                                                    queue_size=1)
        self.prev_mag_sum = 0.0
        self.stationary_time = 0.0
        self.image_stored = False

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

    def pub_captured_image(self, cv_img):
        """
        Publish captured image
        """
        self.captured_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))

    def get_stationary_frames(self, image, diff, interval = 0.25):
        """
        Get stationary frames
        """
        if abs(diff) < 10000:
            self.stationary_time += 1.0 / 30.0
        else:
            self.stationary_time = 0.0
            self.image_stored = False
        if self.stationary_time > interval:
            if not self.image_stored:
                rospy.loginfo("Stationary frame detected")
                self.image_stored = True
                # save image to disk
                image_name = os.path.join(self.img_dir, "{}_{}.png".format(self.camera_name, rospy.get_time()))
                self.pub_captured_image(image)
                if self.first_save:
                    self.first_save = False
                else:
                    num_images = len(os.listdir(self.img_dir))
                    if num_images > self.max_images - 1:
                        rospy.loginfo("Maximum number of images reached")
                        rospy.signal_shutdown("Maximum number of images reached")
                    cv2.imwrite(image_name, image)
                    rospy.loginfo("Image saved to {}".format(image_name))

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
            mag, _, bgr = self.compute_optical_flow(self.prev_image, cv_img)
            self.pub_optical_flow_image(bgr)
            self.prev_image = cv_img
            # compute the sum of optical flow
            mag_sum = np.sum(mag)
            if self.prev_mag_sum == 0.0:
                self.prev_mag_sum = mag_sum
            else:
                # compute the difference of optical flow
                diff = mag_sum - self.prev_mag_sum
                self.prev_mag_sum = mag_sum
                # calculate the time interval of stationary frames
                self.get_stationary_frames(cv_img, diff)
