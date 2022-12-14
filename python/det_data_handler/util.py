#!/usr/bin/python3

"""
Utility functions for data handler
"""

import os
import cv2
import rosbag
import cv_bridge
import rospkg
import rospy
import roslaunch

def record_bag(bag_name, topics, duration):
    """
    Record rosbag for a given duration and save to a given path (only works when roscore is running)
    """
    # create bag directory if not exist
    bag_dir = os.path.dirname(bag_name)
    if not os.path.exists(bag_dir):
        os.makedirs(bag_dir)
    # start rosbag record
    cmd = "rosbag record"
    for topic in topics:
        cmd += " " + topic
    cmd += " -O " + bag_name
    cmd += " --duration=" + str(duration)
    os.system(cmd)

def extract_image_from_bag(bag_name, image_topic, image_name):
    """
    Extract the middle image from a given rosbag and save to a given path
    """
    image_dir = os.path.dirname(image_name)
    # create image directory if not exist
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
    # extract images from bag
    bag = rosbag.Bag(bag_name)
    # save the image in the middle of the bag
    middle_time = bag.get_end_time() / 2
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if t.to_sec() > middle_time:
            # convert ros image to cv image
            bridge = cv_bridge.CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # convert cv image to grayscale if it is not
            if len(cv_image.shape) > 2:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # save image
            cv2.imwrite(image_name, cv_image)
            break
    bag.close()

def extract_pcd_from_bag(bag_name, lidar_topic, pcd_name):
    """
    Extract the pcd from a given rosbag and save to a given path
    """
    # launch bag to pcd launch file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path("det_data_handler")
    launch_file = os.path.join(pkg_dir, "launch", "bag_to_pcd.launch")
    cli_args = [launch_file, "bag_file:=" + bag_name, "lidar_topic:=" + lidar_topic, "pcd_file:=" + pcd_name]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
