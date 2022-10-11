#!/usr/bin/python3

"""
Utility functions for data handler
"""

import os
import rosbag
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
    cmd = "rosbag record -O " + bag_name + " -a"
    for topic in topics:
        cmd += " " + topic
    cmd += " -d " + str(duration)
    os.system(cmd)
