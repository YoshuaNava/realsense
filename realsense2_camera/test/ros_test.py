#!/usr/bin/env python3

# System imports
import os
from datetime import datetime
import time
import sys
import logging

# ROS imports
import rospy
import rosnode
from sensor_msgs.msg import PointCloud2
from anymal_longterm_tests import TopicTimeoutTester
from anymal_longterm_tests import TestNode

# Timing margin that a message is allowed to have before being considered
# timed out. That means if two subsequent messages have a time delta of more
# than  TOPIC_RATE_MARGIN * NOMINAL_RATE, the message is timed out. The nominal
# rate is queried from the param server. TOPIC_RATE_MARGIN should be >= 1 to
# be meaningfuk
TOPIC_RATE_MARGIN = 2.0 
# Default timeout If the nominal rate is not found on the param server
TOPIC_TIMEOUT = 0.5 
TOPIC_HARD_TIMEOUT = 30
TEST_NAME = "realsense_test"


class RealsenseTester(TopicTimeoutTester):
    """
    Realsense test class that inherits from the topic timeout tester. 
    The class tests if the realsense topic times out. It keeps track
    of the number of messages and timeouts and logs the timeout length.

    Args:
        TopicTimeoutTester (TopicTimeoutTester): Base class
    """
    def __init__(self, suffix="front", timeout=TOPIC_TIMEOUT):
        """[summary]

        Args:
            suffix (str, optional): Realsense suffix. Defaults to "front".
        """
        super().__init__(topic="/depth_camera_" + suffix +
                               "/depth/color/points",
                         message_type=PointCloud2,
                         timeout=timeout,
                         hard_timeout=TOPIC_HARD_TIMEOUT)
        self.name = "realsense_test" + suffix

    def get_name(self):
        """
        Name of the tester instance

        Returns:
            str: name
        """
        return self.name


def get_enabled_realsenses():
    """
    Get the enabled realsenses from the parameter server
    and append them to RealsenseTester
    """
    realsenses = []
    if rospy.get_param('/realsense_test/front_enabled', True):
        fps = rospy.get_param('/depth_camera_front/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("front", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/rear_enabled', True):
        fps = rospy.get_param('/depth_camera_rear/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("rear", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/left_enabled', True):
        fps = rospy.get_param('/depth_camera_left/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("left", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/right_enabled', True):
        fps = rospy.get_param('/depth_camera_right/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("right", TOPIC_RATE_MARGIN / fps))
    return realsenses


def main():
    # Run the test and create a log when done
    rospy.init_node(TEST_NAME)
    realsense_testers = get_enabled_realsenses()
    test = TestNode(TEST_NAME, realsense_testers)
    test.run_test()


if __name__ == '__main__':
    main()
