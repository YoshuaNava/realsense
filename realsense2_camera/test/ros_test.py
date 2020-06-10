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


class TestNode():
    """
    Test node class which contains the test cases of all
    four realsenses.
    """
    def __init__(self, logdir):
        """
        Constructor.

        Args:
            logdir (str): Directory to put the log files
        """
        self.realsenses = []
        self.start_time = None
        self.runtime = None
        rospy.init_node(TEST_NAME)
        self.__setup_logger(self.__get_log_filename(logdir))
        self.__get_enabled_realsenses()
        self.__configure()

    def __get_log_filename(self, logdir):
        """
        Creates the log file name

        Args:
            logdir (str): Log directory to use

        Returns:
            str: Full path to the log file
        """
        # In case a '~' is used, expand it to /home/<user>
        logdir = os.path.expanduser(logdir)
        logdir = os.path.join(logdir, TEST_NAME)
        if not os.path.exists(logdir):
            os.makedirs(logdir)
        # Create a stamped directory
        logdir = os.path.join(
            logdir, datetime.now().strftime("%Y%m%d_%H%M%S") + '_log')
        os.makedirs(logdir)
        # Create full path to file name and return it
        return os.path.join(logdir, TEST_NAME + '.log')

    def __setup_logger(self, filename):
        """
        Set up the logger

        Args:
            filename (str): Name of the log file
        """
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        # File handler
        fh = logging.FileHandler(filename)
        fh.setLevel(logging.INFO)
        # Console out handler (stdout&stderr)
        ch = logging.StreamHandler()
        ch.setLevel(logging.WARNING)
        # Log formatter
        formatter = logging.Formatter(
            '[%(asctime)s][%(levelname)s] %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        # Add the handlers to the logger object
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)

    def __get_enabled_realsenses(self):
        """
        Get the enabled realsenses from the parameter server
        and append them to RealsenseTester
        """
        if rospy.get_param('/realsense_test/front_enabled', True):
            fps = rospy.get_param('/depth_camera_front/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
            self.realsenses.append(RealsenseTester("front", TOPIC_RATE_MARGIN / fps))
        if rospy.get_param('/realsense_test/rear_enabled', True):
            fps = rospy.get_param('/depth_camera_rear/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
            self.realsenses.append(RealsenseTester("rear", TOPIC_RATE_MARGIN / fps))
        if rospy.get_param('/realsense_test/left_enabled', True):
            fps = rospy.get_param('/depth_camera_left/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
            self.realsenses.append(RealsenseTester("left", TOPIC_RATE_MARGIN / fps))
        if rospy.get_param('/realsense_test/right_enabled', True):
            fps = rospy.get_param('/depth_camera_right/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
            self.realsenses.append(RealsenseTester("right", TOPIC_RATE_MARGIN / fps))

    def __configure(self):
        """
        Call configure on all RealsenseTester objects
        """
        for r in self.realsenses:
            r.configure()

    def __print_stats(self):
        """
        Prints the stats and also logs them to file
        """
        rospy.loginfo("Test runtime: %s" % self.runtime)
        for r in self.realsenses:
            rospy.loginfo("" + r.get_name())
            rospy.loginfo("  #timeouts: %d" % r.get_num_timeouts())
            rospy.loginfo("  #stale data: %d" % r.get_num_stale_data())
            if r.get_num_msgs() == 0:
                rospy.loginfo("  #received msgs: NO DATA RECEIVED!")
            else:
                rospy.loginfo("  #received msgs: %d" % r.get_num_msgs())

    def run_test(self):
        """
        Runs the test until stopped
        """
        # Registering shutdown hook before starting
        rospy.on_shutdown(self.create_log)

        rospy.loginfo("Start test")
        self.start_time = datetime.now()
        try:
            rospy.spin()
        finally:
            rospy.loginfo("Test stopped")  # Log error
            self.runtime = datetime.now() - self.start_time

    def create_log(self):
        """
        Creates a log file.
        """
        # Print stats for now -> the logger actually logs this.
        self.__print_stats()


def main():
    # Run the test and create a log when done
    test = TestNode("~/longterm_test_logs")
    test.run_test()


if __name__ == '__main__':
    main()
