#!/usr/bin/env python3

from __future__ import print_function

# ROS Imports
import rospy

# PyCRAM Imports
from pycram.process_module import simulated_robot, real_robot
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.designator import ObjectDesignatorDescription
from pycram.pose import Pose
from pycram.plan_failures import IKError
from pycram.local_transformer import LocalTransformer


class PopcornDemo:
    def __init__(self):

        pass

    def run(self):
        pass


if __name__ == "__main__":
    print("Starting demo")

    Demo = PopcornDemo()  # init demo and spawn objects

    rospy.loginfo("Running popcorn demo.")

    Demo.run()  # run demo
    rospy.loginfo("Finishing demo.")
