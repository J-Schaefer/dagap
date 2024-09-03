#!/usr/bin/env python3

from __future__ import print_function

# ROS Imports

# DAGAP Imports
from dagap_msgs.srv import *
from dagap_msgs.msg import *
from pick_and_place_demo import PickAndPlaceDemo

# PyCRAM Imports
from pycram.designators.object_designator import *


def opm_dagap_client(reference_frame: str, object_list: [OPMObjectQuery]) -> GetNextOPMObjectResponse:
    rospy.loginfo("Waiting for service.")
    rospy.wait_for_service('dagap_opm_query')
    try:
        rospy.loginfo("Calling dagap_opm_query.")
        call_common_service = rospy.ServiceProxy('dagap_opm_query', GetNextOPMObject)
        srv = GetNextOPMObjectRequest(reference_frame, object_list)
        response = call_common_service(srv)
        rospy.loginfo("Received response.")
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def opm_client(object_list: [OPMObjectQuery]) -> GetOPMSortedListResponse:
    rospy.loginfo("Waiting for service.")
    rospy.wait_for_service('opm_query')
    try:
        rospy.loginfo("Calling opm_query.")
        call_opm_service = rospy.ServiceProxy('opm_query', GetOPMSortedList)
        srv = GetOPMSortedListRequest(object_list)
        response = call_opm_service(srv)
        rospy.loginfo("Received response.")
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def dagap_client(task_description: str, object_frame: [str]) -> GetGraspPoseResponse:
    rospy.loginfo("Waiting for service.")
    rospy.wait_for_service('dagap_query')
    try:
        rospy.loginfo("Calling dagap_query.")
        call_dagap_service = rospy.ServiceProxy('dagap_query', GetGraspPose)
        srv = GetGraspPoseRequest(task_description, object_frame)
        response = call_dagap_service(srv)
        rospy.loginfo("Received response.")
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print("Starting demo")

    use_dual_arm = True  # use dual arm pickup action
    use_opm = False  # use OPM service

    if use_dual_arm:
        rospy.loginfo("Running demo using the dual arm heuristic.")
    else:
        rospy.loginfo("Running demo without the dual arm heuristic.")

    if use_opm:
        rospy.loginfo("Running demo using the OPM service.")
    else:
        rospy.loginfo("Running demo without the OPM service.")

    Demo = PickAndPlaceDemo(use_dual_arm=use_dual_arm, use_opm=use_opm)  # init demo and spawn objects

    Demo.run()  # run demo
    rospy.loginfo("Finishing demo.")
