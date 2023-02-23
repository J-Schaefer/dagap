#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from dagap_msgs.srv import *
from dagap_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from dagap.utils.tfwrapper import *


def opm_dagap_client(reference_frame, object_list):
    rospy.wait_for_service('dagap_opm_query')
    try:
        call_common_service = rospy.ServiceProxy('dagap_opm_query', GetNextOPMObject)
        srv = GetNextOPMObjectRequest(reference_frame, object_list)
        res = call_common_service(srv)
        print("Received answer.")
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Starting test")
    frame = "sink_area_surface"
    print("Finished cleanly.")
    point = Point(1, 2, 3)
    pose_as_list = point_to_list(point)
    object_spawning_poses = [
                             OPMObjectQuery("robot", list_to_pose([0, 0, 0], [0, 0, 0, 1])),
                             OPMObjectQuery("breakfast-cereal", list_to_pose([0.2, -0.15, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("cup", list_to_pose([0.2, -0.35, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("bowl", list_to_pose([0.18, -0.55, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("spoon", list_to_pose([0.15, -0.4, -0.05], [0, 0, 0, 1])),
                             OPMObjectQuery("milk", list_to_pose([0.07, -0.35, 0.1], [0, 0, 0, 1]))
                            ]
    res = opm_dagap_client(reference_frame=frame,
                           object_list=object_spawning_poses)
    # print()
