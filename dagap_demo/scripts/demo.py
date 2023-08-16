#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from dagap_msgs.srv import *
from dagap_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from dagap.utils.tfwrapper import *
import pycram
from pycram.bullet_world import BulletWorld, Object
import pycram.bullet_world_reasoning as btr
import tf
from pycram.designators.motion_designator import MotionDesignatorDescription, MoveArmJointsMotion
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.language import macros, par
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.enums import Arms

def opm_dagap_client(reference_frame, object_list):
    rospy.wait_for_service('dagap_opm_query')
    try:
        call_common_service = rospy.ServiceProxy('dagap_opm_query', GetNextOPMObject)
        srv = GetNextOPMObjectRequest(reference_frame, object_list)
        response = call_common_service(srv)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Starting test")
    frame = "sink_area_surface"

    object_spawning_poses = [
                             OPMObjectQuery("robot", list_to_pose([0, 0, 0], [0, 0, 0, 1])),
                             OPMObjectQuery("breakfast-cereal", list_to_pose([0.2, -0.15, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("cup", list_to_pose([0.2, -0.35, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("bowl", list_to_pose([0.18, -0.55, 0.1], [0, 0, 0, 1])),
                             OPMObjectQuery("spoon", list_to_pose([0.15, -0.4, -0.05], [0, 0, 0, 1])),
                             OPMObjectQuery("milk", list_to_pose([0.07, -0.35, 0.1], [0, 0, 0, 1]))
                            ]

    # Set up the bullet world
    world = BulletWorld()
    world.set_gravity([0, 0, -9.8])

    plane = Object("floor", "environment", "plane.urdf", world=world)
    # plane.set_color([0, 0, 0, 1])
    # Action designator example from notebook
    # makin a simple pouring plan
    from pycram.designators.object_designator import *

    # spawn kitchen
    kitchen = Object("kitchen", "environment", "kitchen.urdf")
    # kitchen.set_color([0.2, 0, 0.4, 0.6])
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
    # spawn Milkbox
    milk = Object("milk", "milk", "milk.stl", Pose([1.6, 1, 0.90]))
    milk_desig = ObjectDesignatorDescription(names=["milk"])

    # spawn bowl
    bowl = Object("bowl", "bowl", "bowl.stl", Pose([1.6, 1, 0.90]))
    bowl_desig = ObjectDesignatorDescription(names=["bowl"])

    # spawn PR2
    pr2 = Object("pr2", "robot", "pr2.urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

    # Send request to DAGAP service
    res = opm_dagap_client(reference_frame=frame,
                           object_list=object_spawning_poses)
    print("Received answer:")
    print(res)
    print("Finished cleanly.")
