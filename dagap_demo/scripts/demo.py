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
from pycram.designators.object_designator import *

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
    print("Starting demo")
    frame = "sink_area_surface"

    # objects =

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

    # spawn kitchen
    kitchen = Object("kitchen", "environment", "kitchen.urdf")
    # kitchen.set_color([0.2, 0, 0.4, 0.6])
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    # Spawn breakfast cereal
    breakfast_cereal = Object(object_spawning_poses[1].Object,
                              object_spawning_poses[1].Object,
                              "breakfast_cereal.stl",
                              Pose(object_spawning_poses[1].object_location.position,
                                   object_spawning_poses[1].object_location.orientation,
                                   frame=frame))
    breakfast_cereal_desig = ObjectDesignatorDescription(names=[object_spawning_poses[1].Object])
    # Spawn breakfast cereal
    cup = Object(object_spawning_poses[2].Object,
                 object_spawning_poses[2].Object,
                 "../resources/cup.stl",
                 Pose(object_spawning_poses[2].object_location.position,
                      object_spawning_poses[2].object_location.orientation,
                      frame=frame))
    cup_desig = ObjectDesignatorDescription(names=[object_spawning_poses[2].Object])
    # Spawn breakfast cereal
    bowl = Object(object_spawning_poses[3].Object,
                  object_spawning_poses[3].Object,
                  "bowl.stl",
                  Pose(object_spawning_poses[3].object_location.position,
                       object_spawning_poses[3].object_location.orientation,
                       frame=frame))
    bowl_desig = ObjectDesignatorDescription(names=[object_spawning_poses[3].Object])
    # Spawn breakfast cereal
    spoon = Object(object_spawning_poses[4].Object,
                   object_spawning_poses[4].Object,
                   "spoon.stl",
                   Pose(object_spawning_poses[4].object_location.position,
                        object_spawning_poses[4].object_location.orientation,
                        frame=frame))
    spoon_desig = ObjectDesignatorDescription(names=[object_spawning_poses[4].Object])
    # Spawn breakfast cereal
    milk = Object(object_spawning_poses[5].Object,
                  object_spawning_poses[5].Object,
                  "milk.stl",
                  Pose(object_spawning_poses[5].object_location.position,
                       object_spawning_poses[5].object_location.orientation,
                       frame=frame))
    milk_desig = ObjectDesignatorDescription(names=[object_spawning_poses[5].Object])

    # spawn PR2
    pr2 = Object("pr2", "robot", "pr2.urdf", Pose([0, 0, 0]))
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

    # Send request to DAGAP service
    res = opm_dagap_client(reference_frame=frame,
                           object_list=object_spawning_poses)
    print("Received answer:")
    print(res)
    print("Finished cleanly.")
