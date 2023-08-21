#!/usr/bin/env python

from __future__ import print_function

import sys
from typing import List, Type

import rospy

import dagap_msgs.msg
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
    reference_frame = "sink_area_surface"
    init()  # call tfwrapper init()

    object_spawning_poses_sink: List[Pose] = [
        list_to_pose([0, 0, 0], [0, 0, 0, 1]),  # robot, position empty
        list_to_pose([0.2, -0.15, 0.1], [0, 0, 0, 1]),  # breakfast-cereal
        list_to_pose([0.2, -0.35, 0.1], [0, 0, 0, 1]),  # cup
        list_to_pose([0.18, -0.55, 0.1], [0, 0, 0, 1]),  # bowl
        list_to_pose([0.15, -0.4, -0.05], [0, 0, 0, 1]),  # spoon
        list_to_pose([0.07, -0.35, 0.1], [0, 0, 0, 1])  # milk
    ]

    # transform_sink_map = lookup_transform(frame, u'map')

    # Hint for type of element
    element: geometry_msgs.msg.Pose
    # Hint for type of list object_spawning_poses_map
    object_spawning_poses_map: List[geometry_msgs.msg.PoseStamped] = []

    # Transform poses from iai_kitchen/sink_area_surface to map for correct spawn pose
    for element in object_spawning_poses_sink:
        object_spawning_poses_map.append(transform_pose(element, 'map', 'iai_kitchen/'+reference_frame))

    query_object_list_map: List[OPMObjectQuery] = [
                                                   OPMObjectQuery("robot", object_spawning_poses_sink[0]),
                                                   OPMObjectQuery("breakfast-cereal", object_spawning_poses_map[1].pose),
                                                   OPMObjectQuery("cup", object_spawning_poses_map[2].pose),
                                                   OPMObjectQuery("bowl", object_spawning_poses_map[3].pose),
                                                   OPMObjectQuery("spoon", object_spawning_poses_map[4].pose),
                                                   OPMObjectQuery("milk", object_spawning_poses_map[5].pose)
                                                  ]

    query_object_list_sink: List[OPMObjectQuery] = [
                                                   OPMObjectQuery("robot", object_spawning_poses_sink[0]),
                                                   OPMObjectQuery("breakfast-cereal", object_spawning_poses_sink[1]),
                                                   OPMObjectQuery("cup", object_spawning_poses_sink[2]),
                                                   OPMObjectQuery("bowl", object_spawning_poses_sink[3]),
                                                   OPMObjectQuery("spoon", object_spawning_poses_sink[4]),
                                                   OPMObjectQuery("milk", object_spawning_poses_sink[5])
                                                  ]

    # Set up the bullet world
    world = BulletWorld()
    world.set_gravity([0, 0, -9.8])

    plane = Object("floor", "environment", "plane.urdf", world=world)
    # plane.set_color([0, 0, 0, 1])

    # Spawn kitchen
    kitchen = Object("kitchen", "environment", "kitchen.urdf")
    # kitchen.set_color([0.2, 0, 0.4, 0.6])
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    # Spawn breakfast cereal
    breakfast_cereal = Object(query_object_list_map[1].Object,
                              query_object_list_map[1].Object,
                              path="breakfast_cereal.stl",
                              pose=Pose(point_to_list(query_object_list_map[1].object_location.position),
                                        quaternion_to_list(query_object_list_map[1].object_location.orientation),
                                        frame="iai_kitchen/"+reference_frame))
    breakfast_cereal_desig = ObjectDesignatorDescription(names=[query_object_list_map[1].Object])
    # Spawn cup
    cup = Object(query_object_list_map[2].Object,
                 query_object_list_map[2].Object,
                 path="../resources/cup.stl",
                 pose=Pose(point_to_list(query_object_list_map[2].object_location.position),
                           quaternion_to_list(query_object_list_map[2].object_location.orientation),
                           frame="iai_kitchen/"+reference_frame))
    cup_desig = ObjectDesignatorDescription(names=[query_object_list_map[2].Object])
    # Spawn bowl
    bowl = Object(query_object_list_map[3].Object,
                  query_object_list_map[3].Object,
                  path="bowl.stl",
                  pose=Pose(point_to_list(query_object_list_map[3].object_location.position),
                            quaternion_to_list(query_object_list_map[3].object_location.orientation),
                            frame="iai_kitchen/"+reference_frame))
    bowl_desig = ObjectDesignatorDescription(names=[query_object_list_map[3].Object])
    # Spawn spoon
    spoon = Object(query_object_list_map[4].Object,
                   query_object_list_map[4].Object,
                   path="spoon.stl",
                   pose=Pose(point_to_list(query_object_list_map[4].object_location.position),
                             quaternion_to_list(query_object_list_map[4].object_location.orientation),
                             frame="iai_kitchen/"+reference_frame))
    spoon_desig = ObjectDesignatorDescription(names=[query_object_list_map[4].Object])
    # Spawn milk
    milk = Object(query_object_list_map[5].Object,
                  query_object_list_map[5].Object,
                  path="milk.stl",
                  pose=Pose(point_to_list(query_object_list_map[5].object_location.position),
                            quaternion_to_list(query_object_list_map[5].object_location.orientation),
                            frame="iai_kitchen/"+reference_frame))
    milk_desig = ObjectDesignatorDescription(names=[query_object_list_map[5].Object])

    # Spawn PR2 robot
    pr2 = Object("pr2", "robot", "pr2.urdf", Pose([0, 0, 0]))
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

    # Broadcast all object frames
    pub = rospy.Publisher(name=u"kitchen_item_frames", data_class=dagap_msgs.msg.KitchenObjectLocation, queue_size=10)
    element: OPMObjectQuery
    for element in query_object_list_sink:
        rospy.loginfo("Demo: checking pose for object: {}".format(element.Object))
        if not element.Object=="robot":
            rospy.loginfo("Demo: Attempting broadcast of tf for object: {}".format(element.Object))

            message: dagap_msgs.msg.KitchenObjectLocation

            # Set pose in current reference frame (iai_kitchen/sink_area_surface)
            current_element_posestamped = geometry_msgs.msg.PoseStamped()
            current_element_posestamped.header.frame_id = "iai_kitchen/" + reference_frame
            current_element_posestamped.pose = element.object_location

            message = dagap_msgs.msg.KitchenObjectLocation()
            message.Object = element.Object
            message.object_location = current_element_posestamped
            pub.publish(message)

    with simulated_robot:
        # Send request to DAGAP service
        res = opm_dagap_client(reference_frame=reference_frame,
                               object_list=query_object_list_map)
        rospy.loginfo("Received answer:")
        print(res)
        rospy.loginfo("Finished cleanly.")
