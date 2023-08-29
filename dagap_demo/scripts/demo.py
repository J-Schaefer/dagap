#!/usr/bin/env python

from __future__ import print_function

from typing import List

# ROS Imports
import rospy
# from geometry_msgs.msg import Pose

# DAGAP Imports
from dagap_msgs.srv import *
from dagap_msgs.msg import *
import dagap.utils.tfwrapper as dagap_tf

# PyCRAM Imports
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.designator import ObjectDesignatorDescription
from pycram.pose import Pose


def opm_dagap_client(reference_frame: str, object_list: [OPMObjectQuery]) -> GetNextOPMObjectResponse:
    rospy.wait_for_service('dagap_opm_query')
    try:
        call_common_service = rospy.ServiceProxy('dagap_opm_query', GetNextOPMObject)
        srv = GetNextOPMObjectRequest(reference_frame, object_list)
        response = call_common_service(srv)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


class PickAndPlaceDemo:

    def __init__(self):
        self.reference_frame = "sink_area_surface"
        dagap_tf.init()  # call tfwrapper init()

        # Set up the bullet world
        self.world = BulletWorld()
        self.world.set_gravity([0, 0, -9.8])

        self.tfbroadcaster = TFBroadcaster()

        # Spawn ground plane
        self.plane = Object("floor", "environment", "plane.urdf", world=self.world)
        # plane.set_color([0, 0, 0, 1])

        # Spawn kitchen
        self.kitchen = Object("kitchen", "environment", "kitchen.urdf")
        # kitchen.set_color([0.2, 0, 0.4, 0.6])
        self.kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

        self.object_spawning_poses_sink: List[Pose] = [
            dagap_tf.list_to_pose([0, 0, 0], [0, 0, 0, 1]),  # robot, position empty
            dagap_tf.list_to_pose([0.2, -0.15, 0.1], [0, 0, 0, 1]),  # breakfast-cereal
            dagap_tf.list_to_pose([0.2, -0.35, 0.05], [0, 0, 0, 1]),  # cup
            dagap_tf.list_to_pose([0.20, -0.75, 0.05], [0, 0, 0, 1]),  # bowl
            dagap_tf.list_to_pose([0.15, -0.4, 0.1], [0, 0, 0, 1]),  # spoon
            dagap_tf.list_to_pose([0.07, -0.35, 0.1], [0, 0, 0, 1])  # milk
        ]

        # transform_sink_map = lookup_transform(frame, u'map')

        # Hint for type of element
        element: geometry_msgs.msg.Pose
        # Hint for type of list object_spawning_poses_map
        self.object_spawning_poses_map: List[geometry_msgs.msg.PoseStamped] = []

        # Transform poses from iai_kitchen/sink_area_surface to map for correct spawn pose
        kitchen_frame = self.kitchen.get_link_tf_frame("")
        for element in self.object_spawning_poses_sink:
            self.object_spawning_poses_map.append(
                dagap_tf.transform_pose(element, 'simulated/map', "simulated/" + kitchen_frame + self.reference_frame))

        self.object_names = [
            "robot",
            "breakfast-cereal",
            "cup",
            "bowl",
            "spoon",
            "milk"
        ]

        self.query_object_list_map: List[OPMObjectQuery] = [
            OPMObjectQuery(Object="robot", object_location=self.object_spawning_poses_sink[0]),
            OPMObjectQuery(Object="breakfast-cereal", object_location=self.object_spawning_poses_map[1].pose),
            OPMObjectQuery(Object="cup", object_location=self.object_spawning_poses_map[2].pose),
            OPMObjectQuery(Object="bowl", object_location=self.object_spawning_poses_map[3].pose),
            OPMObjectQuery(Object="spoon", object_location=self.object_spawning_poses_map[4].pose),
            OPMObjectQuery(Object="milk", object_location=self.object_spawning_poses_map[5].pose)
        ]

        self.query_object_list_sink: List[OPMObjectQuery] = [
            OPMObjectQuery(Object="robot", object_location=self.object_spawning_poses_sink[0]),
            OPMObjectQuery(Object="breakfast-cereal", object_location=self.object_spawning_poses_sink[1]),
            OPMObjectQuery(Object="cup", object_location=self.object_spawning_poses_sink[2]),
            OPMObjectQuery(Object="bowl", object_location=self.object_spawning_poses_sink[3]),
            OPMObjectQuery(Object="spoon", object_location=self.object_spawning_poses_sink[4]),
            OPMObjectQuery(Object="milk", object_location=self.object_spawning_poses_sink[5])
        ]

        # Spawn breakfast cereal
        self.breakfast_cereal = Object(self.query_object_list_map[1].Object,
                                  self.query_object_list_map[1].Object,
                                  path="breakfast_cereal.stl",
                                  pose=Pose(dagap_tf.point_to_list(self.query_object_list_map[1].object_location.position),
                                            frame="iai_kitchen/" + self.reference_frame))
        self.breakfast_cereal_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[1].Object])
        # Spawn cup
        self.cup = Object(self.query_object_list_map[2].Object,
                     self.query_object_list_map[2].Object,
                     path="../resources/cup.stl",
                     pose=Pose(dagap_tf.point_to_list(self.query_object_list_map[2].object_location.position),
                               frame="iai_kitchen/" + self.reference_frame))
        self.cup_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[2].Object])
        # Spawn bowl
        self.bowl = Object(self.query_object_list_map[3].Object,
                      self.query_object_list_map[3].Object,
                      path="bowl.stl",
                      pose=Pose(dagap_tf.point_to_list(self.query_object_list_map[3].object_location.position),
                                frame="iai_kitchen/" + self.reference_frame))
        self.bowl_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[3].Object])
        # Spawn spoon
        self.spoon = Object(self.query_object_list_map[4].Object,
                       self.query_object_list_map[4].Object,
                       path="spoon.stl",
                       pose=Pose(dagap_tf.point_to_list(self.query_object_list_map[4].object_location.position),
                                 frame="iai_kitchen/" + self.reference_frame))
        self.spoon_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[4].Object])
        # Spawn milk
        self.milk = Object(self.query_object_list_map[5].Object,
                      self.query_object_list_map[5].Object,
                      path="milk.stl",
                      pose=Pose(dagap_tf.point_to_list(self.query_object_list_map[5].object_location.position),
                                frame="iai_kitchen/" + self.reference_frame))
        self.milk_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[5].Object])

        # Spawn PR2 robot
        self.pr2 = Object("pr2", "robot", "pr2.urdf", Pose([0, 0, 0]))
        self.robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

        self.query_object_list_map[1].object_frame = ("simulated/"
                                                      + self.breakfast_cereal.get_link_tf_frame(link_name="").replace("/", ""))
        self.query_object_list_map[2].object_frame = ("simulated/"
                                                      + self.cup.get_link_tf_frame(link_name="").replace("/", ""))
        self.query_object_list_map[3].object_frame = ("simulated/"
                                                      + self.bowl.get_link_tf_frame(link_name="").replace("/", ""))
        self.query_object_list_map[4].object_frame = ("simulated/"
                                                      + self.spoon.get_link_tf_frame(link_name="").replace("/", ""))
        self.query_object_list_map[5].object_frame = ("simulated/"
                                                      + self.milk.get_link_tf_frame(link_name="").replace("/", ""))

        # Test out an example transform to catch exceptions early
        if dagap_tf.lookup_transform("simulated/" + self.kitchen.get_link_tf_frame("sink_area_surface"),
                                     "simulated/" + self.bowl.tf_frame):
            rospy.loginfo("Found transform")
        else:
            rospy.logwarn("Did not find transform")

    def get_designator_from_name(self, object_name: str) -> ObjectDesignatorDescription:
        if self.object_names[0] == object_name:
            return self.robot_desig
        if self.object_names[1] == object_name:
            return self.breakfast_cereal_desig
        if self.object_names[2] == object_name:
            return self.cup_desig
        if self.object_names[3] == object_name:
            return self.bowl_desig
        if self.object_names[4] == object_name:
            return self.spoon_desig
        if self.object_names[5] == object_name:
            return self.milk_desig

    def get_object_from_name(self, object_name: str) -> Object:
        if self.object_names[0] == object_name:
            return self.pr2
        if self.object_names[1] == object_name:
            return self.breakfast_cereal
        if self.object_names[2] == object_name:
            return self.cup
        if self.object_names[3] == object_name:
            return self.bowl
        if self.object_names[4] == object_name:
            return self.spoon
        if self.object_names[5] == object_name:
            return self.milk

    def get_name_from_frame(self, frame: str) -> str:
        for name in self.object_names:
            if name in frame:
                return name
        rospy.logwarn("[get_name_from_frame]: Could not find name.")
        return ""

    def run(self):
        rospy.loginfo("Running demo.")
        with simulated_robot:
            # Send request to DAGAP service
            rospy.set_param(param_name='robot_root',
                            param_value="simulated/" + self.pr2.get_link_tf_frame(link_name=""))

            object_list = self.query_object_list_map

            while len(object_list) > 1:
                # service return frame not the name
                res: GetNextOPMObjectResponse = opm_dagap_client(reference_frame=self.reference_frame,
                                                                 object_list=object_list)
                rospy.loginfo("Received answer:")
                print(res)

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                MoveTorsoAction([0.33]).resolve().perform()

                next_object_name = self.get_name_from_frame(res.next_object)
                next_object_desig: ObjectDesignatorDescription = self.get_designator_from_name(next_object_name)

                # Remove current object from list for next iteration
                for element in list(object_list):
                    if element.Object == next_object_name:
                        object_list.remove(element)

                pickup_pose = CostmapLocation(target=next_object_desig.resolve(),
                                              reachable_for=self.robot_desig).resolve()
                pickup_arm = pickup_pose.reachable_arms[0]

                NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

                # TODO: change arm according to DAGAP output
                # TODO: drive to kitchen first and ask DAGAP again for the arm
                PickUpAction(object_designator_description=next_object_desig, arms=[pickup_arm],
                             grasps=["front"]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                place_island = SemanticCostmapLocation("kitchen_island_surface", self.kitchen_desig.resolve(),
                                                       next_object_desig.resolve()).resolve()

                place_stand = CostmapLocation(place_island.pose, reachable_for=self.robot_desig,
                                              reachable_arm=pickup_arm).resolve()

                NavigateAction(target_locations=[place_stand.pose]).resolve().perform()

                PlaceAction(next_object_desig,
                            target_locations=[place_island.pose],
                            arms=[pickup_arm]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()



if __name__ == "__main__":
    print("Starting demo")

    Demo = PickAndPlaceDemo()  # init demo and spawn objects
    Demo.run()  # run demo
