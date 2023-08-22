import rospy
from py_trees import Behaviour
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from numpy.linalg import norm
from dagap.utils.tfwrapper import *
import giskardpy.utils.tfwrapper as tfw

class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()

    def decide(self, object, action, robot, opm_action: bool, reference_frame: str = u"", object_frame: str = None):
        """

        :param object: list of frame names
        :param action: string
        :param robot: Robot Object
        :param opm_action: Boolean if action comes from OPM
        :param reference_frame
        :param object_frame
        :return:
        """
        if action == u"one hand task":
            # start to perform one handed task
            # check distance to estimate which hand to use

            if opm_action:
                task_object = [object[0]]
            else:
                task_object = object

            try:
                if object_frame:
                    task_object = [object_frame]
                else:
                    rospy.logwarn("DAGAP: Couldn't find object frame.")
            except:
                rospy.logwarn("DAGAP: Types don't seem to fit.")


            try:
                if len(task_object) == 1:
                    # start to perform one handed task
                    # check distance to estimate which hand to use
                    object_frame = task_object[0]
                    rospy.loginfo("Calculating distance to frame " + object_frame)  # TODO: add prints for more objects
                    pose_list = []
                    distance = []

                    # Add tf_root for simulated robot in bullet world
                    if robot.tf_root:
                        robot.gripper_list = ['{0}{1}'.format(robot.tf_root, gripper) for gripper in robot.gripper_list]

                    for gripper in robot.gripper_list:
                        # FIXME: get tf prefix and concatenate, right now frame cannot be found
                        pose = PoseStamped()
                        pose.pose = list_to_pose(*object[1])
                        pose.header.frame_id = u"iai_kitchen/" + reference_frame
                        new_frame: PoseStamped = tfw.transform_msg(gripper, pose)  # transform pose to new frame
                        # TODO: calculate transform instead of transforming pose
                        trans = new_frame.pose.position
                        rot = new_frame.pose.orientation
                        pose_list.append([trans, rot])
                        distance.append(norm(point_to_list(trans)))  # add distance of current gripper to object to list

                    min_index = distance.index(min(distance))
                    winner = robot.gripper_list[min_index]  # get gripper with smallest distance
                    pose_winner = pose_list[min_index]  # get pose with smallest distance
                    rospy.loginfo("Found closest gripper: " + winner)
                elif len(task_object) == 2:
                    # More than one object with
                    rospy.logwarn("Not implemented yet. Returning empty.")
                    pose = PoseStamped()
                    return pose

                else:
                    rospy.logwarn("Too many objects. Returning empty.")
                    pose = PoseStamped()
                    return pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not find transform.")

            if opm_action:
                return [object_frame, winner]
            else:
                # print(self.manipulation_cases.get(sentence, "Not found"))
                GraspPose = TransformStamped()
                GraspPose.header.frame_id = winner
                GraspPose.transform = list_to_transform(pose_winner[0], pose_winner[1])

                return [GraspPose]



        elif action == u"one hand support":
            pass

        elif action == u"fixed offset":
            pass
