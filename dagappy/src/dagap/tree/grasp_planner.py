import rospy
from py_trees import Behaviour
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from numpy.linalg import norm
from dagap.utils.tfwrapper import *

class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()

    def decide(self, object, action, robot, opm_action: bool, reference_frame: str = u""):
        """

        :param object: list of frame names
        :param action: string
        :param robot: Robot Object
        :param opm_action: Boolean if action comes from OPM
        :param reference_frame
        :return:
        """
        if action == u"one hand task":
            # if len(object) == 1:
                # start to perform one handed task
                # check distance to estimate which hand to use

                if opm_action:

                    self.tfb.sendTransform(object[1][0],
                                           object[1][1],
                                           rospy.Time.now(),
                                           object[0],
                                           u"iai_kitchen/" + reference_frame)
                    task_object = [object[0]]
                else:
                    task_object = object

                try:
                    if len(task_object) == 1:
                    # start to perform one handed task
                    # check distance to estimate which hand to use
                        frame = task_object[0]
                        rospy.loginfo("Calculating distance to " + frame)  # TODO: add prints for more objects
                        pose_list = []
                        distance = []
                        for gripper in robot.gripper_list:
                            # FIXME: get tf prefix and concatenate, right now frame cannot be found
                            (trans, rot) = self.tfl.lookupTransform(gripper, frame, rospy.Time(0))
                            pose_list.append([trans, rot])
                            distance.append(norm(trans))
                        winner = robot.gripper_list[distance.index(min(distance))]
                        pose_winner = pose_list[distance.index(min(distance))]
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
                    return [task_object, winner]
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
