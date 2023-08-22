#!/usr/bin/env python
import roslib
import rospy
from dagap.utils.tfwrapper import point_to_tuple, quaternion_to_tuple
import geometry_msgs
import tf
import dagap_msgs.msg
from typing import List

object_list: List[dagap_msgs.msg.KitchenObjectLocation] = []

def update_object_frame(pose: geometry_msgs.msg.PoseStamped, parent_frame: str, frame_name: str):
    # Add tf broadcaster for objects
    rospy.loginfo("object_frame_broadcaster: Broadcasting transform for object {} in parent frame {}.".format(frame_name, parent_frame))
    # TODO: check if tf2 works better
    br = tf.TransformBroadcaster()
    br.sendTransform(translation=point_to_tuple(pose.pose.position),
                     rotation=quaternion_to_tuple(pose.pose.orientation),
                     time=rospy.Time.now(),
                     child=frame_name,
                     parent=parent_frame)


def callback_kitchen_objects(msg: dagap_msgs.msg.KitchenObjectLocation):
    rospy.loginfo("object_frame_broadcaster: Kitchen object tf broadcaster received message with object name: {}.".format(msg.Object))

    update_object_frame(pose=msg.object_location,
                        parent_frame=msg.object_location.header.frame_id,
                        frame_name=msg.Object)


if __name__ == '__main__':
    rospy.init_node('dagap_kitchen_object_publisher')
    rospy.loginfo("object_frame_broadcaster: Registering node dagap_kitchen_object_publisher")
    rospy.Subscriber(name=u"kitchen_item_frames",
                     data_class=dagap_msgs.msg.KitchenObjectLocation,
                     callback=callback_kitchen_objects)
    rospy.spin()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
