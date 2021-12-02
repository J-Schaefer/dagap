#!/usr/bin/python

import rospy

if __name__ == u'main':
    rospy.loginfo("Starting node")
    rospy.init_node(u'dagap')

    sleeper = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Entering loop")
            sleeper.sleep()
        except KeyboardInterrupt:
            break
