#! /usr/bin/python

import rospy
from dagap.dagap import DAGAP

if __name__ == u"__main__":
    rospy.init_node(u'dagap')
    rospy.loginfo('Starting dagap node')

    dagap = DAGAP()

    sleeper = rospy.Rate(10)
    rospy.spin()
