#! /usr/bin/python

import rospy

if __name__ == u"__main__":
    rospy.init_node(u'dagap')
    rospy.loginfo('Starting dagap node')

    sleeper = rospy.Rate(10)
    rospy.spin()
