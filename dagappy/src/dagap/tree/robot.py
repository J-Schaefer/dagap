import rosparam
import rospy
import pathlib


class Robot:
    def __init__(self):
        self.robot = rosparam.get_param(u'robot_description')
        with open(u'robot_description.xml', 'w') as f:
            f.write(self.robot)
            f.close()
        pass
