import os.path

import rosgraph.masterapi
import rosparam
import rospy
import pathlib
import sys


class Robot:
    def __init__(self):
        self.package_root = pathlib.Path(__file__).resolve().parents[3]
        self.model_path = self.package_root.joinpath("model")
        self.robot_description_file_path = self.model_path.joinpath("robot_description.xml")
        if not os.path.exists(self.model_path):
            # check if folder exists and create
            os.makedirs(self.model_path)

        try:
            self.robot = rosparam.get_param(u'robot_description')
            with open(self.robot_description_file_path, 'w') as f:
                f.write(self.robot)
                f.close()
        except rosgraph.masterapi.MasterError:
            rospy.logerr(u'No information about robot on param server. Closing.')
            sys.exit("No information about robot.")
