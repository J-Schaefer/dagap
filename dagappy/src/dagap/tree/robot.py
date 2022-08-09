import rosparam
import rospy
import pathlib


class Robot:
    def __init__(self):
        self.package_root = pathlib.Path(__file__).resolve().parents[3]
        self.robot_description_file_path = pathlib.Path(__file__).resolve().parents[3].joinpath("model/robot_description.xml")
        # TODO: check if folder exists and create
        self.robot = rosparam.get_param(u'robot_description')
        with open(self.robot_description_file_path, 'w') as f:
            f.write(self.robot)
            f.close()
        pass
