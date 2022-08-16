import os.path

import rosgraph.masterapi
import rosparam
import rospy
import pathlib
import sys
import xml.etree.ElementTree as ET


class Robot:
    def __init__(self):
        self.package_root = pathlib.Path(__file__).resolve().parents[3]
        self.model_path = self.package_root.joinpath("model")
        self.robot_description_file_path = self.model_path.joinpath("robot_description.xml")
        if not os.path.exists(self.model_path):
            # check if folder exists and create
            os.makedirs(self.model_path)

        try:
            robot_xml = rosparam.get_param(u'robot_description')
            tree = ET.parse(robot_xml)
            root = tree.getroot()
            print(root)
            # print(self.robot)
            # self.chain = self.compile_chain(self.robot)
            with open(self.robot_description_file_path, 'w') as f:
                f.write(self.robot)
                f.close()
        except rosgraph.masterapi.MasterError:
            rospy.logerr(u'No information about robot on param server. Closing.')
            sys.exit("No information about robot.")

    # def compile_chain(self, robot_description):
    #     """
    #
    #     :param robot_description: string from param server
    #     :type robot_description: string
    #     :return:
    #     """
    #     line_list = robot_description.splitlines()
    #     for line in line_list:
    #
    #     print(num_links)
    #     return 0


class Link:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.children = []

    def add_child(self, child_node):
        # adds parent-child relationship
        self.children.append(child_node)

    def remove_child(self, child_node):
        # removes parent-child relationship
        self.children = [child for child in self.children
                         if child is not child_node]
