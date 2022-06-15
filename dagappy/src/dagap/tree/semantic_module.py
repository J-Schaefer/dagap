import nltk
import rospy
import yaml
from yaml.loader import SafeLoader
import pathlib
from py_trees import Behaviour
import os.path

class SemanticModule(Behaviour):
    semantic_command = None
    package_root = None
    config_file_path = None

    def __init__(self, name):
        super(SemanticModule, self).__init__()
        # self.semantic_command = command
        self.package_root = pathlib.Path(__file__).resolve().parents[3]
        self.config_file_path = pathlib.Path(__file__).resolve().parents[3].joinpath("config/semantic.yaml")
        if not self.config_file_path.is_file():
            rospy.logerr("Config file does not exist")
        else:
            with open(self.config_file_path) as f:
                self.config_data = yaml.load(f, Loader=SafeLoader)
                print(self.config_data)
