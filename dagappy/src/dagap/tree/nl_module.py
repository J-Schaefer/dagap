import rospy
import yaml
from yaml.loader import SafeLoader
import pathlib
from py_trees import Behaviour
import nltk


class NLModule(Behaviour):
    # semantic_command = None
    # package_root = None
    # config_file_path = None

    def __init__(self, name):
        super(NLModule, self).__init__()
        # self.semantic_command = command
        nltk.download('punkt')
        rospy.loginfo("Initialising NL module.")
        self.package_root = pathlib.Path(__file__).resolve().parents[3]
        self.config_file_path = pathlib.Path(__file__).resolve().parents[3].joinpath("config/semantic.yaml")
        if not self.config_file_path.is_file():
            rospy.logerr("Config file does not exist")
        else:
            with open(self.config_file_path) as f:
                self.config_data = yaml.load(f, Loader=SafeLoader)
                rospy.loginfo(self.config_data)

    def define_action(self, command):
        """
        Define the meaning of the entered command
        :type command: string
        :return string
        """
        try:
            print(self.config_data[command])
            res = self.config_data[command]  # check if exact word sequence is on yaml config
        except KeyError:
            tokens = nltk.word_tokenize(command)
            rospy.loginfo("Tokens are: ")
            rospy.loginfo(tokens)
            # TODO: process tokens and try to find fitting action
        return res
