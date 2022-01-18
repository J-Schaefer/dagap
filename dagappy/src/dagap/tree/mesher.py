import pymesh
from py_trees import Behaviour


# Class to examine mesh of the object
# Functionality: find CoG, predict mass, find thinner parts to grasp
class Mesher(Behaviour):
    def __init__(self, name):
        super(Mesher, self).__init__()
        # self.mesh_name = mesh_name
        # self.object = pymesh.load_mesh(self.mesh_name)

    def get_cog(self):
        # TODO:
        pass

    def make_urdf(self):
        pass
