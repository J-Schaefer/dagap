import pymesh

# Class to examine mesh of the object
# Functionality: find CoG, predict mass, find thinner parts to grasp
class Mesher:
    def __init__(self, mesh_name):
        self.mesh_name = mesh_name
        self.object = pymesh.load_mesh(self.mesh_name)

    def get_cog(self):
        # TODO:
        pass