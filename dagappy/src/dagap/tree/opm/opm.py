import random
from collections import Counter
from scipy.spatial.distance import euclidean

'''
Author: Petra Wenzl
Example testing values
objects = ['plate', 'cup']
coordinates = {'plate': (0,0,0), 'cup': (1,1,1)}
sequence = ['plate', 'cup']
start_coordinates = [[0,0,1], [1,1,0]]
containment = ['plate']
'''


class OPM:

    def __init__(self):
        pass

    def predict_sequence(self, objects, coordinates, sequence, start_coordinates,
                         containment, dimension=2):
        '''
        Generate a sequence of actions based on weighted cost.
        Parameters
        ----------
        objects : list
            Objects in episode.
        coordinates : dictionary
            Coordinates of objects.
        sequence : list
            List of actions/objects in sequence.
        start_coordinates : list
            List of coordinates where subject is standing before each action.
        containment: list
            List of all objects being stored in closed locations (drawers etc.).
        dimension : list [int, str], optional
            Dimension in which to consider distances. The default is [3, ].
        Returns
        -------
        predicted_sequence : list
            Model-generated sequence of actions (objects to interact with).
        '''

        # set category parameters
        strong_k = ['tray', 'placemat']
        mid_k = ['plate', 'napkin']

        value_c = 1.8
        value_k = 0.2
        value_mid_k = 0.3
        value_food_k = 1.7

        c = {obj: value_c if obj in containment else 1.0 for obj in objects}
        k = {obj: value_k if obj in strong_k else value_mid_k if obj in mid_k
        else 1.0 for obj in objects}
        # print('k: ', k)
        # print('c: ', c)

        i = 0
        generated_sequence = []
        possible_items = dict.fromkeys(objects, 0)  # generate dict from obj list
        item_count = Counter(objects)

        coord_index = 0

        new_coords, new_start_coords = filter_for_dimension(coordinates,
                                                            start_coordinates,
                                                            dimension)

        while i < len(sequence):
            for obj in possible_items.keys():
                try:
                    position = tuple(new_start_coords[coord_index])
                except TypeError:
                    position = str(new_start_coords[coord_index])

                distance = euclidean(position, new_coords[obj])

                possible_items[obj] = distance ** k[obj] * c[obj]
                # print(obj, possible_items[obj])

            minval = min(possible_items.values())
            minval = [k for k, v in possible_items.items() if v == minval]
            # choose prediction randomly if multiple items have same cost
            minval = random.choice(minval)

            prediction = minval
            generated_sequence.append(prediction)

            if item_count[sequence[i]] > 1:
                item_count[sequence[i]] = item_count[sequence[i]] - 1
            else:
                del possible_items[sequence[i]]

            coord_index += 1
            i += 1

        return generated_sequence


    def filter_for_dimension(self, coordinates, start_coordinates, dimension=2):
        '''
        Filter coordinates and start coordinates for given dimension
        (e.g., xyz -> xy).
        Parameters
        ----------
        dimension : int, default: 2
            Dimension for which to adapt coordinates
            (default before filtering: 3D).
        coordinates : dictionary
            Coordinates of objects in 3D.
        start_coordinates : list
            List of start coordinates where subject is standing before next
            picking_up action in 3D.
        Returns
        -------
        new_coords : dictionary
            Dictionary with filtered coordinates.
        new_start_coords : list
            List with filtered start coordinates.
        '''

        new_coords = {}
        new_start_coords = []

        new_coords = {key: value[:-1] for key, value in coordinates.items()}
        new_start_coords = [x[:-1] for x in start_coordinates]

        return new_coords, new_start_coords
