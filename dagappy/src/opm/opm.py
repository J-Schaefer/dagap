#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Petra Wenzl
"""

import random
# from collections import Counter
from scipy.spatial.distance import euclidean


class OPM:

    def __init__(self):
        pass

    def predict_next_action(self, object_information, dimension=2):
        '''
        Predict the next action based on weighted cost.

        Parameters
        ----------
        object_information : list of lists
            List of objects used in the sequence with spatial information.
            Default: [['object1', [[x, y, z], [orientation]]],
                      'object2', [[x, y, z], [orientation]]]
        dimension : list [int, str], optional
            Dimension in which to consider distances. The default is [2, ].

        Returns
        -------
        predicted_action : str
            Model-generated next action (object to interact with).
        '''

        # get spatial info from object_information
        objects = [elem[0] for elem in object_information
                   if elem[0] != 'robot']
        sequence = objects
        coordinates = {elem[0]: elem[1][0] for elem in object_information
                       if elem[0] in objects}
        start_coordinates = [elem[1][0] for elem in object_information
                             if elem[0] == 'robot']
        start_coordinates = start_coordinates * 2
        containment = []

        # set category parameters
        strong_k = ['tray', 'placemat']
        mid_k = ['plate', 'napkin', 'bowl']

        value_c = 1.8
        value_k = 0.2
        value_mid_k = 0.3
        # value_food_k = 1.7

        c = {obj: value_c if obj in containment else 1.0 for obj in objects}
        k = {obj: value_k if obj in strong_k else value_mid_k if obj in mid_k
             else 1.0 for obj in objects}

        i = 0
        # generated_sequence = []
        possible_items = dict.fromkeys(objects, 0)
        # item_count = Counter(objects)

        coord_index = 0

        #new_coords, new_start_coords = self.filter_for_dimension(coordinates,                                                            coordinates,
        #                                                         start_coordinates)
        
        new_coords = {k: v[:-1] for k, v in coordinates.items()}
        
        new_start_coords = [coords[:-1] for coords in start_coordinates]
        
        while i < 1:
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
            # generated_sequence.append(prediction)

            # if item_count[sequence[i]] > 1:
            #     item_count[sequence[i]] = item_count[sequence[i]] - 1
            # else:
            #     del possible_items[sequence[i]]

            # coord_index += 1
            i += 1

            predicted_item = [elem for elem in object_information
                              if elem[0] == prediction]

        return predicted_item[0]

    def filter_for_dimension(self, coordinates, start_coordinates,
                             dimension=2):
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

        print('filter: coords ', coordinates)
        print('filter: start ', start_coordinates)

        new_coords = {key: value[:-1] for key, value in coordinates.items()}
        new_start_coords = [x[:-1] for x in start_coordinates]

        return new_coords, new_start_coords
