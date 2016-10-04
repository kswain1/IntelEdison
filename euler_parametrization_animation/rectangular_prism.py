
import copy
import math
import numpy
import itertools

import utilities


class RectangularPrism(object):

    def __init__(self,
                 long_s=10,
                 medium_s=6,
                 short_s=2,
                 color='b',
                 axis_length_factor=1.5):
        self.color = color
        self.long_side = long_s
        self.axis_length_factor = axis_length_factor
        self.length = [-long_s, long_s]
        self.width  = [-medium_s, medium_s]
        self.height = [-short_s, short_s]
        self.adjacent_distances = [
            2 * long_s,
            2 * medium_s,
            2 * short_s
        ]
        self._setup_3D_object_data()
        self._keep_only_correct_sides()

    @property
    def data(self):
        return(copy.deepcopy(self.object_data))

    @data.setter
    def data(self, value):
        self.object_data = value

    @property
    def color(self):
        return(self.object_color)

    @color.setter
    def color(self, value):
        self.object_color = value

    @property
    def axis_length(self):
        return(self.axis_length_factor * self.long_side)

    def _setup_3D_object_data(self):
        data = list(itertools.product(self.length, self.width, self.height))
        data = list(itertools.combinations(numpy.array(data), 2))
        data = utilities.list_of_tuples_to_list_of_lists(data)
        self.object_data = data

    def _keep_only_correct_sides(self):
        self.indexes_to_be_removed = []
        for index, line in enumerate(self.object_data):
            start, end = line
            if not self._only_one_coordinate_change(start, end):
                self.indexes_to_be_removed.append(index)
        self.object_data = [
            i for j, i in enumerate(self.object_data)
            if j not in self.indexes_to_be_removed
        ]

    def _approximately_correct_distance(self, start, end):
        line_distance = math.sqrt(sum(pow(start - end, 2)))
        for adjacent_distance in self.adjacent_distances:
            if abs(line_distance - adjacent_distance) < 0.1:
                return(True)
        return(False)

    def _only_one_coordinate_change(self, start, end):
        number_of_changed_dimensions = 0
        for dimension in range(len(start)):
            if start[dimension] != end[dimension]:
                number_of_changed_dimensions += 1
        return(number_of_changed_dimensions <= 1)
