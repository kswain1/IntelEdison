#
# Euler Parametrization
# ---------------------
# - roll  : degrees, rotate on Y
# - pitch : degrees, rotate on X
# - yaw   : degrees, rotate on Z
#
# Omar Trejo
# September, 2016
#

import math
import numpy
import pandas
import plotly
import itertools

from matplotlib import pyplot
from matplotlib import animation
from transforms3d import euler
from mpl_toolkits.mplot3d import Axes3D

import utilities


class EulerParametrization(object):

    def __init__(self,
                 data_file=None,
                 rectangle_color='b',
                 axes_size_factor=1.5,
                 interval=30,
                 jupyter=False):
        self._current_index = 0
        if not data_file:
            #data_file = "./data.csv"
            data_file = "accel_ROLLPITCHYAW.csv"
        self._rotation_data = pandas.read_csv(data_file)
        self._axes_size_factor = axes_size_factor
        self._rectangle_color = rectangle_color
        self._jupyter = jupyter
        self._interval = interval
        self._setup_3D_object_properties()
        self._setup_3D_object_data()
        self._setup_figure_and_axes()

    @property
    def number_of_rows(self):
        return(self._rotation_data.shape[0])

    def animation(self, jupyter_notebook=False):
        return(animation.FuncAnimation(
            self._figure,
            self._animate,
            frames=self.number_of_rows,
            interval=self._interval,
            repeat=True
        ))

    def show(self):
        self._figure.show()

    def rotation(self, iteration):
        roll, pitch, yaw = list(self._rotation_data.loc[iteration])
        if not self._jupyter:
            print("{0}, Roll: {1:.3f}, Pitch: {2:.3f}, Yaw: {3:.3f}".format(
                iteration, roll, pitch, yaw
            ))
        #
        # TODO: Do roll, pitch, and yaw seem to behave correctly?
        #       There are various ways of setting up the model
        #       and I need to verify this with Kehlin:
        #       - http://mathworld.wolfram.com/EulerAngles.html
        #       - https://en.wikipedia.org/wiki/Euler_angles
        #       - http://matthew-brett.github.io/transforms3d/gimbal_lock.html#example
        #
        return(roll, pitch, yaw)

    def interactive_plot(self, roll, pitch, yaw):
        self._setup_3D_object_data()
        self._rotate_3D_object_data(roll, pitch, yaw)
        self._add_correct_sides_to_plot()

    def _animate(self, index):
        self._rotate_3D_object_data(*self.rotation(index))
        self._add_correct_sides_to_plot()

    def _add_correct_sides_to_plot(self):
        self._remove_lines_in_plot()
        for index, line in enumerate(self._object_data):
            start, end = line
            if self._approximately_correct_distance(start, end):
                self._axes.plot3D(*zip(start, end), color=self._rectangle_color)

    def _remove_lines_in_plot(self):
        self._axes.lines = []

    def _setup_3D_object_data(self):
        #
        # This creates the 3D object
        # If you want a different shape, make the change here
        #
        data = list(itertools.product(self._length, self._width, self._height))
        data = list(itertools.combinations(numpy.array(data), 2))
        data = utilities.list_of_tuples_to_list_of_lists(data)
        self._object_data = data

    def _setup_3D_object_properties(self, long_s=10, medium_s=6, short_s=2):
        self._adjacent_distances = [2 * long_s, 2 * medium_s, 2 * short_s]
        self._length = [-long_s, long_s]
        self._width  = [-medium_s, medium_s]
        self._height = [-short_s, short_s]
        self._long_side = long_s

    def _setup_figure_and_axes(self):
        figure = pyplot.figure()
        axes = figure.gca(projection='3d')
        axes.set_aspect('equal')
        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.set_zlabel('Z')

        length = self._axes_size_factor * self._long_side
        axes.set_xlim((-length, length))
        axes.set_ylim((-length, length))
        axes.set_zlim((-length, length))

        self._figure = figure
        self._axes = axes

    def _rotate_3D_object_data(self, roll, pitch, yaw):
        for idx, line in enumerate(self._object_data):
            self._object_data[idx][0] = self._rotate_vector(line[0], roll, pitch, yaw)
            self._object_data[idx][1] = self._rotate_vector(line[1], roll, pitch, yaw)

    def _rotate_vector(self, vector, roll, pitch, yaw, degrees=True):
        #
        # vector := x
        # b := rotated_vector
        # rotation_matrix := A (R)
        # A*x = b  ~~ rotation_matrix.dot(vector)
        #
        if degrees:
            pitch = utilities.degrees_to_radians(pitch)
            roll = utilities.degrees_to_radians(roll)
            yaw = utilities.degrees_to_radians(yaw)
        rotation_matrix = euler.euler2mat(roll, pitch, yaw, 'sxyz')
        rotated_vector = rotation_matrix.dot(vector)
        return(rotated_vector)

    def _approximately_correct_distance(self, start, end):
        line_distance = math.sqrt(sum(pow(start - end, 2)))
        for adjacent_distance in self._adjacent_distances:
            if abs(line_distance - adjacent_distance) < 0.1:
                return(True)
        return(False)
