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

import pandas

from matplotlib import pyplot
from matplotlib import animation
from transforms3d import euler
from mpl_toolkits.mplot3d import Axes3D

import utilities

from rectangular_prism import RectangularPrism


class EulerParametrization(object):

    def __init__(self,
                 rotation_data_file=None,
                 object_3D=RectangularPrism(),
                 interval=5,
                 jupyter=False):
        self._current_index = 0
        if not rotation_data_file:
            rotation_data_file = "accel_ROLLPITCHYAW.csv"
        self.rotation_data = pandas.read_csv(rotation_data_file)
        self.jupyter = jupyter
        self.interval = interval
        self.object_3D = object_3D
        self.data = self.object_3D.data
        self._setup_figure_and_axes()

    @property
    def number_of_rows(self):
        return(self.rotation_data.shape[0])

    def animation(self, jupyter_notebook=False):
        return(animation.FuncAnimation(
            self._figure,
            self._animate,
            frames=self.number_of_rows,
            interval=self.interval,
            repeat=True
        ))

    def show(self):
        pyplot.show(block=True)

    def get_rotation(self, iteration):
        roll, pitch, yaw = list(self.rotation_data.loc[iteration])
        if not self.jupyter:
            print("{0}, Roll: {1:.3f}, Pitch: {2:.3f}, Yaw: {3:.3f}".format(
                iteration, roll, pitch, yaw
            ))
        return(roll, pitch, yaw)

    def interactive_plot(self, roll, pitch, yaw):
        self._rotate_3D_object_data(roll, pitch, yaw)
        self._update_plot_lines()

    def _animate(self, index):
        self._rotate_3D_object_data(*self.get_rotation(index))
        self._update_plot_lines()

    def _rotate_3D_object_data(self, roll, pitch, yaw):
        self.data = self.object_3D.data
        for idx, line in enumerate(self.data):
            self.data[idx][0] = self._rotate_vector(line[0], roll, pitch, yaw)
            self.data[idx][1] = self._rotate_vector(line[1], roll, pitch, yaw)

    def _update_plot_lines(self):
        self._remove_lines_in_plot()
        for index, line in enumerate(self.data):
            start, end = line
            self._axes.plot3D(*zip(start, end), color=self.object_3D.color)

    def _remove_lines_in_plot(self):
        self._axes.lines = []

    def _setup_figure_and_axes(self):
        figure = pyplot.figure()
        axes = figure.gca(projection='3d')
        axes.set_aspect('equal')
        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.set_zlabel('Z')

        length = self.object_3D.axis_length
        axes.set_xlim((-length, length))
        axes.set_ylim((-length, length))
        axes.set_zlim((-length, length))

        self._figure = figure
        self._axes = axes

    def _rotate_vector(self, vector, roll, pitch, yaw, degrees=True):
        if degrees:
            pitch = utilities.degrees_to_radians(pitch)
            roll = utilities.degrees_to_radians(roll)
            yaw = utilities.degrees_to_radians(yaw)
        rotation_matrix = euler.euler2mat(roll, pitch, yaw, 'sxyz')
        rotated_vector = rotation_matrix.dot(vector)
        return(rotated_vector)
