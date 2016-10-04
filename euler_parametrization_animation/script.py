#!/usr/bin/env python
#
# How-to execute this script
# --------------------------
# - Method 1: `$ pyton ./script.py`
# - Method 2: `$ ./script.py` (needs to be executable)
#
# To use a custom 3D object, just inject it's class. These
# objects can be created programatically (look at the
# rectangular prism file) or statically (look at the
# triangular prism file).
#
# Omar Trejo
# October, 2016
#

from triangular_prism import TriangularPrism
from euler_parametrization import EulerParametrization

# To use the default rectangular prism use this:
# e = EulerParametrization()

# To inject a new 3D object of your choosing use this:
e = EulerParametrization(rotation_data_file='accel.csv')

_ = e.animation()
e.show()
