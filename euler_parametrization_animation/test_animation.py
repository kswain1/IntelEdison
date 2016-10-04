from euler_parametrization import EulerParametrization as E
import time

e = E('accel_ROLLPITCHYAW.csv')
e.animation()
e.show()

time.sleep(10)