#!/usr/bin/python
from SF_9DOF import IMU
from math import *
from scipy.integrate import odeint
from scipy.integrate import trapz

import numpy as np
import time as tm

# IMU SAMPLES AT 100 HZ/ 100 samples per second
# WE ARE WORKING IN METERS NOT FEET!


def initialize():
    """Creates and initializes the IMU object

    Returns an IMU object
    """

    # Create IMU object
    imu = IMU()  # To select a specific I2C port, use IMU(n). Default is 1.
    imu.initialize()

    # Enable accelerometer, magnetometer, gyroscope
    imu.enable_accel()
    imu.enable_mag()
    imu.enable_gyro()

    # Change IMU buffer mode to Bypass
    imu.accel_mode(0b000)
    imu.gyro_mode(0b000)

    # Specify ranges for accelerometer, magnetometer and gyroscope
    # Accelerometer Options: "2G", "4G", "6G", "8G", "16G"
    # Magnetometer Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
    # Gyroscope Options: "245DPS", "500DPS", "2000DPS"
    imu.accel_range("2G")
    imu.mag_range("2GAUSS")
    imu.gyro_range("245DPS")

    return imu


def calibrate(imu):
    """Calibrates the metric program. Batter must point the tip of the bat
    in the (I_hat x K_hat) plane of the field frame as demonstrated
    in the figures in the paper. User has 5 seconds to complete.
    Returns the four initial euler parameters.

    :param imu:
    :return:
    """

    # TODO: PLEASE REMEMBER TO CHECK GRAVITATIONAL CONSTANT UNITS
    # Begin by computing theta1 and theta2
    accelVec = readAcceleration(imu)  # TODO: Verify that angular acceleration units are correct
    ax = accelVec[0]
    ay = accelVec[1]
    az = accelVec[2]
    g = 9.81  # Gravitational constant m/s^2 may change to ft/s^2

    theta1 = asin(-ax / g)
    theta2 = atan(ay / az)

    # Calculate initial euler parameters
    e4_0 = sqrt(1 + cos(theta1) + cos(theta2) + cos(theta1) * cos(theta2) / 2)
    e1_0 = sin(theta2) * (1 + cos(theta1)) / (4 * e4_0)
    e2_0 = sin(theta1) * (1 + cos(theta2)) / (4 * e4_0)
    e3_0 = -sin(theta1) * sin(theta2) / (4 * e4_0)

    return [e1_0, e2_0, e3_0, e4_0]


def readAcceleration(imu):
    """Obtains accelerometer sample from IMU
    The accelerometer measures linear acceleration

    Returns a 3x1 numpy Column Vector with (x,y,z) linear acceleration components
    :param imu:
    :return:
    """

    imu.read_accel()
    accelVec = np.zeros(3)  # 3x1 Column Vector
    accelVec[0] = imu.ax
    accelVec[1] = imu.ay
    accelVec[2] = imu.az

    return accelVec


def readAngularVelocity(imu):
    """ Obtains gyroscope sample from IMU
    The gyroscope measures angular velocity

    Returns a 3x1 numpy Column Vector with (x,y,z) angular velocity components
    :param imu:
    :return:
    """

    imu.read_gyro()
    angularVelocityVec = np.zeros(3)  # 3x1 Column Vector
    angularVelocityVec[0] = imu.gx
    angularVelocityVec[1] = imu.gy
    angularVelocityVec[2] = imu.gy

    return angularVelocityVec


def stateEquationModel(e, t, w0, w1, w2):
    """Method provides the state equation model of the swing physics modeled
    in the paper. This model contains the system of differential equations
    that must be solved with the ode solver.

    Returns derivatives of the euler parameters
    :param e: a vector containing e1 through e4
    :param t:
    :param w0: x-component angular velocity
    :param w1: y-component angular velocity
    :param w2: z-component angualr velocity
    :return:
    """
    # Returns a list with the four differential euler parameter equations
    w = [w0, w1, w2]

    de1 = e[3] * w[0] - e[2] * w[1] + e[1] * w[2]
    de2 = e[2] * w[0] + e[3] * w[1] - e[0] * w[2]
    de3 = -e[1] * w[0] + e[0] * w[1] + e[3] * w[2]
    de4 = -e[0] * w[0] - e[1] * w[1] - e[2] * w[2]

    return [de1, de2, de3, de4]


def computeDirectionCosineMatrix(e):
    """Computes Direction Cosine Matrix from Euler Parameters

    Returns 3x3 numpy array
    :param e:
    :return:
    """

    e1 = e[0]
    e2 = e[1]
    e3 = e[2]
    e4 = e[3]

    # Using the definition from the Bat Physics Paper
    cosineMatrix = np.zeros([3, 3])

    # TODO: MUST FIX ARRAY INDEXING
    cosineMatrix[0][0] = e1**2 - e2**2 - e3**2 + e4**2
    cosineMatrix[0][1] = 2 * (e1*e2 + e3*e4)
    cosineMatrix[0][2] = 2 * (e2*e3 - e1*e4)
    cosineMatrix[1][0] = 2 * (e1*e2 - e3*e4)
    cosineMatrix[1][1] = e2**2 - e1**2 - e3**2 + e4**2
    cosineMatrix[1][2] = 2 * (e2*e3 + e1*e4)
    cosineMatrix[2][0] = 2 * (e1*e3 + e2*e4)
    cosineMatrix[2][1] = 2 * (e2*e3 - e1*e4)
    cosineMatrix[2][2] = e3**2 - e1**2 - e2**2 + e4**2

    return cosineMatrix


def computeInertialAcceleration(imu, orientMat):
    """ Computes the inertial frame (field frame) acceleration
    according to equation 12 in the paper

    Returns a 3x1 numpy column vector with (x,y,z) inertial acceleration components
    :param imu:
    :param orientMat:
    :return:
    """

    g = 9.81  # m/s^2 Remember to change if we switch to ft/s^2

    localAcceleration = readAcceleration(imu)  # TODO: This may be replaced with a local acceleration parameter
    inertialAcceleration = np.dot(orientMat.transpose(), localAcceleration) - g * np.array([0, 0, 1])

    return inertialAcceleration


def computeInertialVelocity(imu, inertialAcceleration, sampleTimes):
    """Computes the inertial frame (field frame) velocity by numerical integration

    Returns a 3x1 numpy column vector with (x,y,z) inertial velocity components
    :param imu:
    :param inertialAcceleration:
    :return:
    """

    xInertialAcceleration = inertialAcceleration[0]
    yInertialAcceleration = inertialAcceleration[1]
    zInertialAcceleration = inertialAcceleration[2]

    xInertialVelocity = trapz(xInertialAcceleration, sampleTimes)  # Beez in the trap
    yInertialVelocity = trapz(yInertialAcceleration, sampleTimes)
    zInertialVelocity = trapz(zInertialAcceleration, sampleTimes)

    InertialVelocity = np.array([xInertialAcceleration,
                                 yInertialAcceleration,
                                 zInertialAcceleration])

    return InertialVelocity


def computeSweetSpotVelocity(imu, localVelocity, angularVelocity):
    """Computes sweet spot velocity on the bat
    Returns a 3x1 numpy column vector with (x,y,z) sweet spot velocity components
    :param imu:
    :return:
    """
    sweetSpotDistance = 0.7  # meters
    sweetDistanceVector = np.array([1, 0, 0])
    sweetSpotVelocity = localVelocity + np.cross(angularVelocity,sweetDistanceVector)






def streamSwingTrial():
    """Runs a swing trial event and computes important bat metrics

    Returns the bat metrics in an array
    """

    # Initialize
    imu = initialize()

    # Obtain four initial euler parameters to solve for cosine matrix
    print "5 seconds to Calibrate. Please hold Calibration Position:"

    tm.sleep(5.5)  # Wait for calibration position
    e_initial = calibrate(imu)
    # TODO:Do we have to normalize the quaternion?
    print "5 seconds to place in desired position:"

    tm.sleep(5.5)  # User may place in desired position

    # Init time object
    initialTime = tm.time()

    # Read Angular velocity
    # angularVelocity = readAngularVelocity(imu)

    # Read time at which sample was read (elapsed time)
    sampleTime = tm.time() - initialTime  # TODO:check if sample times are correct
    print sampleTime

    # Create time vector
    time = [0.0, sampleTime]

    # Solve for euler parameter
    e = odeint(stateEquationModel, e_initial, time, (imu.ax, imu.ay, imu.az))
    # TODO:Do we have to normalize the quaternion?
    eCurrent = e.tolist()[1]  # Assign 2nd entry of e array to eCurrent

    # Compute Direction Cosine Matrix
    directionMatrix = computeDirectionCosineMatrix(eCurrent)
    print "Direction Cosine Matrix:"
    print directionMatrix

    # Get Inertial Acceleration snd Velocity
    inertialAcceleration = computeInertialAcceleration(imu, directionMatrix)
    inertialVelocity = computeInertialVelocity(imu, inertialAcceleration, time)





streamSwingTrial()