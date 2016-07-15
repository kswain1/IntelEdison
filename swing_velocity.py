#!/usr/bin/python
from SF_9DOF import IMU
from math import *
from scipy.integrate import odeint
import numpy as np
import time as tm


# import termios
# This script uses numpy in order to perform matrix operations
# So the numpy library must be downloaded otherwise script won't work
# Calibrate
# Begin sampling acceleration
# IMU SAMPLES AT 100 HZ/ 100 samples per second
# WE ARE WORKING IN METERS NOT FEET!



def initialize():
    # Returns  initialized IMU object

    # Initialization code
    # Create IMU object
    imu = IMU(1)  # To select a specific I2C port, use IMU(n). Default is 1.

    # Initialize IMU
    imu.initialize()

    # Enable accel, mag, gyro, and temperature
    imu.enable_accel()
    imu.enable_mag()
    imu.enable_gyro()

    # Specify Options: "2G", "4G", "6G", "8G", "16G"
    imu.accel_range("2G")  # leave blank for default of "2G"

    # Specify Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
    imu.mag_range("2GAUSS")  # leave blank for default of "2GAUSS"

    # Specify Options: "245DPS", "500DPS", "2000DPS"
    imu.gyro_range("245DPS")  # leave blank for default of "245DPS"
    return imu


def calibrate(imu):
    # IMU must calibrate by pointing the tip of the bat
    # in the (I_hat x K_hat) plane of the field frame.
    # Returns the four initial euler parameters
    # User has 5 seconds to complete.
    # PLEASE REMEMBER TO CHECK GRAVITATIONAL CONSTANT UNITS

    # Begin by computing theta1 and theta2
    accelVec = readAcceleration(imu)
    ax = accelVec[0][0]
    ay = accelVec[1][0]
    az = accelVec[2][0]
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
    # Returns 3x1 numpy Column Vector with acceleration values

    imu.read_accel()
    accelVec = np.zeros((3, 1))  # 3x1 Column Vector
    accelVec[0][0] = imu.ax
    accelVec[1][0] = imu.ay
    accelVec[2][0] = imu.az

    return accelVec


def readAngularVelocity(imu):
    # angularVelocityVec is a 3x1 Column Vector
    # Returns 3x1 numpyVc[1-3] = x y z component respectively

    imu.read_gyro()
    angularVelocityVec = np.zeros(3)  # 3x1 Column Vector
    angularVelocityVec[0] = imu.gx
    angularVelocityVec[1] = imu.gy
    angularVelocityVec[2] = imu.gy

    return angularVelocityVec


def stateEquationModel(e, t, w0, w1, w2):
    # e is a vector containing e1 through e4
    # angularVelocity is a vector containing the component velocities
    # Returns a list with the four differential euler parameter equations
    w = [w0, w1, w2]

    de1 = e[3] * w[0] - e[2] * w[1] + e[1] * w[2]
    de2 = e[2] * w[0] + e[3] * w[1] - e[0] * w[2]
    de3 = -e[1] * w[0] + e[0] * w[1] + e[3] * w[2]
    de4 = -e[0] * w[0] - e[1] * w[1] - e[2] * w[2]

    return [de1, de2, de3, de4]


def computeDirectionCosineMatrix(e):
    # Computes Direction Cosine Matrix from Euler Parameters
    # Returns 3x3 numpy array

    e1 = e[0]
    print e1
    e2 = e[1]
    print e1
    e3 = e[2]
    e4 = e[3]

    # Using the definition from the Bat Physics Paper
    cosineMatrix = np.zeros([3, 3])
    #MUST FIX INDEXING
    cosineMatrix[1, 1] = e1**2 - e2**2 - e3**2 + e4**2
    cosineMatrix[1, 2] = 2 * (e1*e2 + e3*e4)
    cosineMatrix[1, 3] = 2 * (e2*e3 - e1*e4)
    cosineMatrix[2, 1] = 2 * (e1*e2 - e3*e4)
    cosineMatrix[2, 2] = e2**2 - e1**2 - e3**2 + e4**2
    cosineMatrix[2, 3] = 2 * (e2*e3 + e1*e4)
    cosineMatrix[3, 1] = 2 * (e1*e3 + e2*e4)
    cosineMatrix[3, 2] = 2 * (e2*e3 - e1*e4)
    cosineMatrix[3, 3] = e3**2 - e1**2 - e2**2 + e4**2

    return cosineMatrix


def streamSwingTrial():
    # Initialize
    imu = initialize()

    # Calibrate returns the four initial euler parameters
    # which are needed in order to solve for cosine matrix
    e_initial = calibrate(imu)

    #Init time object
    initialTime = tm.time()

    # Read Angular velocity
    angularVelocity = readAngularVelocity(imu)

    # Read time at which sample was read (elapsed time)
    sampleTime = tm.time() - initialTime

    # Create time vector
    time = [0.0, sampleTime]

    # Solve for euler parameter
    e = odeint(stateEquationModel, e_initial, time, (imu.ax, imu.ay, imu.az))
    eCurrent = e.tolist()[1]
    print eCurrent

    # Compute Direction Cosine Matrix
    directionMatrix = computeDirectionCosineMatrix(e)


streamSwingTrial()