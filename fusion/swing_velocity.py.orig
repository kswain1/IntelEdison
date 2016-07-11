from SF_9DOF import IMU
<<<<<<< HEAD
import time
import sys
import select
import numpy as np
=======
import numpy as np
from math import *
from scipy.integrate import odeint
>>>>>>> 08eb9df3045e816ada5be95b92de43e9b09bd53b


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
    imu = IMU()  # To select a specific I2C port, use IMU(n). Default is 1.

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
    # User has 5 seconds to complete.
    # PLEASE REMEMBER TO CHECK GRAVITATIONAL CONSTANT UNITS

    # Begin by computing theta1 and theta2
    accelVec = readAcceleration(imu)
<<<<<<< HEAD
    print accelVec
    ax = accelVec(1, 1)
    ay = accelVec(2, 1)
    az = accelVec(3, 1)
    g = 9.81 #Gravitational constant m/s^2 may change to ft/s^2
=======
    ax = accelVec[1, 1]
    ay = accelVec[2, 1]
    az = accelVec[3, 1]
    g = 9.81  # Gravitational constant m/s^2 may change to ft/s^2

    theta1 = asin(-ax / g)
    theta2 = atan(ay / az)
>>>>>>> 08eb9df3045e816ada5be95b92de43e9b09bd53b

    # Calculate initial euler parameters
    e4_0 = sqrt(1 + cos(theta1) + cos(theta2) + cos(theta1) * cos(theta2) / 2)
    e1_0 = sin(theta2) * (1 + cos(theta1)) / (4 * e4_0)
    e2_0 = sin(theta1) * (1 + cos(theta2)) / (4 * e4_0)
    e3_0 = -sin(theta1) * sin(theta2) / (4 * e4_0)

    return [e1_0, e2_0, e3_0, e4_0]

    #additions by Kehlin Swain 
    print e1,e2,e3


def readAcceleration(imu):
    imu.read_accel()
<<<<<<< HEAD
    accelVec = [imu.ax, imu.ay, imu.az]
    accelVec = np.array(accelVec)
    accelVec.reshape(3,1) 
    ## Changes edited by Kehlin Swain for gathering Accel Vector Information
    #accelVec = np.reshape(3, 1) # 3x1 Column Vector
    #accelVec(1, 1) = imu.ax
    #accelVec(2, 1) = imu.ay
    #accelVec(3, 1) = imu.az
=======
    accelVec = np.shape(3, 1)  # 3x1 Column Vector
    accelVec[1, 1] = imu.ax
    accelVec[2, 1] = imu.ay
    accelVec[3, 1] = imu.az
>>>>>>> 08eb9df3045e816ada5be95b92de43e9b09bd53b
    return accelVec


def readAngularVelocity(imu):
    # angularVelocityVec is a 3x1 Column Vector
    # angularVelocityVec[1-3] = x y z component respectively
    imu.read_gyro()
    angularVelocityVec = np.shape(3, 1)  # 3x1 Column Vector
    angularVelocityVec[1, 1] = imu.gx
    angularVelocityVec[2, 1] = imu.gy
    angularVelocityVec[3, 1] = imu.gy
    return angularVelocityVec


def stateEquationModel(e, angularVelocity):
    # e is a vector containing e1 through e4
    # angularVelocity is a vector containing the component velocities
    w = angularVelocity

    de1 = e[4] * w[1] - e[3] * w[2] + e[2] * w[3]
    de2 = e[3] * w[1] + e[4] * w[2] - e[1] * w[3]
    de3 = -e[2] * w[1] + e[1] * w[2] + e[4] * w[3]
    de4 = -e[1] * w[1] - e[2] * w[2] - e[3] * w[3]

    return [de1, de2, de3, de4]

# def computeDirectionCosineMatrix


# Initialize
imu = initialize()
<<<<<<< HEAD
accelVec = readAcceleration(imu)
calibrate(imu)
=======

# Begin calibration procedure
# Calibrate returns the four initial euler parameters
# which are needed in order to solve for cosine matrix
e_initial = calibrate(imu)
>>>>>>> 08eb9df3045e816ada5be95b92de43e9b09bd53b

# Read Angular velocity
angularVelocity = readAngularVelocity(imu)

# Create time vector
time = np.linspace(0.0, 0.01, 2)

# Solve for euler parameters
e = odeint(stateEquationModel, e_initial, time)

# Compute Direction Cosine Matrix

accelVec = readAcceleration(imu)
