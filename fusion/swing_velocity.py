from SF_9DOF import IMU
import numpy as np
from math import *
#import termios

#This script uses numpy in order to perform matrix operations
#So the numpy library must be downloaded otherwise script won't work
#Calibrate
#Begin sampling acceleration
def initialize():
    #Returns  initialized IMU object
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
    #PLEASE REMEMBER TO CHECK GRAVITATIONAL CONSTANT UNITS

    #Begin by computing theta1 and theta2
    accelVec = readAcceleration(imu)
    ax = accelVec(1, 1)
    ay = accelVec(2, 1)
    az = accelVec(3, 1)
    g = 9.81 #Gravitational constant m/s^2 may change to ft/s^2

    theta1 = asin(-ax/g)
    theta2 = atan(ay/az)

    #Calculate initial euler parameters
    e4_0 = sqrt(1+cos(theta1)+ cos(theta2) + cos(theta1)*cos(theta2)/2)
    e1_0 = sin(theta2)*(1+cos(theta1))/(4*e4_0)
    e2_0 = sin(theta1)*(1+cos(theta2))/(4*e4_0)
    e3_0 = -sin(theta1)*sin(theta2)/(4*e4_0)

    return [e1_0,e2_0,e3_0,e4_0]

def readAcceleration(imu):
    imu.read_accel()
    accelVec = np.shape(3, 1) # 3x1 Column Vector
    accelVec(1, 1) = imu.ax
    accelVec(2, 1) = imu.ay
    accelVec(3, 1) = imu.az
    return accelVec

def stateEquationModel(e,e_initial, t_inital,t_final,wx,wy,wz):
    #e is a vector containing e1 through e4
    #e_inital contains initial conditions for the euler angles

    de1 = e[4]*wx - e[3]*wy + e[2]*wz
    de2 = e[3]*wx + e[4]*wy - e[1]*wz
    de3 = -e[2]*wx + e[1]*wy + e[4]*wz
    de4 = -e[1]*wx - e[2]*wy - e[3]*wy

    return [de1,de2,de3,de4]


#Initialize
imu = initialize()

#Begin calibration procedure
#Calibrate returns the four initial euler parameters
#which are needed in order to solve for cosine matrix
e_initial = calibrate(imu)

#Compute Angular velocity


#Compute Direction Cosine Matrix



#accelVec = readAcceleration(imu)









