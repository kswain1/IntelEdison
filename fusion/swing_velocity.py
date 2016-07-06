from SF_9DOF import IMU
import time
import sys
import select
import nunmpy as np



import tty
import termios

#This script uses numpy in order to perform matrix operations

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
    imu.enable_temp()

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

    theta1 = arcsin(-ax/g)
    theta2 = arctan(ay/az)

    #Calculate initial euler parameters
    e1 = sin(theta2)*(1+cos(theta1))/(4*e4)
    e2 = sin(theta1)*(1+cos(theta2))/(4*e4)
    e3 = -sin(theta1)*sin(theta2)/(4*e4)


def readAcceleration(imu):
    imu.read_accel()
    accelVec = np.shape(3, 1) # 3x1 Column Vector
    accelVec(1, 1) = imu.ax
    accelVec(2, 1) = imu.ay
    accelVec(3, 1) = imu.az
    return accelVec


#Begin by calibrating
imu = initialize()
accelVec = readAcceleration(imu)






