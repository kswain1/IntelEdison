#!/usr/bin/python
import mraa
from SF_9DOF import IMU
from math import *
from scipy.integrate import trapz
import serial
import numpy as np
import time as tm
import socket
import sys
import select
import termios
import tty
import requests
from keman_imu import KemanImu

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
    # TODO: Try other modes as well
    imu.accel_mode(0b001)
    imu.gyro_mode(0b001)

    # Specify ranges for accelerometer, magnetometer and gyroscope
    # Accelerometer Options: "2G", "4G", "6G", "8G", "16G"
    # Magnetometer Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
    # Gyroscope Options: "245DPS", "500DPS", "2000DPS"

    imu.accel_range("16G")
    imu.mag_range("2GAUSS")
    imu.gyro_range("2000DPS")


    return imu

def roundEntries(list):
    """Rounds entries in list to the third decimal place"""
    index = 0
    for number in list:
        list[index] = round(number, 3)
        index = index + 1


    return list

def keyboard():
    global angle
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        c = sys.stdin.read(1)
        if c == '\x1b':
            exit(0)
        if c == '1':
            angle = 'stop'
        if c == '2':
            angle = 'kill'
        if c == '3':
            angle = 30
        if c == '4':
            angle = 60
        if c == '5':
            angle = 90
        if c == '6':
            angle = 120
        if c == '7':
            angle = 150
        if c == '8':
            angle = 180
        if c == '0':
            angle = 0
        if c == 'e':
            angle = 190
        if c == 'a':
            angle = 'stop'
        if c == 's':
            angle = False


    return angle

def streamSwingTrial():
    """Runs a swing trial event and computes important bat metrics
    Returns the bat metrics in an array
    """
    k = KemanImu()
    imu = initialize()
    print "5 seconds to Calibrate. Please hold Calibration Position:"
    tm.sleep(5.5)  # Wait for calibration position
    e_initial = k.calibrate(imu) # Obtain four initial euler parameters
    #e_initial = normalizeEulerParameters(e_initial) #Normalize

    initialTime = tm.time() # Time


    # Initialize Storage Vectors
    acceleration = k.readAcceleration(imu)
    angularVelocity = k.readAngularVelocity(imu)
    xinertialAccelerationVector = [0]
    yinertialAccelerationVector = [0]
    zinertialAccelerationVector = [0]
    velocityMagnitudeVector = [0]
    xAccelerationVector = [acceleration[0]]
    yAccelerationVector = [acceleration[1]]
    zAccelerationVector = [acceleration[2]]
    xAngularVelocity = [angularVelocity[0]]
    yAngularVelocity = [angularVelocity[1]]
    zAngularVelocity = [angularVelocity[2]]
    aimAngleVector = [0]
    rollVector = [0]
    rotationMatrices = [k.computeDirectionCosineMatrix(e_initial)]
    elevationAngles = [0]
    timeVectors = [0]
    sampleTimes = [0]
    calibration_angles = [0]

    # Initialize useful computation variables
    previousEpochTime = initialTime  # t0
    previousElapsedSampleTime = 0
    currentElapsedSampleTime = 0
    previousEulerParameters = e_initial
    index = 0
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Loop for 10 seconds
        input('press 1 to stop program\n press 2 to kill recording\n press 3 to start recording \n press 4 to record at 10 deg')
        isSwinging = False
        while (keyboard() != 'stop'):
            while (keyboard() != 'kill'):
                    #read callibration angles
                tm.sleep(.5)
                if ((keyboard() != 'kill') and (keyboard() != 'stop')):
                    calibration_angles.append(keyboard())

                    # Read Angular Velocity and Acceleration
                    currentAngularVelocity = k.readAngularVelocity(imu)
                    currentAcceleration = k.readAcceleration(imu)
                    xAccelerationVector.append(currentAcceleration[0])
                    yAccelerationVector.append(currentAcceleration[1])
                    zAccelerationVector.append(currentAcceleration[2])
                    xAngularVelocity.append(currentAngularVelocity[0])
                    yAngularVelocity.append(currentAngularVelocity[1])
                    zAngularVelocity.append(currentAngularVelocity[2])
                    currentEpochTime = tm.time()
                    currentElapsedSampleTime = currentEpochTime - previousEpochTime
                    sampleTimes.append(currentElapsedSampleTime)
                    timeVectors.append(previousElapsedSampleTime+currentElapsedSampleTime)  # Time History TODO: CHANGE NAME TO AVOID CONFUSION
                    timeVector = [0, currentElapsedSampleTime]


                    # TODO:Do we have to normalize the quaternion?
                    # TODO:Can we use this same solver or do we have to switch

                    # Solve for current rotation matrix
                    currentEulerParameters = k.computeEulerParameters(previousEulerParameters, timeVector, currentAngularVelocity)
                    eulerParametersNormalized = currentEulerParameters
                    #eulerPrametersNoramlized = normalizeEulerParameters(currentEulerParameters)

                    # Compute Direction Cosine Matrix
                    directionMatrix = k.computeDirectionCosineMatrix(eulerParametersNormalized)
                    rotationMatrices.append(directionMatrix)

                    #print "Direction Cosine Matrix:", directionMatrix[0]


                    # Get Inertial Acceleration snd Velocity
                    xinertialAcceleration, yinertialAcceleration, zinertialAcceleration = k.computeInertialAcceleration(imu, directionMatrix)
                    xinertialAccelerationVector.append(xinertialAcceleration)
                    yinertialAccelerationVector.append(yinertialAcceleration)
                    zinertialAccelerationVector.append(zinertialAcceleration)


                    # Stop collecting data once acceleration has reached zero again.
                    previousEulerParameters = currentEulerParameters
                    previousEpochTime = currentEpochTime
                    previousElapsedSampleTime += currentElapsedSampleTime  # move to next step

                    #Calculate Yaw, pitch and roll
                    elevationAngle = asin(directionMatrix[0][2]) * 57.3
                    aimAngle = atan(directionMatrix[0][1] / directionMatrix[0][0]) * 57.3
                    #roll = currentEulerParameters[3]**2 - currentEulerParameters[1]**2 \
                    #       - currentEulerParameters[2]**2 - currentEulerParameters[3]**2

                    #roll = acos(roll) * 57.3

                    roll = atan(directionMatrix[1][2]/directionMatrix[2][2]) * 57.3

                    elevationAngles.append(elevationAngle)
                    aimAngleVector.append(aimAngle)
                    rollVector.append(roll)
                    isSwinging = True


                # Compute Velocity
            if(isSwinging):
                xinertialVelocity = k.computeVelocityHistory(xinertialAccelerationVector, sampleTimes)
                yinertialVelocity = k.computeVelocityHistory(yinertialAccelerationVector, sampleTimes)
                zinertialVelocity = k.computeVelocityHistory(zinertialAccelerationVector, sampleTimes)

                xpositionVector = k.computePosition(xinertialVelocity, sampleTimes)
                ypositionVector = k.computePosition(yinertialVelocity, sampleTimes)
                zpositionVector = k.computePosition(zinertialVelocity, sampleTimes)

                #TODO: FIX THIS
                velocityMagnitude = k.computeVelocityMagnitude(xinertialVelocity, yinertialVelocity, zinertialVelocity)
                velocityMagnitudeVector.append(velocityMagnitude)
                sweetSpotVelocityVector = k.computeSweetSpotVelocity([xinertialVelocity, yinertialVelocity, zinertialVelocity],
                                                             [xAngularVelocity, yAngularVelocity, zAngularVelocity])


                roundEntries(yAccelerationVector)
                roundEntries(zAccelerationVector)
                roundEntries(xAngularVelocity)
                roundEntries(yAngularVelocity)
                roundEntries(zAngularVelocity)
                roundEntries(elevationAngles)
                roundEntries(timeVectors)
                roundEntries(xinertialVelocity)
                roundEntries(yinertialVelocity)
                roundEntries(zinertialVelocity)
                roundEntries(xinertialAccelerationVector)
                roundEntries(yinertialAccelerationVector)
                roundEntries(zinertialAccelerationVector)
                roundEntries(aimAngleVector)
                roundEntries(rollVector)
                roundEntries(sweetSpotVelocityVector)
                roundEntries(velocityMagnitude)
                roundEntries(xpositionVector)
                roundEntries(ypositionVector)
                roundEntries(zpositionVector)


                payload = {"accelx":xinertialAccelerationVector, "accely":yinertialAccelerationVector,
                       "accelz":yinertialAccelerationVector}

                r=requests.post('https://obscure-headland-45385.herokuapp.com/hips',json=payload)
                isSwinging = False
        # s.connect(('192.168.1.41', port))
        # transmitString = listToString(xAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(yAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(zAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(xAngularVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(yAngularVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(zAngularVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(elevationAngles)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(timeVectors)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(xinertialVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(yinertialVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(zinertialVelocity)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(xinertialAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(yinertialAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(zinertialAccelerationVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(aimAngleVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(rollVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(sweetSpotVelocityVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(velocityMagnitude)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(xpositionVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(ypositionVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(zpositionVector)
        # transmitString = transmitString + '!'
        # transmitString = transmitString + listToString(calibration_angles)
        #
        # sendData(transmitString)
        s.close()


    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

streamSwingTrial()
