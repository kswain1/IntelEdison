#!/usr/bin/python
import mraa
from SF_9DOF import IMU
from math import *
from scipy.integrate import trapz
import serial
import numpy as np
import time as tm
import sys
import select
import termios
import tty
import requests

# IMU SAMPLES AT 100 HZ/ 100 samples per second
# WE ARE WORKING IN METERS NOT FEET!


class KemanImu():

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

        theta1 = asin(-ax / g) # please check if the units are right on this
        theta2 = atan(ay / az)

        # TODO: PLEASE REMEMBER TO START TRACING FROM HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Calculate initial euler parameters
        e4_0 = sqrt(1 + cos(theta1) + cos(theta2) + cos(theta1) * cos(theta2)) / 2
        e1_0 = (sin(theta2) * (1 + cos(theta1))) / (4 * e4_0)
        e2_0 = (sin(theta1) * (1 + cos(theta2))) / (4 * e4_0)
        e3_0 = (-sin(theta1) * sin(theta2)) / (4 * e4_0)

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
        accelVec[0] = (imu.ax * 9.81) * 1.401 # Constants are for 2G mode
        accelVec[1] = (imu.ay * 9.81) * 1.401
        accelVec[2] = (imu.az * 9.81) * 1.401

        accelVec[0] = round(accelVec[0], 3)
        accelVec[1] = round(accelVec[1], 3)
        accelVec[2] = round(accelVec[2], 3)



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
        angularVelocityVec[0] = (imu.gx * (pi/180)) + 0.045 #Should the angular velocity equation be in radians/sec or degrees/sec?
        angularVelocityVec[1] = (imu.gy * (pi/180)) - 0.170
        angularVelocityVec[2] = (imu.gz * (pi/180)) + 0.207

        angularVelocityVec[0] = round(angularVelocityVec[0], 3)
        angularVelocityVec[1] = round(angularVelocityVec[1], 3)
        angularVelocityVec[2] = round(angularVelocityVec[2], 3)
        print angularVelocityVec[2]

        return angularVelocityVec

    def computeDirectionCosineMatrix(e):
        """Computes Direction Cosine Matrix from Euler Parameters
        Returns 3x3 numpy array
        :param e:
        :return:
        """

        e1 = e[1]
        e2 = e[2]
        e3 = e[3]
        e4 = e[0]

        # Using the definition from the Bat Physics Paper
        cosineMatrix = np.zeros([3, 3])

        # TODO: MUST FIX ARRAY INDEXING
        cosineMatrix[0][0] = e1**2 - e2**2 - e3**2 + e4**2
        cosineMatrix[0][1] = 2 * (e1*e2 + e3*e4)
        cosineMatrix[0][2] = 2 * (e1*e3 - e2*e4)
        cosineMatrix[1][0] = 2 * (e1*e2 - e3*e4)
        cosineMatrix[1][1] = e2**2 - e1**2 - e3**2 + e4**2
        cosineMatrix[1][2] = 2 * (e2*e3 + e1*e4)
        cosineMatrix[2][0] = 2 * (e1*e3 + e2*e4)
        cosineMatrix[2][1] = 2 * (e2*e3 - e1*e4)
        cosineMatrix[2][2] = e3**2 - e1**2 - e2**2 + e4**2

        return cosineMatrix

    def computeEulerParameters(e_current, timeVector, currentAngularVelocity):
        """Computes the orientation at each sampled instant of time during the swing trial.
        Waits for IMU interrupt which signals IMU has undergone sufficient acceleration
        Returns direction cosine/rotation matrix at each sampled-instant time
        which is used to compute the kinematic path
        :param: e_current: current euler parameters; for t0 e_current is e_initial
        :param: previousSampleTime: Time at which the last sample was taken
        :return:
        """



        xAngularVelocity = currentAngularVelocity[0]
        yAngularVelocity = currentAngularVelocity[1]
        zAngularVelocity = currentAngularVelocity[2]
        angularVelocityMagnitude = sqrt(xAngularVelocity**2
                                        +yAngularVelocity**2
                                        +zAngularVelocity**2)
        elapsedTime = timeVector[1]

        # Compute new quaternion
        q = np.zeros(4) #TODO: WHY IS THIS NAMED Q AND NOT OMEGA??
        q[0] = cos(angularVelocityMagnitude * elapsedTime / 2)
        q[1] = sin(angularVelocityMagnitude * elapsedTime / 2) *(xAngularVelocity/angularVelocityMagnitude)
        q[2] = sin(angularVelocityMagnitude * elapsedTime / 2) *(yAngularVelocity/angularVelocityMagnitude)
        q[3] = sin(angularVelocityMagnitude * elapsedTime / 2) *(zAngularVelocity/angularVelocityMagnitude)

        # TODO: This may not be needed
        # TODO: E4 == E0!!!!
        q0 = q[0]  # q1
        q1 = q[1]  # q2
        q2 = q[2]  # q3
        q3 = q[3]  # q4

        # Define quaternion multiplication matrix
        quaternionMultiplicationMatrix = np.zeros([4, 4])
        quaternionMultiplicationMatrix[0][0] = q0  # ???? Why are there q's here and not omegas.
        quaternionMultiplicationMatrix[0][1] = -q1
        quaternionMultiplicationMatrix[0][2] = -q2
        quaternionMultiplicationMatrix[0][3] = -q3
        quaternionMultiplicationMatrix[1][0] = q1
        quaternionMultiplicationMatrix[1][1] = q0
        quaternionMultiplicationMatrix[1][2] = q3
        quaternionMultiplicationMatrix[1][3] = -q2
        quaternionMultiplicationMatrix[2][0] = q2
        quaternionMultiplicationMatrix[2][1] = -q3
        quaternionMultiplicationMatrix[2][2] = q0
        quaternionMultiplicationMatrix[2][3] = q1
        quaternionMultiplicationMatrix[3][0] = q3
        quaternionMultiplicationMatrix[3][1] = q2
        quaternionMultiplicationMatrix[3][2] = -q1
        quaternionMultiplicationMatrix[3][3] = q0

        # New quaternion computed through multiplication
        newEulerParameters = np.dot(quaternionMultiplicationMatrix, e_current)  # Dot is used to multiply


        # TODO:Do we have to normalize the quaternion?
        # TODO:Can we use this same solver or do we have to switch

        #normalEulerParameters = normalizeEulerParameters(eulerParameters)

        return newEulerParameters

    def computeInertialAcceleration(imu, orientMat):
        """ Computes the inertial frame (field frame) acceleration
        according to equation 12 in the paper
        Returns a 3x1 numpy column vector with (x,y,z) inertial acceleration components
        :param imu:
        :param orientMat:
        :return:
        """

        g = -9.81  # m/s^2 Remember to change if we switch to ft/s^2
        #TODO: ADD G CORRECTIVE FACTOR FOR CALIBRATION

        # print orientMat

        localAcceleration = readAcceleration(imu)  # TODO: This may be replaced with a local acceleration parameter
        ax = localAcceleration[0]
        ay = localAcceleration[1]
        az = localAcceleration[2]



        #inertialAcceleration = np.dot(orientMat.transpose(), localAcceleration) - (g * np.array([0, 0, 1]).transpose())

        #Create Direction Matrix Transpose
        orientMatTranspose = np.zeros([3, 3])  # Initialize array with equal dimensions
        orientMatTranspose[0][0] = orientMat[0][0]
        orientMatTranspose[0][1] = orientMat[1][0]
        orientMatTranspose[0][2] = orientMat[2][0]
        orientMatTranspose[1][0] = orientMat[0][1]
        orientMatTranspose[1][1] = orientMat[1][1]
        orientMatTranspose[1][2] = orientMat[2][1]
        orientMatTranspose[2][0] = orientMat[0][2]
        orientMatTranspose[2][1] = orientMat[1][2]
        orientMatTranspose[2][2] = orientMat[2][2]

        #Perform matrix multiplication
        inertialAcceleration = np.zeros(3)
        inertialAcceleration[0] = orientMatTranspose[0][0]*ax + orientMatTranspose[0][1]*ay + orientMatTranspose[0][2]*az
        inertialAcceleration[1] = orientMatTranspose[1][0]*ax + orientMatTranspose[1][1]*ay + orientMatTranspose[1][2]*az
        inertialAcceleration[2] = (orientMatTranspose[2][0]*ax + orientMatTranspose[2][1]*ay + orientMatTranspose[2][2]*az) - g
        #inertialAcceleration[2] = inertialAcceleration[2] - g # Compensate for gravity

        #Compensate for gravity
        #inertialAcceleration[0] -= 0.25 # STRICTLY EXPERIMENTAL NO THEORY
        #inertialAcceleration[1] += 0.45 # STRICTLY EXPERIMENTAL NO THEORY
        #inertialAcceleration[2] -= (-9.81)


        # print "inertial Acceleration:", inertialAcceleration
        # print "direction matrix:", orientMat

        xinertialAcceleration = inertialAcceleration[0]
        yinertialAcceleration = inertialAcceleration[1]
        zinertialAcceleration = inertialAcceleration[2]

        return xinertialAcceleration, yinertialAcceleration, zinertialAcceleration

    def computeInertialVelocity(inertialAccelerationVec,sampleTimes):
        """Computes the inertial frame (field frame) velocity by numerical integration
        Returns a 3x1 numpy column vector with (x,y,z) inertial velocity components
        :param imu:
        :param inertialAcceleration:
        :return:
        """

        # Find velocity at various points in time. The current setup yields velocity


        print "In computeInertial Velocity Function"
        print inertialAccelerationVec
        print "Length of inertialVec", len(inertialAccelerationVec)
        print "Length of timeVector", len(sampleTimes)

        InertialVelocity = trapz(inertialAccelerationVec, sampleTimes)  # I Beez in the trap


        #InertialVelocity = np.array([xInertialVelocity,
        #                             yInertialVelocity,
        #                             zInertialVelocity])

        return InertialVelocity.tolist()

    def computeVelocity(accelerationVector, timeVector):
        """
        Function computes Velocity using averages between the acceleration values
        :param accelerationVector:
        :param timeVector:
        :return:
        """

        velocityHistory = [0]
        index = 1
        print "The time vector is:"
        print timeVector
        while index < len(accelerationVector):

            velocity_final = velocityHistory[index - 1] + ((accelerationVector[index] + accelerationVector[index-1])/2)*timeVector[index]
            index = index + 1
            velocityHistory.append(velocity_final)

        return velocityHistory

    def computePosition(inertialVelocity, sampleTimes):
        #params pass in the velocity of the component of choice
        #returns the position vector

        posVector = [0]
        index = 1
        while index < len(sampleTimes):

            position_final = posVector[index - 1] + ((inertialVelocity[index] + inertialVelocity[index-1])/2)*sampleTimes[index]
            index = index + 1
            posVector.append(position_final)


        return posVector

    def computeVelocityMagnitude(xVelocity, yVelocity, zVelocity):
        """Computes angular velocity vector magnitude
        :param velocity:
        :return:
        """

        velocityMagnitudeVector  = [0]

        index = 0
        for number in range(0, len((xVelocity))-1):

            xVel = xVelocity[index]
            yVel = yVelocity[index]
            zVel = zVelocity[index]

            velocityMagnitudeVector.append(sqrt((xVel ** 2) + (yVel ** 2) + (zVel ** 2))) #Compute Magnitude
            index = index + 1


        return velocityMagnitudeVector

    def normalizeAngularVelocityVector(angularVelocity):
        """Normalizes angular velocity vector so that its magnitude
        may be 1
        :param angularVelocity:
        :return:
        """

        angularVelocityMagnitude = \
            sqrt(angularVelocity[0] ** 2 + angularVelocity[1] ** 2 +
                 angularVelocity[2] ** 2 + angularVelocity[3] ** 2)

        normalizedAngularVelocity = np.asarray(angularVelocity)  # Convert to numpy array to perform element wise operation

        normalizedQuaternion = angularVelocity / angularVelocityMagnitude

        #print angularVelocityMagnitude

        return normalizedQuaternion

    def computeSweetSpotVelocity(inertialVelocityVector, angularVelocityVector):
        """Computes sweet spot velocity on the bat
        Returns a 3x1 numpy column vector with (x,y,z) sweet spot velocity components
        :param imu:
        :return:
        """

        #For each element in the x-velocity vector
        index = 0
        localVelocity = np.zeros([3, 3])
        angularVelocity = np.zeros([3, 3])
        sweetSpotVector = []

        for x in inertialVelocityVector[0]:

            localVelocity = [inertialVelocityVector[0][index],
                             inertialVelocityVector[1][index],
                             inertialVelocityVector[2][index]]

            angularVelocity = [angularVelocityVector[0][index],
                               angularVelocityVector[1][index],
                               angularVelocityVector[2][index]]

            sweetSpotDistance = 0.7  # meters TODO:VERIFY SWEET SPOT DISTANCE
            sweetDistanceVector = np.array([1, 0, 0])
            sweetSpotVelocity = localVelocity + np.cross(angularVelocity, sweetDistanceVector)
            sweetSpotVelMag = sqrt(sweetSpotVelocity[0]**2+sweetSpotVelocity[1]**2+sweetSpotVelocity[2]**2)
            index = index + 1
            sweetSpotVector.append(sweetSpotVelMag)


        return sweetSpotVector

    def normalizeEulerParameters(eulerParameters):
        """Normalizes the quaternion/euler parameters into a unit quaternion
        --That is a quaternion whose magnitude is equal to 1
        :param eulerParameters: 1x4 numpy array
        :return: 1x4 numpy array
        """

        quaternionMagnitude = \
            sqrt(eulerParameters[0]**2 + eulerParameters[1]**2 +
                 eulerParameters[2]**2 + eulerParameters[3]**2)

        eulerParameters = np.asarray(eulerParameters)  # Convert to numpy array to perform element wise operation

        normalizedQuaternion = eulerParameters/quaternionMagnitude

        return normalizedQuaternion

    def computeEulerParameters(e_current, timeVector, currentAngularVelocity):
        """Computes the orientation at each sampled instant of time during the swing trial.
        Waits for IMU interrupt which signals IMU has undergone sufficient acceleration
        Returns direction cosine/rotation matrix at each sampled-instant time
        which is used to compute the kinematic path
        :param: e_current: current euler parameters; for t0 e_current is e_initial
        :param: previousSampleTime: Time at which the last sample was taken
        :return:
        """



        xAngularVelocity = currentAngularVelocity[0]
        yAngularVelocity = currentAngularVelocity[1]
        zAngularVelocity = currentAngularVelocity[2]
        angularVelocityMagnitude = sqrt(xAngularVelocity**2
                                        +yAngularVelocity**2
                                        +zAngularVelocity**2)
        elapsedTime = timeVector[1]

        # Compute new quaternion
        q = np.zeros(4) #TODO: WHY IS THIS NAMED Q AND NOT OMEGA??
        q[0] = cos(angularVelocityMagnitude * elapsedTime / 2)
        q[1] = sin(angularVelocityMagnitude * elapsedTime / 2) *(xAngularVelocity/angularVelocityMagnitude)
        q[2] = sin(angularVelocityMagnitude * elapsedTime / 2) *(yAngularVelocity/angularVelocityMagnitude)
        q[3] = sin(angularVelocityMagnitude * elapsedTime / 2) *(zAngularVelocity/angularVelocityMagnitude)

        # TODO: This may not be needed
        # TODO: E4 == E0!!!!
        q0 = q[0]  # q1
        q1 = q[1]  # q2
        q2 = q[2]  # q3
        q3 = q[3]  # q4

        # Define quaternion multiplication matrix
        quaternionMultiplicationMatrix = np.zeros([4, 4])
        quaternionMultiplicationMatrix[0][0] = q0  # ???? Why are there q's here and not omegas.
        quaternionMultiplicationMatrix[0][1] = -q1
        quaternionMultiplicationMatrix[0][2] = -q2
        quaternionMultiplicationMatrix[0][3] = -q3
        quaternionMultiplicationMatrix[1][0] = q1
        quaternionMultiplicationMatrix[1][1] = q0
        quaternionMultiplicationMatrix[1][2] = q3
        quaternionMultiplicationMatrix[1][3] = -q2
        quaternionMultiplicationMatrix[2][0] = q2
        quaternionMultiplicationMatrix[2][1] = -q3
        quaternionMultiplicationMatrix[2][2] = q0
        quaternionMultiplicationMatrix[2][3] = q1
        quaternionMultiplicationMatrix[3][0] = q3
        quaternionMultiplicationMatrix[3][1] = q2
        quaternionMultiplicationMatrix[3][2] = -q1
        quaternionMultiplicationMatrix[3][3] = q0

        # New quaternion computed through multiplication
        newEulerParameters = np.dot(quaternionMultiplicationMatrix, e_current)  # Dot is used to multiply


        # TODO:Do we have to normalize the quaternion?
        # TODO:Can we use this same solver or do we have to switch

        #normalEulerParameters = normalizeEulerParameters(eulerParameters)

        return newEulerParameters









