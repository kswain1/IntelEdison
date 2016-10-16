#!/usr/bin/python
import mraa
from SF_9DOF import IMU
from math import *
from scipy.integrate import trapz
import serial
import numpy as np
import time as tm
import socket

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
    # TODO: Try other modes as well
    imu.accel_mode(0b001)
    imu.gyro_mode(0b001)

    # Specify ranges for accelerometer, magnetometer and gyroscope
    # Accelerometer Options: "2G", "4G", "6G", "8G", "16G"
    # Magnetometer Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
    # Gyroscope Options: "245DPS", "500DPS", "2000DPS"

    imu.accel_range("2G")
    imu.mag_range("2GAUSS")
    imu.gyro_range("2000DPS")

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
    accelVec[0] = (imu.ax * 9.81)  + 0.8 #Constants are for 2G mode
    accelVec[1] = imu.ay * 9.81 + 0.75
    accelVec[2] = imu.az * 9.81 +0.44

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
    :param w2: z-component angular velocity
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


def computeInertialAcceleration(imu, orientMat):
    """ Computes the inertial frame (field frame) acceleration
    according to equation 12 in the paper

    Returns a 3x1 numpy column vector with (x,y,z) inertial acceleration components
    :param imu:
    :param orientMat:
    :return:
    """

    g = 9.81  # m/s^2 Remember to change if we switch to ft/s^2

    print orientMat

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
    inertialAcceleration[2] = orientMatTranspose[2][0]*ax + orientMatTranspose[2][1]*ay + orientMatTranspose[2][2]*az

    #Compensate for gravity
    inertialAcceleration[0] -= 0.25 # STRICTLY EXPERIMENTAL NO THEORY
    inertialAcceleration[1] += 0.45 # STRICTLY EXPERIMENTAL NO THEORY
    inertialAcceleration[2] -= (-9.81)


    print "inertial Acceleration:", inertialAcceleration
    print "direction matrix:", orientMat

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


def computeVelocityHistory(accelerationVector, timeVector):
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


def computeAngularVelocityMagnitude(angularVelocity):
    """Computes angular velocity vector magnitude

    :param angularVelocity:
    :return:
    """

    return sqrt(angularVelocity[0]**2 + angularVelocity[1]**2 + angularVelocity[2]**2)


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


def computeSweetSpotVelocity(imu, localVelocity, angularVelocity):
    """Computes sweet spot velocity on the bat
    Returns a 3x1 numpy column vector with (x,y,z) sweet spot velocity components
    :param imu:
    :return:
    """
    sweetSpotDistance = 0.7  # meters TODO:VERIFY SWEET SPOT DISTANCE
    sweetDistanceVector = np.array([1, 0, 0])
    sweetSpotVelocity = localVelocity + np.cross(angularVelocity,sweetDistanceVector)
    return sweetSpotVelocity


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

    #print sqrt(normalizedQuaternion[0]**2 + normalizedQuaternion[1]**2 +
    #         normalizedQuaternion[2]**2 + normalizedQuaternion[3]**2)

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
    angularVelocityMagnitude = computeAngularVelocityMagnitude(currentAngularVelocity)
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




    # Solve for euler parameters
    #eulerParameters = odeint(stateEquationModel, e_current, timeVector,
    #                         (xAngularVelocity, yAngularVelocity, zAngularVelocity))
    #eulerParameters = eulerParameters.tolist()[1]

    #print "Obtained Euler Parameters:"
    #print eulerParameters

    # TODO:Do we have to normalize the quaternion?
    # TODO:Can we use this same solver or do we have to switch

    #normalEulerParameters = normalizeEulerParameters(eulerParameters)

    return newEulerParameters


def sendData(data, interface=1):
    """Send data through selected interface. Interface is
    selected according to the chart below

    Interfaces:
    Serial -- 0
    Bluetooth -- 1

    :param data: Data to send of type list
    :param interface: integer denoting interface to select
    :return:
    """
    #Serial
    if interface is 0:
        uart = mraa.Uart(0)  # Enable UART for serial port usage
        ser = serial.Serial(
            port='/dev/ttyMFD2',  # Port dependant on hardware block
            baudrate=115200,
            parity=serial.PARITY_EVEN)

        # Send each number in the list
        for number in data:
            #print "Sending:", number
            bytesSent = ser.write(str(number) + '\n')
            #print "Bytes Sent:", bytesSent

        ser.write('\n')  # Send EOF Character
        print "Transmission successful"
    else:
        s = socket.socket()  # Create a socket object
        port = 80  # Reserve a port for your service.
        s.connect(('192.168.0.11', port))

        for number in data:
            s.send(str(number) + '\n')
            s.recv(1024)

        s.send('\n')
        s.recv(1024)
        print "Transmission Successful"



def valueStream():
    """Continuously displays linear acceleration and angular velocity values

    :return:
    """

    imu = initialize()
    # Obtain four initial euler parameters
    print "5 seconds to Calibrate. Please hold Calibration Position:"
    tm.sleep(5.5)  # Wait for calibration position
    e_initial = calibrate(imu)

    while(True):
        angularVelocity = readAngularVelocity(imu)
        acceleration = readAcceleration(imu)
        print("Acceleration:%s  Angular Velocity:%s", (acceleration, angularVelocity))
        tm.sleep(1)


def streamSwingTrial():
    """Runs a swing trial event and computes important bat metrics

    Returns the bat metrics in an array
    """

    imu = initialize()
    print "5 seconds to Calibrate. Please hold Calibration Position:"
    tm.sleep(5.5)  # Wait for calibration position
    e_initial = calibrate(imu) # Obtain four initial euler parameters
    #e_initial = normalizeEulerParameters(e_initial) #Normalize

    initialTime = tm.time() # Time

    # Initialize Storage Vectors
    acceleration = readAcceleration(imu)
    angularVelocity = readAngularVelocity(imu)
    xinertialAccelerationVector = [0]
    yinertialAccelerationVector = [0]
    zinertialAccelerationVector = [0]
    xAccelerationVector = [acceleration[0]]
    yAccelerationVector = [acceleration[1]]
    zAccelerationVector = [acceleration[2]]
    xAngularVelocity = [angularVelocity[0]]
    yAngularVelocity = [angularVelocity[1]]
    zAngularVelocity = [angularVelocity[2]]
    aimAngleVector = [0]
    rollVector = [0]

    rotationMatrices = [computeDirectionCosineMatrix(e_initial)]
    elevationAngles = [0]
    timeVector = [0]
    timeVectors = [0]
    sampleTimes = [0]

    # Initialize useful computation variables
    previousEpochTime = initialTime  # t0
    previousElapsedSampleTime = 0
    currentElapsedSampleTime = 0
    previousEulerParameters = e_initial
    index = 0

    # Loop for 10 seconds
    while (tm.time() - initialTime) < 10:

        # Read Angular Velocity and Acceleration
        currentAngularVelocity = readAngularVelocity(imu)
        currentAcceleration = readAcceleration(imu)
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
        currentEulerParameters = computeEulerParameters(previousEulerParameters, timeVector, currentAngularVelocity)
        eulerParametersNormalized = currentEulerParameters
        #eulerPrametersNoramlized = normalizeEulerParameters(currentEulerParameters)

        # Compute Direction Cosine Matrix
        directionMatrix = computeDirectionCosineMatrix(eulerParametersNormalized)
        rotationMatrices.append(directionMatrix)

        #print "Direction Cosine Matrix:", directionMatrix[0]


        # Get Inertial Acceleration snd Velocity
        xinertialAcceleration, yinertialAcceleration, zinertialAcceleration = computeInertialAcceleration(imu, directionMatrix)
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
        roll = currentEulerParameters[3]**2 - currentEulerParameters[1]**2 \
               - currentEulerParameters[2]**2 - currentEulerParameters[3]**2

        roll = acos(roll)

        elevationAngles.append(elevationAngle)
        aimAngleVector.append(aimAngle)
        rollVector.append(roll)


    # Once trial is finished, compute inertial velocity

    #xinertialVelocity, yinertialVelocity, zinertialVelocity = computeInertialVelocity(imu, xinertialAccelerationVector, yinertialAccelerationVector,
    #                                                                                  zinertialAccelerationVector, timeVectors)

    # Compute Velocity
    xinertialVelocity = computeVelocityHistory(xinertialAccelerationVector, sampleTimes)
    yinertialVelocity = computeVelocityHistory(yinertialAccelerationVector, sampleTimes)
    zinertialVelocity = computeVelocityHistory(zinertialAccelerationVector, sampleTimes)


    # Data must be received in the same order sent
    sendData(xAccelerationVector)
    sendData(yAccelerationVector)
    sendData(zAccelerationVector)
    sendData(xAngularVelocity)
    sendData(yAngularVelocity)
    sendData(zAngularVelocity)
    sendData(elevationAngles)
    sendData(timeVectors)
    sendData(xinertialVelocity)
    sendData(yinertialVelocity)
    sendData(zinertialVelocity)
    sendData(xinertialAccelerationVector)
    sendData(yinertialAccelerationVector)
    sendData(zinertialAccelerationVector)
    sendData(aimAngleVector)
    sendData(rollVector)




#valueStream()
streamSwingTrial()
#s.close()