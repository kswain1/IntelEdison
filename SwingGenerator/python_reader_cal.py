import serial
from sys import platform as _platform
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, freqz
import socket
from euler_parametrization import EulerParametrization
import time

# Open Serial Port
swing_file_name = raw_input("Please input the name of your swing file: ")

# Init Server
#host = socket.gethostname()
port = 81
s = socket.socket()  # Create a socket object
s.bind(('', port))  # Bind to the port
s.listen(5)
#socket.setdefaulttimeout(20)
c, addr = s.accept()
print "Connection Accepted:"




def obtainSwingData():
    """Obtains all the data from the edison
    The order in which the data is read
    must be the same order in which it is
    sent. Data is then plotted

    :return:
    """

    #Obtain the long transmission string and parse
    recieveString = readData()

    xAccelerationVector = recieveString[0].split()
    yAccelerationVector = recieveString[1].split()
    zAccelerationVector = recieveString[2].split()
    xAngularVelocity = recieveString[3].split()
    yAngularVelocity = recieveString[4].split()
    zAngularVelocity = recieveString[5].split()
    elevationAngles = recieveString[6].split()
    timeVector = recieveString[7].split()
    xInertialAcceleration = recieveString[8].split()
    yInertialAcceleration = recieveString[9].split()
    zInertialAcceleration = recieveString[10].split()
    xInertialVelocity = recieveString[11].split()
    yInertialVelocity = recieveString[12].split()
    zInertialVelocity = recieveString[13].split()
    aimAngles = recieveString[14].split()
    rolls = recieveString[15].split()
    sweetSpotVelocity = recieveString[16].split()
    velocityMagnitude = recieveString[17].split()
    xpositionVector = recieveString[18].split()
    ypositionVector = recieveString[19].split()
    zpositionVector = recieveString[20].split()
    calibration_angles = recieveString[21].split()


    print "Recieve String:"
    print recieveString
    print "Recieve String Length:"
    print len(recieveString)
    c.close()
    s.close()

    klistEntryTypes(xAccelerationVector,yAccelerationVector, zAccelerationVector, xAngularVelocity, yAngularVelocity,
                    zAngularVelocity,elevationAngles, timeVector, xInertialAcceleration,yInertialAcceleration,
                    zInertialAcceleration,xInertialVelocity, yInertialVelocity, zInertialVelocity, aimAngles,
                    rolls, sweetSpotVelocity, velocityMagnitude, xpositionVector, ypositionVector, zpositionVector,
                    calibration_angles)


    plotEverything(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                   xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                   xInertialVelocity, yInertialVelocity, zInertialVelocity,
                   xInertialAcceleration, yInertialAcceleration, zInertialAcceleration,
                   aimAngles, rolls, sweetSpotVelocity, velocityMagnitude, xpositionVector,
                   ypositionVector, zpositionVector, calibration_angles)

    csv_writer(rolls, aimAngles, elevationAngles)

    csv_writer_cal(rolls, aimAngles, elevationAngles, calibration_angles)

    e = EulerParametrization(rotation_data_file="guzman_logs/"+swing_file_name+".csv")
    _ = e.animation()
    e.show()


def plotEverything(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                   xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                   xInertialVelocity, yInertialVelocity, zInertialVelocity,
                   xInertialAcceleration, yInertialAcceleration, zInertialAcceleration, aimAngles, rolls, sweetSpotVelocity,
                   velocityMagnitude, xpositionVector, ypositionVector, zPositionVector, calibration_angles):
    """ Plots Acceleration vs Time

    :param accelerationVector:
    :param timeVector:
    :return:
    """

    # Filter requirements.
    order = 6
    fs = 400  # sample rate, Hz
    cutoff = 110  # desired cutoff frequency of the filter, Hz

    # Get the filter coefficients so we can check its frequency response.
    #b, a = butter_lowpass(cutoff, fs, order)

    # Filter the data
    # FilteredData = butter_lowpass_filter(xAngularVelocity, cutoff, fs, order)
    # yFilteredData = butter_lowpass_filter(yAngularVelocity, cutoff, fs, order)
    # zFilteredData = butter_lowpass_filter(zAngularVelocity, cutoff, fs, order)

    #xAccelerationVector = xFilteredData
    #yAngularVelocity = yFilteredData
    #zAngularVelocity = zFilteredData

    print "Time length", len(timeVector)
    print "Acceleration Length", len(xAccelerationVector)

    #timeVector = timeVector[0:len(xAccelerationVector)]

    plt.subplot(3, 2, 1)

    plt.plot(timeVector, xAccelerationVector, 'b',
             timeVector, yAccelerationVector, 'g',
             timeVector, zAccelerationVector, 'r')
    plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    plt.ylim(-5, 5)
    plt.legend(['x - Linear Acceleration',
                'y - Linear Acceleration',
                'z - Linear Acceleration'],
               loc='lower left')
    plt.ylabel('Linear Acceleration [m/s^2]')
    plt.xlabel('Time [seconds]')

    plt.subplot(3, 2, 2)

    plt.plot(timeVector, xAngularVelocity, 'b',
             timeVector, yAngularVelocity, 'g',
             timeVector, zAngularVelocity, 'r')
    plt.ylim(-6, 6)

    plt.ylabel('Angular Velocity [Degrees/second]')
    plt.xlabel('Time [seconds]')
    #plt.axis([0, 10, -5, 5])
    plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    plt.legend(['x - Angular Velocity',
                'y - Angular Velocity',
                'z - Angular Velocity'],
               loc='lower left')

    plt.subplot(3, 2, 3)
    plt.xlim(0, timeVector[-1]) # Last value in time vector as upper limit
    plt.plot(timeVector, rolls, 'b',
             timeVector, aimAngles, 'g',
             timeVector, elevationAngles, 'r',
             )
    plt.ylabel('Elevation Angle [degrees]')
    plt.xlabel('Time [seconds]')
    plt.legend(['Roll',
                'Pitch',
                'Yaw'], loc='lower left')

    plt.subplot(3, 2, 4)
    plt.plot(timeVector, calibration_angles, 'b',
             timeVector, elevationAngles, 'g')

    plt.ylabel('Angle [degrees]')
    plt.xlabel('Time [seconds]')
    plt.legend(['Expected Yaw',
                'Yaw'], loc='lower left')

    plt.subplot(3, 2, 5)
    plt.plot(timeVector, xInertialVelocity, 'b',
             timeVector, yInertialVelocity, 'g',
             timeVector, zInertialVelocity, 'r')

    plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    plt.ylabel('Inertial Velocity [m/s]')
    plt.xlabel('Time [seconds]')
    plt.legend(['x - Inertial Velocity',
                'y - Inertial Velocity',
                'z - Inertial Velocity'],
               loc='lower left')

    plt.subplot(3, 2, 6)
    plt.plot(timeVector, xInertialAcceleration, 'b',
             timeVector, yInertialAcceleration, 'g',
             timeVector, zInertialAcceleration, 'r')
    plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    plt.ylabel('Inertial Acceleration [m/s^2]')
    plt.xlabel('Time [seconds]')
    plt.legend(['x - Inertial Acceleration',
                'y - Inertial Acceleration',
                'z - Inertial Acceleration'],
               loc='lower left')

    # plt.subplot(3, 2, 6)
    # plt.plot(timeVector, sweetSpotVelocity, 'b',
    #          timeVector, velocityMagnitude, 'r')
    # plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    # plt.ylabel('Sweet Spot Velocity [m/s]')
    # plt.xlabel('Time [seconds]')
    # plt.legend(['Sweet Spot Velocity',
    #             'Velocity Magnitude'],
    #            loc='lower left')

    # plt.subplot(3, 2, 6)
    # plt.plot(timeVector, xpositionVector, 'b',
    #          timeVector, ypositionVector, 'g',
    #          timeVector, zPositionVector, 'r')
    # plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    # plt.ylabel('Position [m]')
    # plt.xlabel('Time [seconds]')
    # plt.legend(['X-Position',
    #             'Y-Position,'
    #             'Z-Position'],
    #            loc='lower left')



    plt.grid()
    plt.show()

def klistEntryTypes(*args):

    for data_list in args:
        counter = 0
        for entry in data_list:
            # print "Entry" + str(counter) + " type:"
            # print "Entry value:"
            # print entry
            # print type(entry)
            data_list[counter] = float(entry)

        # print type(dataList[counter])
            counter += 1

    return data_list

def listEntryTypes(dataList):
    """Reads the type of each entry in the list"""
    counter = 0

    for entry in dataList:
        print "Entry" + str(counter) + " type:"
        print "Entry value:"
        print entry
        print type(entry)

        dataList[counter] = float(entry)
        print type(dataList[counter])
        counter = counter + 1

    return dataList
    #return dataList.astype(np.float)

def readData():
    """Reads data into a list until EOF character is detected
    :param
    :return: Data received [Numpy Array]
    """

    dataList = []
    longString = ''

    #print "Connection accepted"
    while True:
        data = c.recv(100000)
        print "Data recieved:"
        print data

        if data == '':
            break

        longString = longString + data

        #print "I recieved:"
        #print data

        dataList = longString.split('!')

    return dataList
    #return np.asarray(dataList)


def stringToList(dataList):
    """Converts a string with entries seperated with newlines into a list"""



def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def csv_writer(rolls, pitchs, yaws):
    outFile_accel = open("guzman_logs/"+swing_file_name+".csv", 'w')
    # File header
    outFile_accel.write("roll, pitch, yaw\n")
    # for roll,elevationAngle, aimAngle in rolls, yaw, pitch:
    for i in range(0,len(rolls)):
        outFile_accel.write("{:7.3f},{:7.3f},{:7.3f}\n".format(float(rolls[i]),float(pitchs[i]), float(yaws[i])))



def csv_writer_cal(rolls, pitchs, yaws, calibration_angles):
    outFile_accel = open("guzman_logs/cal_"+swing_file_name+".csv", 'w')
    # File header
    outFile_accel.write("roll, expected_roll, pitch, yaw\n")
    # for roll,elevationAngle, aimAngle in rolls, yaw, pitch:
    for i in range(0,len(rolls)):
        outFile_accel.write("{:7.3f},{:f},{:7.3f},{:7.3f}\n".format(float(rolls[i]), calibration_angles[i],float(pitchs[i]), float(yaws[i])))




obtainSwingData()
s.close()
