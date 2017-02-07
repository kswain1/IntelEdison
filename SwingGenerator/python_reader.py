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
interface = input("Recieve Data serially or through wifi?")

if interface == 0:
    if _platform == "linux" or _platform == "linux2":
        #Linux
        print "Linux Detected"

    elif _platform == "darwin":
        #MAC OS X
        print "OSX Detected"
        ser = serial.Serial(port='/dev/tty.usbserial-DNO1EW18', baudrate=115200, parity=serial.PARITY_EVEN, timeout=20)

    elif _platform == "win32":
       #Windows
       print "Windows Detected"
       ser = serial.Serial(port='COM6', baudrate=115200, parity=serial.PARITY_EVEN, timeout=20)

       print "Port Open:", ser.is_open
       print "Port Name:", ser.name
       print "Flushing Serial Input Buffer.."
       print "Waiting for Data.."
       ser.flushInput()

else:
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

    """
    print "xAccel Vector:"
    xAccelerationVector = readData(interface)
    print "yAccel Vector:"
    yAccelerationVector = readData(interface)
    print "zAccel Vector:"
    zAccelerationVector = readData(interface)
    print "xAngular Velocity Vector:"
    xAngularVelocity = readData(interface)
    print "yAngular Velocity Vector:"
    yAngularVelocity = readData(interface)
    print "zAngular Velocity Vector:"
    zAngularVelocity = readData(interface)
    print "elevation Angles"
    elevationAngles = readData(interface)
    print "time vectors"
    timeVector = readData(interface) # TODO: Change name to avoid confusion
    print "xinertial velocity"
    xInertialVelocity = readData(interface)
    print "yinertial Velocity"
    yInertialVelocity = readData(interface)
    print "zinertial Velocity"
    zInertialVelocity = readData(interface)
    print "x inertial Acceleration Vector"
    xInertialAcceleration = readData(interface)
    print "y inertial Acceleration Vector"
    yInertialAcceleration = readData(interface)
    print "z inertial Acceleration Vector"
    zInertialAcceleration = readData(interface)
    print "aim angle vector"
    aimAngles = readData(interface)
    print "roll vectors"
    rolls = readData(interface)
    print "sweet spot velocity vector"
    sweetSpotVelocity = readData(interface)
    print "velocity magnitude vector"
    velocityMagnitude = readData(interface)
    """

    #Obtain the long transmission string and parse
    recieveString = readData(interface)

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


    print "Recieve String:"
    print recieveString
    print "Recieve String Length:"
    print len(recieveString)
    c.close()
    s.close()

    print "Sweet Spot Velocity Vector"
    print type(sweetSpotVelocity)
    print sweetSpotVelocity

    print "Sweet Velocity Magnitude Vector"
    print type(velocityMagnitude)
    print velocityMagnitude

    print "Sweetspotvelocity length"
    print len(sweetSpotVelocity)
    print "VelocityMagnitude length"
    print len(velocityMagnitude)

    #csv_writer(rolls, elevationAngles, aimAngles)
    print type(xAngularVelocity)

    print "xAccelerationVector:"
    listEntryTypes(xAccelerationVector)
    print "yAccelerationVector"
    listEntryTypes(yAccelerationVector)
    print "zAccelerationVector"
    listEntryTypes(zAccelerationVector)
    print "xAngularVelocity"
    listEntryTypes(xAngularVelocity)
    print "yAngularVelocity"
    listEntryTypes(yAngularVelocity)
    print "zAngularVelocity"
    listEntryTypes(zAngularVelocity)
    print "elevationAngles"
    listEntryTypes(elevationAngles)
    print "timeVector:"
    listEntryTypes(timeVector)
    print "xInertialAcceleration"
    listEntryTypes(xInertialAcceleration)
    print "yInertialAcceleration"
    listEntryTypes(yInertialAcceleration)
    print "zInertialAcceleration"
    listEntryTypes(zInertialAcceleration)
    print "xInertialVelocity"
    listEntryTypes(xInertialVelocity)
    print "yInertialVelocity"
    listEntryTypes(yInertialVelocity)
    print "zInertialVelocity"
    listEntryTypes(zInertialVelocity)
    print "aimAngles"
    listEntryTypes(aimAngles)
    print "rolls"
    listEntryTypes(rolls)
    print "sweetSpotVelocity"
    listEntryTypes(sweetSpotVelocity)
    print "velocityMagnitude"
    listEntryTypes(velocityMagnitude)


    csv_writer_id(rolls, aimAngles, elevationAngles)
    

    plotEverything(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                   xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                   xInertialVelocity, yInertialVelocity, zInertialVelocity,
                   xInertialAcceleration, yInertialAcceleration, zInertialAcceleration,
                   aimAngles, rolls, sweetSpotVelocity, velocityMagnitude)

    csv_writer(rolls, aimAngles, elevationAngles)
   
    e = EulerParametrization(rotation_data_file='guzman_logs/'+swing_file_name)
    _ = e.animation()
    e.show()
    

def plotEverything(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                   xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                   xInertialVelocity, yInertialVelocity, zInertialVelocity,
                   xInertialAcceleration, yInertialAcceleration, zInertialAcceleration, aimAngles, rolls, sweetSpotVelocity,
                   velocityMagnitude):
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
    xFilteredData = butter_lowpass_filter(xAngularVelocity, cutoff, fs, order)
    yFilteredData = butter_lowpass_filter(yAngularVelocity, cutoff, fs, order)
    zFilteredData = butter_lowpass_filter(zAngularVelocity, cutoff, fs, order)

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
    plt.plot(timeVector, elevationAngles, 'b',
             timeVector, aimAngles, 'g',
             timeVector, rolls, 'r')
    plt.ylabel('Elevation Angle [degrees]')
    plt.xlabel('Time [seconds]')
    plt.legend(['Elevation Angle',
                'Aim',
                'Roll'], loc='lower left')

    plt.subplot(3, 2, 4)
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

    plt.subplot(3, 2, 5)
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

    plt.subplot(3, 2, 6)
    plt.plot(timeVector, sweetSpotVelocity, 'b',
             timeVector, velocityMagnitude, 'r')
    plt.xlim(0, timeVector[-1])  # Last value in time vector as upper limit
    plt.ylabel('Sweet Spot Velocity [m/s]')
    plt.xlabel('Time [seconds]')
    plt.legend(['Sweet Spot Velocity',
                'Velocity Magnitude'],
               loc='lower left')



    plt.grid()
    plt.show()

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

def readData(interface=1):
    """Reads data into a list until EOF character is detected
    :param
    :return: Data received [Numpy Array]
    """

    #Serial
    if interface is 0:
        dataList = []
        while True:

            if ser.in_waiting >= 1:
                out = ser.readline()

                if out == '\n':
                    print "EOL Character Received:"
                    #print out
                    break

                else:
                    dataList.append(float(out))
                    #print out
    else:


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

            """while True:
                  # Establish connection with client.
                #print 'Got connection from', addr
                data = c.recvfrom(4096)
                print "Data Recieved:"
                print data
                if data == "\n":
                    print "EOL Character Received:"
                    break
                else:
                    #print dataList
                    #dataList.append(float(data))
                    dataList.append(data)
                    #print dataList
                #c.close()  # Close the connection
            print dataList"""
            #print "The dataList is:"
            #print dataList

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



def csv_writer_id(rolls, pitchs, yaws):
    outFile_accel = open("guzman_logs/id_"+swing_file_name+".csv", 'w')
    # File header
    outFile_accel.write("id, roll, pitch, yaw\n")
    # for roll,elevationAngle, aimAngle in rolls, yaw, pitch:
    for i in range(0,len(rolls)):
        outFile_accel.write("{:d},{:7.3f},{:7.3f},{:7.3f}\n".format(i,float(rolls[i]),float(pitchs[i]), float(yaws[i])))




obtainSwingData()
s.close()
