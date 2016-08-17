import serial
from sys import platform as _platform
import matplotlib.pyplot as plt
import numpy as np
import time

# Open Serial Port
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


def obtainSwingData():
    """Obtains all the data from the edison
    The order in which the data is read
    must be the same order in which it is
    sent. Data is then plotted

    :return:
    """

    xAccelerationVector = readData()
    yAccelerationVector = readData()
    zAccelerationVector = readData()
    xAngularVelocity = readData()
    yAngularVelocity = readData()
    zAngularVelocity = readData()
    timeVector = readData() # TODO: Change name to avoid confusion


    plotLinearAcceleration(xAccelerationVector, timeVector)


def plotLinearAcceleration(accelerationVector, timeVector):
    """ Plots Acceleration vs Time

    :param accelerationVector:
    :param timeVector:
    :return:
    """

    plt.plot(timeVector, accelerationVector)
    plt.show()


def readData():
    """Reads data into a list until EOF character is detected
    :param
    :return: Data received [Numpy Array]
    """

    dataList = np.array()
    while True:

        if ser.in_waiting >= 1:
            out = ser.readline()

            if out == "\n":
                print "EOL Character Received:"
                print out
                break

            else:
                dataList.append(float(out))
                print out

    return dataList



obtainSwingData()