import serial
from sys import platform as _platform
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, freqz
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
    elevationAngles = readData()
    timeVector = readData() # TODO: Change name to avoid confusion
    xInertialVelocity = readData()
    yInertialVelocity = readData()
    zInertialVelocity = readData()
    xInertialAcceleration = readData()
    yInertialAcceleration = readData()
    zInertialAcceleration = readData()

    print elevationAngles


    plotLinearAcceleration(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                           xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                           xInertialVelocity, yInertialVelocity, zInertialVelocity,
                           xInertialAcceleration, yInertialAcceleration, zInertialAcceleration)


def plotLinearAcceleration(xAccelerationVector, yAccelerationVector, zAccelerationVector, timeVector,
                           xAngularVelocity, yAngularVelocity, zAngularVelocity, elevationAngles,
                           xInertialVelocity, yInertialVelocity, zInertialVelocity,
                           xInertialAcceleration, yInertialAcceleration, zInertialAcceleration):
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
    plt.ylabel('Linear Acceleration [G[s]]')
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
    plt.plot(timeVector, elevationAngles, 'b')
    plt.ylabel('Elevation Angle [degrees]')
    plt.xlabel('Time [seconds]')
    plt.legend(['Elevation Angle'], loc='lower left')

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
    plt.grid()
    plt.show()


def readData():
    """Reads data into a list until EOF character is detected
    :param
    :return: Data received [Numpy Array]
    """

    dataList = []
    while True:

        if ser.in_waiting >= 1:
            out = ser.readline()

            if out == "\n":
                print "EOL Character Received:"
                #print out
                break

            else:
                dataList.append(float(out))
                #print out

    return np.asarray(dataList)

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y




obtainSwingData()