import serial
from sys import platform as _platform

import matplotlib
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

myList = []


while True:

    if ser.in_waiting >= 1:
        out = ser.readline()

        if out == "\n":
            print "EOL Character Received:"
            print out
            break

        else:
            myList.append(float(out))
            print out



print "Elements in myList:", myList
print "Transmission Successful"
