import serial
import mraa
import time

ser = serial.Serial(port='/dev/ttyMFD2',
        baudrate=115200,
        parity=serial.PARITY_EVEN)

uart = mraa.Uart(0)
print(uart.getDevicePath)
intList = [-1.1, 2.01312, 3.01312, 50000000000000000]

for number in intList: 
       print "Sending:", number
       bytesSent = ser.write(str(number)+'\n')
       print "Bytes Sent:", bytesSent

print "Transmission Compelete"
       

print("Serial write successful.")

