import serial
import time

ser = serial.Serial(
	port='COM6',
	baudrate=115200,
	parity=serial.PARITY_EVEN,
)

print "Port Open:", ser.is_open
print "Port Name:", ser.name
print "Flushing Serial Input Buffer.."
ser.flushInput()

print "Information Received:"


myList = []

while True:

	if ser.in_waiting >= 1:
		out = ser.readline()
		myList.append(float(out))
		print out
		time.sleep(2)

	else:
		print "Waiting for data.."
		print "Elements in myList:", myList
		time.sleep(2)

