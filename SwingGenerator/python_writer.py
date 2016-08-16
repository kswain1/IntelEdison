import serial 

ser = serial.Serial(
	port='/dev/ttyMFD1',
	baudrate=115200
)

ser.write('information is being passed')


