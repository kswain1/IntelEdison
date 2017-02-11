from SF_9DOF import IMU
from fusion import Fusion
from swing_detector import SwingDetector
import time
import sys
import select
import tty
import termios

#Initialization code
# Create IMU object
imu = IMU() # To select a specific I2C port, use IMU(n). Default is 1.

# Initialize IMU
imu.initialize()

# Enable accel, mag, gyro, and temperature
imu.enable_accel()
imu.enable_mag()
imu.enable_gyro()
imu.enable_temp()
fuse = Fusion()

callibrate_count = 0

#swing detection
swing = SwingDetector()

# Set range on accel, mag, and gyro

# Specify Options: "2G", "4G", "6G", "8G", "16G"
imu.accel_range("2G")       # leave blank for default of "2G"

# Specify Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
imu.mag_range("2GAUSS")     # leave blank for default of "2GAUSS"

# Specify Options: "245DPS", "500DPS", "2000DPS"
imu.gyro_range("245DPS")    # leave blank for default of "245DPS"


# Loop and read accel, mag, and gyro
log_file = raw_input("Enter name of log file: ")
#outFile = open("logs/"+log_file, 'w')
outFile_accel = open("guzman_logs/accel_"+log_file, 'w')

#File header 
outFile_accel.write("ax, ay, az, gx, gy, gz \n")

callibrated = False
_heading = 0
_pitch = 0
_roll = 0
swinging = False

def is_swinging():
    global swinging
    print(swinging)
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        c = sys.stdin.read(1)
        if c == '\x1b':
            exit(0)
        if c != 1:
            return 0
        swinging = not swinging
    return swinging

def calibrate(mag):
    print("Calibrating. Press switch when done.")
    sw = switch
    fuse.calibrate(mag, sw, lambda: time.sleep(0.1))
    print(fuse.magbias)

def switch(count):
    if count < 20:
        time.sleep(30)
        count += 1
        return True
    else:
        return False

# this is for non-blocking input
old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())
    imu.read_mag()
    mag = (imu.mx, imu.my, imu.mz)
    calibrate(mag)
    print fuse.magbias
    
    while(1):

        imu.read_accel()
        imu.read_mag()
        imu.read_gyro()
        imu.readTemp()



         #gather the accel results for the fusion algorithm
        accel = (float(imu.ax), float(imu.ay), float(imu.az))
        gyro = (float(imu.gx), float(imu.gy), float(imu.gz))
        mag = (float(imu.mx), float(imu.my), float(imu.mz))



        # Print the results
        print "Accel: " + str(imu.ax) + ", " + str(imu.ay) + ", " + str(imu.az)
        print "Mag: " + str(imu.mx) + ", " + str(imu.my) + ", " + str(imu.mz)
        print "Gyro: " + str(imu.gx) + ", " + str(imu.gy) + ", " + str(imu.gz)
        print "Temperature: " + str(imu.temp)
        outFile_accel.write("{:7.3f},{:7.3f},{:7.3f},{:d}\n".format(imu.ax, imu.ay, imu.az,is_swinging()))
        data = {"AX":str(imu.ax),"AY":str(imu.ay),"AZ":str(imu.az)}

           #outFile.write("{:7.3f},{:7.3f},{:7.3f},{:d}\n".format(fuse.heading, fuse.pitch, fuse.roll, is_swinging()))


           # swing.swing_detector(heading, pitch, roll)
        print("count is : ", callibrate_count, "\n")
        # Sleep for 1/10th of a second
        time.sleep(0.1)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
