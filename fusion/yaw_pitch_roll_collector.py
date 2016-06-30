from SF_9DOF import IMU
from fusion import Fusion
from swing_detector import SwingDetector
import time
import sys
import select
import tty
import termios
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
outFile = open("logs/"+log_file, 'w')
outFile_accel = open("logs/accel_"+log_file, 'w') 


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
        swinging = not swinging
    return swinging

def calibrate(mag):
    fuse = Fusion()
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
    #fuse.calibrate((imu.mx, imu.my, imu.mz),switch(0), lambda: time.sleep(0.1 ))
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

        #passes the data to the fusion data for yaw pitch and roll data
        fuse.update(accel,gyro, mag)
       # print("I am printing the fuse data", fuse.roll)
        print("\n")
        callibrate_count += 1

        #initial callibration
        if (not callibrated or callibrate_count < 50) and (abs(_heading - fuse.heading) > 1.0 or abs(_pitch - fuse.pitch) > 1.0 or abs(_roll - fuse.roll) > 1.0):
           #_heading = (_heading * callibrate_count + fuse.heading) / (callibrate_count + 1)
           #_pitch = (_pitch * callibrate_count + fuse.pitch) / (callibrate_count + 1)
           #_roll = (_roll * callibrate_count + fuse.roll) / (callibrate_count + 1)
           _heading = fuse.heading
           _pitch = fuse.pitch
           _roll = fuse.roll
        else:
           callibrated = True
           heading = fuse.heading - _heading
           pitch = fuse.pitch - _pitch
           roll = fuse.roll - _roll
           print("Motion Tracking Values!!: Pitch: {:7.3f} Heading: {:7.3f} Roll: {:7.3f}".format(pitch, heading, roll))
           outFile.write("{:7.3f},{:7.3f},{:7.3f},{:d}\n".format(heading, pitch, roll, is_swinging()))


           swing.swing_detector(heading, pitch, roll)
        print("count is : ", callibrate_count, "\n")
        # Sleep for 1/10th of a second
        time.sleep(0.1)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
