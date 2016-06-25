from SF_9DOF import IMU
from fusion import Fusion
import time
# Create IMU object
imu = IMU() # To select a specific I2C port, use IMU(n). Default is 1.

# Initialize IMU
imu.initialize()

# Enable accel, mag, gyro, and temperature
imu.enable_accel()
imu.enable_mag()
imu.enable_gyro()

#create yaw_pitch_roll object
fuse = Fusion()

# Specify Options: "2G", "4G", "6G", "8G", "16G"
imu.accel_range("2G")       # leave blank for default of "2G"

# Specify Options: "2GAUSS", "4GAUSS", "8GAUSS", "12GAUSS"
imu.mag_range("2GAUSS")     # leave blank for default of "2GAUSS"

#specify options for gyro range
imu.gyro_range("2000DPS")    # leave blank for default of "245DPS"


#callibrating values to zero
callibrated = False
_heading = 0
_pitch = 0
_roll = 0


while(1):

             imu.read_accel()
             imu.read_mag()
             imu.read_gyro()
             imu.readTemp()
             

              #gather the accel results for the fusion algorithm
             accel = (float(imu.ax), float(imu.ay), float(imu.az))
             gyro = (float(imu.gx), float(imu.gy), float(imu.gz))
             mag = (float(imu.mx), float(imu.my), float(imu.mz))

            #passes the accel, gyro, mag data to the fusion data for yaw pitch and roll data
             fuse.update(accel,gyro, mag)

            #2 set the reference point
            
            #initial callibration
             if (not callibrated) and (abs(_heading - fuse.heading) > 1.0 and abs(_pitch - fuse.pitch) > 1.0 and abs((_roll - fuse.roll) > 1.0)):
                 _heading = fuse.heading
                 _pitch = fuse.pitch
                 _roll = fuse.roll
             else:
                 callibrated = True
                 heading = fuse.heading - _heading
                 pitch = fuse.pitch - _pitch
                 roll = fuse.roll - _roll
                 print("Motion Tracking Values!!: Pitch: {:7.3f} Heading: {:7.3f} Roll: {:7.3f}".format(pitch, heading, roll))

          

            #4 take the angle measurements at zero

            #5 Take cos of Euler Angle




             time.sleep(0.1)
