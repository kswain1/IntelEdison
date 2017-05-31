#!/usr/bin/python
import mraa
from SF_9DOF import IMU


class GetSwing():

      def imu(self):

        imu = IMU()  # To select a specific I2C port, use IMU(n). Default is 1.

        imu.initialize()

        # Enable accelerometer, magnetometer, gyroscope
        imu.enable_accel()
        imu.enable_mag()
        imu.enable_gyro()
        print imu.read_accel()
        print imu.ax, imu.ay, imu.az
        return imu

      def imu_data(self):
          data = self.imu()
          print data.ax, data.ay, data.az

s = GetSwing()

s.imu()
s.imu_data()
      
