"""
File: D2WMotion.py
Author: Victor Barros Coch
Date: 20/03/2024
Description: This script contais a class and methods to communicate with the
D2W platform prototype, with i2c.
"""
import time
import smbus
from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu
from .quaternion_tools import quaternion_from_euler

class D2WMotion():
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = [0x0a,0x0b,0x0c,0x0d]
        self.imu_address = 0x68
        self.imu = MPU9250.MPU9250(self.bus,self.imu_address)
        self.imu.begin()

    def writeData(self,value,address,command=0x69):
        byteValue = list(b'000000000000000') #15 bits, expected by controller
        for idc, c in enumerate(value):
            byteValue[idc] = ord(c)
        try:
            self.bus.write_i2c_block_data(address,command,byteValue)
        except:
            print('Could not write to i2c buffer')
            return -1

    def setVel(self,vel,joint):
        dir = 0
        if (vel > 0): dir = 1
        if (abs(vel) < 200):
            vel = 0
            dir = 0
        elif (abs(vel) > 4095): vel = 4095
        else: vel = abs(vel)

        payload = '%04d,%1d'%(int(vel),dir)
        self.writeData(payload,self.address[joint])
        return -1

    def readEncoder(self,joint): 
        try:
            pos = float(''.join(chr(i) for i in self.bus.read_i2c_block_data(self.address[joint],0xFE,16)).strip('\xFF'))
            vel = float(''.join(chr(i) for i in self.bus.read_i2c_block_data(self.address[joint],0xFF,16)).strip('\xFF'))
        except:
            print('Could not get odometry')
            return (-1,-1)
        return (pos,vel)
    
    def readImu(self):
        try:
            self.imu.readSensor()
            self.imu.computeOrientation()
        except:
            print('Could not get IMU feedback')
            return -1
        
        imu_msg = Imu()
        #imu_msg.linear_acceleration_covariance = [0]
        imu_msg.linear_acceleration.x = self.imu.AccelVals[0]
        imu_msg.linear_acceleration.y = self.imu.AccelVals[1]
        imu_msg.linear_acceleration.z = self.imu.AccelVals[2]
        #imu_msg.angular_velocity_covariance = [0]
        imu_msg.angular_velocity.x = self.imu.GyroVals[0]
        imu_msg.angular_velocity.y = self.imu.GyroVals[1]
        imu_msg.angular_velocity.z = self.imu.GyroVals[2]
        quaternion = quaternion_from_euler(self.imu.roll, self.imu.pitch, self.imu.yaw)
        #imu_msg.orientation_covariance = [0]
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        return imu_msg

    def calImu(self):
        try:
            self.imu.caliberateGyro()
            self.imu.caliberateAccelerometer()
        except:
            print('Could not execute cal')

def main():
    comms = D2WMotion()
    while True:
        print("Send Msg")
        joint = int(input("Joint >"))
        if joint == -1: return -1
        vel = int(input("vel >"))
        comms.setVel(vel,joint)
        time.sleep(1)

if (__name__ == '__main__'):
    main()
