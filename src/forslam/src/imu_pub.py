#!/usr/bin/env python
'''imu ROS Node'''
# license removed for brevity
import rospy
import string
import serial
import math
import sys
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
degree2rad = math.pi/180.0

rospy.init_node("imu_pub_node")
pub = rospy.Publisher("imu",Imu,queue_size=1)
imuMsg = Imu()

imuMsg.orientation_covariance = [
    0.0025, 0, 0,
    0, 0.0025, 0,
    0, 0, 0.0025
]

imuMsg.angular_velocity_covariance = [
    0.02, 0, 0,
    0, 0.02, 0,
    0, 0, 0.02
]

imuMsg.linear_acceleration_covariance = [
    0.04, 0, 0,
    0, 0.04, 0,
    0, 0, 0.04
]

port = '/dev/ttyUSB0'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found ar port" + port + ".")
    sys.exit(0)

roll = 0
pitch = 0
yaw = 0
seq = 0

rate = rospy.Rate(10)
# ser.flushInput()
def minus16int(src):
    if src > 32768.0-1:
        src = src - 65536.0 +1
    return src
while not rospy.is_shutdown():

    ser.flushInput()
    while True:
        while not ord(ser.read()) == 0x55:
            pass
        if ord(ser.read()) == 0x51:
            break
    
    data = ser.read(31)

    checkFlag = False
    checkFlag=((0x55+0x51+ord(data[0])+ord(data[1])+ord(data[2])+ord(data[3])+ord(data[4])+ord(data[5])+ord(data[6])+ord(data[7]))%256==ord(data[8]))
    if not checkFlag:
        continue
    
    imuMsg.linear_acceleration.x = minus16int(ord(data[1])*256.0+ord(data[0]))/32768.0*16*9.8
    imuMsg.linear_acceleration.y = minus16int(ord(data[3])*256.0+ord(data[2]))/32768.0*16*9.8
    imuMsg.linear_acceleration.z = minus16int(ord(data[5])*256.0+ord(data[4]))/32768.0*16*9.8
    
    imuMsg.angular_velocity.x = minus16int(ord(data[12])*256.0+ord(data[11]))/32768.0*2000.0*3.14/180.0
    imuMsg.angular_velocity.y = minus16int(ord(data[14])*256.0+ord(data[13]))/32768.0*2000.0*3.14/180.0
    imuMsg.angular_velocity.z = minus16int(ord(data[16])*256.0+ord(data[15]))/32768.0*2000.0*3.14/180.0

    roll = minus16int(ord(data[23])*256.0+ord(data[22]))/32768.0*180.0*3.14/180.0
    pitch = minus16int(ord(data[25])*256.0+ord(data[24]))/32768.0*180.0*3.14/180.0
    yaw = minus16int(ord(data[27])*256.0+ord(data[26]))/32768.0*180.0*3.14/180.0

    q = quaternion_from_euler(roll, pitch, yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp = rospy.Time.now()
    imuMsg.header.frame_id = 'base_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    rate.sleep()
ser.close