#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
import serial
import binascii
import struct
import numpy as np
#roscore , rosrun joy joy_node, rosrun steering_pkg test.py

ser = serial.Serial(
    port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0',
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser.isOpen()

def tj_callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[0]
    angle = data.axes[0]  * 720  *27
    position = data.axes[0]
    print("The angle position : %f" %angle)
    print("The position : %f"  %position)
    data = ("acec21"+'{:0>8X}'.format(int(angle) & (2**32-1)))
    sumCheck = hex(sum(int(str(data[i:i+2]),base = 16) for i in range(0, len(data), 2)))[3:]
    ser.write(bytearray.fromhex((data+sumCheck).lower()))
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy", Joy, tj_callback, queue_size=1)
    rospy.spin()
    ser.close()
if __name__ == '__main__':
    listener()
