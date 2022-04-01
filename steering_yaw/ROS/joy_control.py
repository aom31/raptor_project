#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
import serial
import binascii
import struct
import numpy as np
#roscore , rosrun joy joy_node, rosrun test_code joy_control.py

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
    angle = data.axes[0]  * 210  *27
    position = data.axes[0]
    print("The angle position : %f" %angle)
    print("The position : %f"  %position)
    data = ("acec21"+'{:0>8X}'.format(int(angle) & (2**32-1)))
    sumCheck = hex(sum(int(str(data[i:i+2]),base = 16) for i in range(0, len(data), 2)))[3:]
    ser.write(bytearray.fromhex((data+sumCheck).lower()))
    
    
    steering_angle = angle
    
    print("get steering %.2f" %steering_angle)
    
    # topic_steer = rospy.Publisher("raptor", Float64, queue_size=1)
    
    # # rate = rospy.Rate(10)
    
    # while not rospy.is_shutdown():
    #     rospy.loginfo(angle)
    #     pub = ("This angle from steering : %.2f" %angle)
    #     # topic_steer.publish(pub)
    #     topic_steer.publish(angle)
        
        # rate.sleep()
    
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy", Joy, tj_callback, queue_size=1)
    rospy.spin()
    ser.close()
    
 
if __name__ == '__main__':
    
    try:
        
        listener()
        
        
    except rospy.ROSInterruptException:
        pass 
