# python 3.8
#run roscore,  rosrun joy joy_node ,rosrun steering_pkg test.py
import time
from numpy import angle
import numpy as np  
import pandas as pd
import math
import matplotlib.pyplot as plt     
import serial
ser = serial.Serial(
    port = '/dev/ttyUSB0', #usbพอร์ทจอย ด้านซ้ายบน พอร์ทมอเตอร์ ด้านหลัง /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser.isOpen()

 #set param 
dt = 0.1 #s 
L = 1.68 #m wheel base of vehicle
  #gain controller for control steer to minimize cte
Kp = 1
Ki = 0.1
Kd = 0.5
#controller PID output is yaw_expect
'''
PID cal YAW
        yaw_expect = [] 
        P = Kp * CTE_t
        I = last_integral + Ki *CTE_t *(t - last_time) 
        D = Kd*(CTE_t - last_CTE) 
        yaw_expect = P+I+D
'''
WAYPOINTS_file = 'xy.csv'      #put file record waypoint that reference for tracking 

class LBicycle_vehicle:
        def __init__(self,x = 0.0,y = 0.0,v = 0.0,yaw = 0.0):
            self.x = x
            self.y = y  
            self.v = v    
            self.yaw = yaw
            
        def update_state(self,delta):
            delta = np.clip(delta,-3000,3000)
            self.x += self.v * math.cos(self.yaw)*dt
            self.y += self.v *math.sin(self.yaw)*dt     
            self.yaw += self.v/L * math.tan(delta)*dt     
            self.v = 10


def steer_input(steer):
        out = ''
        angle = steer
        # angle = int(angle1)
        if angle == 'exit' :
            ser.close()
            exit()
        else :
            data = ("acec21"+f'{int(angle):0>8X}')
            sumCheck = hex(sum(int(data[i:i+2],16) for i in range(0, len(data), 2)))[3:]
            print(bytes.fromhex((data+sumCheck).lower()))
            ser.write(bytes.fromhex((data+sumCheck).lower()))
            time.sleep(1)
            while ser.inWaiting() > 0:
                output = ser.read(1)
                out += str(output.hex())
            if out != '':
                 print(out)
        
        return angle
    
def main():
     #=== load waypoint =============
    waypoints_file = WAYPOINTS_file
    with open (waypoints_file) as f:
        waypoints = pd.read_table(f, sep=',', header=0, names=['x','y'])
        waypoints_np = np.array(waypoints)
        
    yaw_expect = [] 
    P = Kp * CTE_t
    I = last_integral + Ki *CTE_t *(t - last_time) 
    D = Kd*(CTE_t - last_CTE) 
    yaw_expect = P+I+D
    current_angle = steer_input(yaw_expect)

if __name__ == '__main__':
    main()
    # current_angle = steer_input(input(">>"))
    
    
    

