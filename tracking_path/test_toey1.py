from cmath import sin, sqrt
from dis import dis
from distutils.log import error
import math
import csv
from re import X
import re
from turtle import distance
from matplotlib.contour import ContourLabeler
import pandas as pd
import numpy as np
import matplotlib.pyplot  as plt
import pandas as pd
from scipy.misc import derivative
import serial
import time

#data  = pd.read_csv('xy.csv')
#x = data['X'].unique()
#y = data['Y'].unique()
#print(data.head(10))  #ข้อมูล 10 เเถวเเรก
#print(data.tail(10))  #ข้อมูล 10 เเถวหลัง
#print(x[0])
#print(y[0])

ser = serial.Serial(
    port = '/dev/tty0', #usbพอร์ทจอย ด้านซ้ายบน พอร์ทมอเตอร์ ด้านหลัง /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser.isOpen()

dt = 0.1

def distance(x,y): 
        dis = []
        for i in range(0,251,1): #for i=0,i<251,i++
                dis.append(math.sqrt((x[i]-x[i+1])**2+(y[i]-y[i+1])**2)) #หาระยะระหว่างจุด
        return dis

def slop(x,y):
        sl = []
        for i in range(251): 
                sl.append((y[i+1]-y[i])/(x[i+1]-x[i]))  # m=(y2-y1)/(x2-x1)
        return sl

def C (x,y,w):
        C_C = []
        for i in range (251):
                C_C.append(y[i]-(w[i]*x[i+1]))             # c= y1-(m*x2)
        return C_C

def CTE (x,y,w,e):
        B = -1
        CTE = []
        for i in range(251): 
                CTE.append(abs(w[i]*x[i]+(-1)*y[i]+e[i])/math.sqrt((w[i]*w[i])+(-1*-1)))  # A=m , B=-1
        return CTE

class Current_State: #เก็บค่าสถานะปัจจุบันของรถ
    
    def __init__(self, x = 0.0, y = 0.0, v = 0.0, theta = 0.0):
        self.x = x
        self.y = y
        self.v = v
        self.theta = theta
        
    def update_state(self, deg):
        self.theta += deg 
        self.x += self.v * dt * math.cos(self.theta)
        self.y += self.v * dt * math.sin(self.theta)

class Previous_States: #store information of all the previous states
    
    def __init__(self):
        self.x = []
        self.y = []
        self.v = []
        self.theta = []
        self.t = []
    
    def add_to_list(self, state, t):
        self.x.append(state.x)
        self.y.append(state.y)
        self.v.append(state.v)
        self.theta.append(state.theta)
        self.t.append(t)

class PID:
        def __init__(self, kp=1, ki=1, kd=1,i=0,t=[]): 
                self.kp = kp
                self.ki = ki
                self.kd = kd
                self.i = i
                self.t = []
                self.Proportional = []
                self.Integral = []
                self.Derivative = []

        def PP (self,controller,kp,t):
                for i in range(t):
                        Proportional = kp * CTE[i]
                        self.Proportional.append(controller.Proportional)
        
        
        def II (self,controller,ki,Integral,t):
                for i in range(t):
                        Integral = ki * CTE[i]
                        self.Integral.append(controller.Integral)
                        
        def DD (self,controller,kd,t):
                for i in range(t):
                        Derivative = kd*(CTE[t]+CTE[t-1])
                        self.Derivative.append(controller.Derivative)
def steer_input(o):
        out = ''
        angle = o
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

if _name_ == '_main_':
        WAYPOINTS_file = 'xy.csv' 
        data  = pd.read_csv('xy.csv')
        # x = data['X'].unique()
        # y = data['Y'].unique()
        waypoints_file = WAYPOINTS_file
        with open (waypoints_file) as f:
                waypoints = pd.read_table(f, sep=',', header=0, names=['x','y'])
                waypoints_np = np.array(waypoints)
        x = waypoints.x
        y = waypoints.y
        #print(x,y)
        # print("show data waypoint x and y : %f %f" %x %y)
        q =[]
        q = distance(x,y)
        w = slop(x,y)
        e = C(x,y,w)
        u = CTE(x,y,w,e)
        
        print(u)
        #o = 1000
        #current_angle = steer_input(o)