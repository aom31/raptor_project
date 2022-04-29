import time
from numpy import angle
import numpy as np  
import pandas as pd
import math
import matplotlib.pyplot as plt     
from distutils.log import error
from socket import timeout
import serial
import pynmea2
import utm 
import csv 

#====== Port conection ========
port1 = "COM32"     #port usb that connect rover in your computer for gnss f9p
port2 = "COM17"      #port usb that connect steering rs232
ser = serial.Serial(port1, baudrate = 115200)    
ser2 = serial.Serial(
    port2, 
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser2.isOpen()
#============Param variable =====
WAYPOINTS_file = 'refLinear_utm.csv'    #put file record waypoint that reference for tracking 
                                                #y_east linear route forward, x_north calculate cte error from positoion y_east
L = 1.68          #m wheel base of vehicle
Kp = 80
Ki = 0.002
Kd = 0.9
dt = 0.2
sum_error_cte =0
prev_error_cte = 0
error_positive_negative = 0
yaw_expect =0
yaw_control = 0
# def main():
#     #=== load waypoint =============
#     waypoints_file = WAYPOINTS_file
#     with open (waypoints_file) as f:
#         waypoints = pd.read_table(f, sep=',', header=0, names=['x','y'])
#     #input waypoint reference 
#     x_axiswaypoint_NORTH = waypoints.x     #run direct forward in axis
#     y_axiswaypoint_EAST = waypoints.y
#     #print(x_axiswaypoint_NORTH,y_axiswaypoint_EAST)
    
#     Dis = math.sqrt(( x_axiswaypoint_NORTH[0]- x_axiswaypoint_NORTH[88])**2+(y_axiswaypoint_EAST[0]-y_axiswaypoint_EAST[88])**2) #distance point start to stop
#     # #print("value distance is %f" %Dis)  #value distance is 59.056766   and Measuring is 58m
#     # #find linear equretion
#     slop  = (y_axiswaypoint_EAST[88]-y_axiswaypoint_EAST[0])/( x_axiswaypoint_NORTH[88]- x_axiswaypoint_NORTH[0])#find slop of linear equation -147.302103
#     print("slop is %f" %slop)
#     # # find C fo linear equation
#     c  = y_axiswaypoint_EAST[0]-slop* x_axiswaypoint_NORTH[0]
#     print("value C is %f"%c)
#     A = slop
#     B = -1
#     return A,B,c

def cte_current(x_car,y_car,A,B,c):

    cross_track_error = abs((A*x_car)+(B*y_car)+c)/math.sqrt(A**2+B**2)
    
    return cross_track_error

def cte_positive_negative(y_east,error_cte):
    r = y_east
   
    global error_positive_negative,cte_previous
    cte_previous = error_positive_negative
    #print(r,y)
    #linear 1509630.8806535355 661486.7829172977
    
    if y_east >= 661486.90607257 and  y_east <= 661488.3534151604:      #cte range 0.77m -1.226195m
        error_positive_negative = error_cte
        #print("error cte is positive %f" %error_positive_negative)
    elif y_east<= 661486.9058593663 and y_east >=  661485.6607180513:    #cte 0.85m - -1.283984 m
        error_positive_negative = (-1)*error_cte
        #print("erorr cte negative is %f"%error_positive_negative)
    else:
        error_positive_negative = error_cte

    return error_positive_negative

def Previous_state():
    
    return cte_previous

def pid_angle(cte_p_n,cte_prev):
    global sum_error_cte
    global yaw_expect,prev_error_cte,yaw_previous
    
    yaw_previous = yaw_expect
    #print("*******************************")
    #print("current error is %f" %cte_p_n)
    sum_error_cte += cte_p_n
    
    #print("sum error is %f " %sum_error_cte)  #error range [-1.28,1.28] yaw output [-300,300] so gain [-234,234]
    P = Kp*cte_p_n
    # I = Ki*sum_error_cte
    # D = Kd*((cte_p_n-cte_prev)/dt)
    yaw_expect = -1*P

    return yaw_expect

def steer_input(steer):
    out = ''
    angle = steer*27 #27 from degree angle *( 10000 rpm/360 degree )  กำหนดช่วงมุมเลี้ยว decimal min20,max300 to  [] degree
    #define min steer 0, max steer 300 if decimal number 
    #งานทีี่เหลือคือ แมปค่าเลขฐานสิบนั้นให้เป็นองศา
    if angle == 'exit' :
        ser2.close()
        exit()
    else :
        data = ("acec21"+'{:0>8X}'.format(int(angle) & (2**32-1)))
        sumCheck = hex(sum(int(str(data[i:i+2]),base = 16) for i in range(0, len(data), 2)))[3:]
        ser2.write(bytearray.fromhex((data+sumCheck).lower()))
        # time.sleep(1)
        while ser2.inWaiting() > 0:
            output = ser2.read(1)
            out += str(output.hex())
        if out != '':
            print(out)
 
def Previous_state_yaw():
    

    return yaw_previous


def angle_controlMotor(cte_pos_neg,yaw_expect,yaw_prev):
    global yaw_control 
    # if cte_pos_neg < 0.05 or cte_pos_neg > -0.05 and cte_pos_neg <= 0.1:
    #     yaw_control = 10

    if cte_pos_neg > 0.05  :
        yaw = abs(yaw_expect - yaw_previous)
        #print(yaw)
        if yaw >= 1:
            yaw_control = yaw_expect

    elif cte_pos_neg < -0.05:
        yaw = abs(yaw_expect - yaw_previous)
        if yaw >= 1:
            yaw_control = yaw_expect
    else:
        yaw_control = 10
    
    #cte + left side yaw -   and cte - right side yaw +
    return yaw_control

if __name__ == '__main__':
    
    #=============Serial position from rover =================================
    while True:
        
        data = ser.readline()
        #output = b'$GNGGA,173534.70,1339.04546,N,10029.60344,E,1,12,0.56,6.3,M,-27.8,M,,*63\r\n'
        gngga_data = data.split(b",")
        if gngga_data[0] == b"$GNGGA":
            newmsg=pynmea2.parse(data.decode("utf-8"))
            lat=newmsg.latitude                         #get data from gnss is latitude and longtitude
            lng=newmsg.longitude
            gps = lat,lng
            xy = utm.from_latlon(lat,lng)               #convert data lat lng to utm-xy 
            x_north = xy[1]                             #for check cte 
            y_east = xy[0]                              #for track forward linear
            utem_position = x_north,y_east              #forward in x-axis north , check y-axis east
            # lat 13.650680 to utm east 661487.25   // lng  100.493001 to utm north 1509590.43
            #เราวิ่งจาก noth ไป south ทำให้ค่า utm north 1509590.43 มีค่าลดลงไปเรื่อยๆ ซึ่งเราจะกำหนดให้มันเป็นแกน x แกนที่เราวิ่งอยู่
            # east คือค่าที่เกิดจากการวิ่งเอียงออกจากเส้นตรง ซึ่งจะมีค่าเป็นช่วง ส่วนนี้เราจะนำมาคิดเป็นค่าความคาดเคลื่อน 
            #  โดย ไปทางซ้าย ค่าeast จะเพิ่มขึ้น 86-88 ไปทางขวาค่า east จะลดลง 86-85
    #===========================================================================
    #=== import file path reference to compute==
            a,b,c =  -0.004432,-1,668177.730316                     #receive param linear eqaution of path ref
    # #cal CTE======
            error_current = cte_current(x_north,y_east,a,b,c)
            cte_pos_neg = cte_positive_negative(y_east,error_current)    #data can tell cte + - 
    # # store data previous
            cte_prev = Previous_state()
    # #Cal yaw angle that expect from PID controller
            yaw_expect = pid_angle(cte_pos_neg,cte_prev)
            yaw_prev = Previous_state_yaw()
           
            yaw_controlSteering = angle_controlMotor(cte_pos_neg,yaw_expect,yaw_prev)
    # #===========input data yaw angle (degree) for control Steering Motor=============
            #print(yaw_controlSteering)
            steer_input(yaw_controlSteering)
            # time.sleep(0.01)

            #======= For write data to file CSV ========================
            # test_see = cte_pos_neg,yaw_controlSteering
            # with open('see_dataPrint.csv', 'a', newline='') as f:  
            #     writer = csv.writer(f,delimiter=",")    
            #     writer.writerow(test_see) 

           