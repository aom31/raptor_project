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
port1 = "COM26"     #port usb that connect rover in your computer for gnss f9p
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
WAYPOINTS_file = 'refLinear_tracking.csv'    #put file record waypoint that reference for tracking 
                                                #y_east linear route forward, x_north calculate cte error from positoion y_east
L = 1.68          #m wheel base of vehicle
Kp = 50
Ki = 0.002
Kd = 0.9
dt = 0.2
sum_error_cte =0
prev_error_cte = 0
error_positive_negative = 0
fileWrite = 'see_dataPrint.csv'


def main():
    #=== load waypoint =============
    waypoints_file = WAYPOINTS_file
    with open (waypoints_file) as f:
        waypoints = pd.read_table(f, sep=',', header=0, names=['x','y'])
        waypoints_np = np.array(waypoints)
    #input waypoint reference 
    x_axiswaypoint_yEAST = waypoints.x     #run direct forward in axis
    y_axiswaypoint_xNORTH = waypoints.y

    
    Dis = math.sqrt(( x_axiswaypoint_yEAST[0]- x_axiswaypoint_yEAST[412])**2+(y_axiswaypoint_xNORTH[0]-y_axiswaypoint_xNORTH[412])**2) #distance point start to stop
    #print("value distance is %f" %Dis)  #value distance is 59.056766   and Measuring is 58m
    #find linear equretion
    slop  = (y_axiswaypoint_xNORTH[412]-y_axiswaypoint_xNORTH[0])/( x_axiswaypoint_yEAST[412]- x_axiswaypoint_yEAST[0])#find slop of linear equation -147.302103
    print("slop is %f" %slop)
    # find C fo linear equation
    c  = y_axiswaypoint_xNORTH[0]-slop* x_axiswaypoint_yEAST[0]
    #print("value C is %f"%c)
    A = slop
    B = -1
    return A,B,c

def cte_current(x_car,y_car,A,B,c):

    cross_track_error = abs((A*x_car)+(B*y_car)+c)/math.sqrt(A**2+B**2)
    #print(cross_track_error)
    #linear x tell cte scope is 661486.950 to  661487.299  cte is 0.01 to 0.09
    #cte is positive +cte left   is 661487.3008148358       to 661488.5024203522 cte is 0.1 to 1.5 
    #cte is positive -cte right is  661486.9422366153          to 661485.416215819    cte is     1.680500
    
    return cross_track_error

def cte_positive_negative(x_north,error_cte):
    r = x_north
    y = error_cte
    global error_positive_negative
    #print(r,y)
    
    if x_north >= 1509602.5441973794 and x_north <= 1509610.2045558211:      #cte range 0.77m -1.95m
        error_positive_negative = y
        print("error cte is positive %f" %error_positive_negative)
    elif x_north <= 1509649.8721396332 and x_north >= 1509599.8578825532:    #cte 0.85m -
        error_positive_negative = (-1)*y
        print("erorr cte negative is %f"%error_positive_negative)
    return error_positive_negative

def Previous_state(x_north,y_east,cte_p_n):
    x = []
    y = []
    cte_prev = []
    x.append(x_north)
    y.append(y_east)
    cte_prev.append(cte_p_n)
    
    return cte_prev

def pid_angle(cte_p_n,cte_prev):
    global sum_error_cte
    global prev_error_cte 
    #print("*******************************")
    #print("current error is %f" %cte_p_n)
    sum_error_cte += cte_p_n
    
    print("sum error is %f " %sum_error_cte)
    P = Kp*cte_p_n
    I = Ki*sum_error_cte
    D = Kd*((cte_p_n-cte_prev)/dt)
    yaw_expect = P+I+D
    yaw_sent = yaw_expect
    print("yaw angle current is %f" %yaw_sent)
    #prev_error_cte = pre_error
    #print("previus error is %f " %prev_error_cte)
    print("**********************************")
    return yaw_sent

def steer_input(steer):
    out = ''
    angle = steer*27 #27 from 10000 rpm/360 degree
    #define min steer 0, max steer 300 if decimal number 
    #งานทีี่เหลือคือ แมปค่าเลขฐานสิบนั้นให้เป็นองศา
    if angle == 'exit' :
        ser2.close()
        exit()
    else :
        data = ("acec21"+'{:0>8X}'.format(int(angle) & (2**32-1)))
        sumCheck = hex(sum(int(str(data[i:i+2]),base = 16) for i in range(0, len(data), 2)))[3:]
        ser2.write(bytearray.fromhex((data+sumCheck).lower()))
        time.sleep(1)
        while ser2.inWaiting() > 0:
            output = ser2.read(1)
            out += str(output.hex())
        if out != '':
            print(out)
 

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
            utem_position = y_east,x_north
    #===========================================================================
    #=== import file path reference to compute==
            a,b,c = main()                       #receive param linear eqaution of path ref
    #cal CTE======
            error_current = cte_current(y_east,x_north,a,b,c)
            cte_pos_neg = cte_positive_negative(x_north,error_current)    #data can tell cte + - 
    # store data previous
            cte_previous = Previous_state(x_north,y_east,cte_pos_neg)
    #Cal yaw angle that expect from PID controller
            yaw_expect = pid_angle(cte_pos_neg,cte_previous)

    #===========input data yaw angle (degree) for control Steering Motor=============
            steer_input(yaw_expect)

            #======= For write data to file CSV ========================
            DataWantWrite = utem_position                                     #==Change==
            # with open('fileWrite', 'a', newline='') as f:
            #     writer = csv.writer(f,delimiter=",")
            #     writer.writerow(DataWantWrite)