from distutils.log import error
from socket import timeout
import serial
import time
import pynmea2
import matplotlib.pyplot as plt
import math
import time 
import pandas as pd 
import numpy as np
import datetime
import utm 

#param variable
port1 = "COM13"     #port usb that connect rover in your computer for gnss f9p
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

dt = 0.2           #time Hz define 5Hz
WAYPOINTS_file = 'pathRef_linearUTM.csv'    #put file record waypoint that reference for tracking 
L = 1.68          #m wheel base of vehicle
Kp = 100
Ki = 20
Kd = 50
sum_error_cte =0
prev_error_cte = 0
error_positive_negative = 0

def steer_input(steer):
        out = ''
        angle = steer    #decimal number
        print("angle yaw current is %f " %steer)
        # angle = int(angle1)
        if angle == 'exit' :
            ser2.close()
            exit()
        else :
            # data = ("acec21"+f'{int(angle):0>8X}')
            # sumCheck = hex(sum(int(data[i:i+2],16) for i in range(0, len(data), 2)))[3:]
            data = ("acec21"+'{:0>8X}'.format(int(angle) & (2**32-1)))
            sumCheck = hex(sum(int(str(data[i:i+2]),base = 16) for i in range(0, len(data), 2)))[3:]
            #print(bytes.fromhex((data+sumCheck).lower()))
            ser2.write(bytes.fromhex((data+sumCheck).lower()))
            time.sleep(1)
            while ser2.inWaiting() > 0:
                output = ser2.read(1)
                out += str(output.hex())
            if out != '':
                 print(out)
        return steer
 
def main():
    #=== load waypoint =============
    waypoints_file = WAYPOINTS_file
    with open (waypoints_file) as f:
        waypoints = pd.read_table(f, sep=',', header=0, names=['x','y'])
        waypoints_np = np.array(waypoints)
    #input waypoint reference 
    y_waypointRef = waypoints.x
    x_waypointRef = waypoints.y
    Dis = math.sqrt((x_waypointRef[0]-x_waypointRef[376])**2+(y_waypointRef[0]-y_waypointRef[376])**2) #distance point start to stop
    #print("value distance is %f" %Dis)  #56.56m
    #find linear equretion
    slop  = (x_waypointRef[376]-x_waypointRef[0])/(y_waypointRef[376]-y_waypointRef[0])#find slop of linear equation -147.302103
    #print("slop is %f" %slop)
    # find C fo linear equation
    c  = x_waypointRef[0]-slop*y_waypointRef[0]
    #print("value C is %f"%c)
    A = slop
    B = -1
   
    return A,B,c

def cte(x_car,y_car,A,B,c):

    cross_track_error = abs((A*x_car)+(B*y_car)+c)/math.sqrt(A**2+B**2)
    #linear x tell cte scope is 661486.950 to  661487.299  cte is 0.01 to 0.09
    #cte is positive +cte left   is 661487.3008148358       to 661488.5024203522 cte is 0.1 to 1.5 
    #cte is positive -cte right is  661486.9422366153          to 661485.416215819    cte is     1.680500
    
    return cross_track_error
def cte_positive_negative(x_north,error_cte):
    r = x_north
    y = error_cte
    global error_positive_negative
    if x_north >= 661487.3008148358 and x_north <= 661488.5024203522:
        error_positive_negative = y
        print("error cte is positive %f" %error_positive_negative)
    elif x_north <= 661486.9422366153 and x_north >= 661485.416215819:
        error_positive_negative = (-1)*y
        print("erorr cte negative is %f"%error_positive_negative)
    return error_positive_negative

def pid_angle(cte_p_n,cte_prev):
    global sum_error_cte
    global prev_error_cte 
    #print("*******************************")
    #print("current error is %f" %cte_p_n)
    sum_error_cte += cte_p_n
    print("sum error is %f " %sum_error_cte)
    P = Kp*cte_p_n
    I = Ki*sum_error_cte
    D = Kd*(error_cte-cte_prev)
    yaw_expect = P+I+D
    yaw_sent = yaw_expect
    print("yaw angle is %f" %yaw_sent)
    #prev_error_cte = pre_error
    #print("previus error is %f " %prev_error_cte)
    print("**********************************")
    return yaw_sent

def Previous_state(x_north,y_east,cte_p_n):
    x = []
    y = []
    cte_prev = []
    x.append(x_north)
    y.append(y_east)
    cte_prev.append(cte_p_n)
    
    return cte_prev
    
if __name__ == '__main__':
    
    #=============Serial position from rover =================================
    while True:
        data = ser.readline()
        #output = b'$GNGGA,173534.70,1339.04546,N,10029.60344,E,1,12,0.56,6.3,M,-27.8,M,,*63\r\n'
        gngga_data = data.split(b",")
        if gngga_data[0] == b"$GNGGA":
            newmsg=pynmea2.parse(data.decode("utf-8"))
            lat=newmsg.latitude
            lng=newmsg.longitude
            xy = utm.from_latlon(lat,lng)
            #gps = lat,lng 
            gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
            # sleep(second) 5Hz = 1/5=0.2sec
            #position current from Vehicle 
            lat_y = lat
            lng_x = lng
            x_north = xy[0]
            y_east = xy[1]
            utm_locationre = x_north,y_east
            
            #print(type(x_north))
            print("current x %f" %x_north)
            print("current y %f" %y_east)
            a,b,c = main()
            error_cte = cte(x_north,y_east,a,b,c)
            cte_p_n = cte_positive_negative(x_north,error_cte)
            cte_prev = Previous_state(x_north,y_east,cte_p_n)
            print("cte prev is ")
            print(cte_prev)
            yaw_expect = pid_angle(cte_p_n,cte_prev)
            time.sleep(dt)
            #steer_input(yaw_expect)
            
            
            

            
            
            
            

            
            
            
            
           

            
            
            

                
                
    


        
          

