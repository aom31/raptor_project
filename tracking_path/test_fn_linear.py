from socket import timeout
import serial
import time
import pynmea2
import matplotlib.pyplot as plt
import math
import time 
import pandas as pd 
import numpy as np

#param variable
port = "COM13"     #port usb that connect rover in your computer for gnss f9p
ser = serial.Serial(port, baudrate = 115200)                  
dt = 0.2           #time Hz define 5Hz
WAYPOINTS_file = 'waypoint_refLinear.csv'    #put file record waypoint that reference for tracking 
L = 1.68          #m wheel base of vehicle

        
def steer_input(steer):

        out = ''
        angle = steer    #decimal number
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
    x_waypointRef = waypoints.x
    y_waypointRef = waypoints.y
    Dis = math.sqrt((x_waypointRef[0]-x_waypointRef[315])**2+(y_waypointRef[0]-y_waypointRef[315])**2) #distance point start to stop
    print("value distance is %f" %Dis)
    #find linear equretion
    slop  = (y_waypointRef[315]-y_waypointRef[0])/(x_waypointRef[315]-x_waypointRef[0])#find slop of linear equation
    #print("slop is %f" %slop)
    # find C fo linear equation
    c  = y_waypointRef[0]-slop*x_waypointRef[0]
    #print("value C is %f"%c)
    #print("linear equation is %f A+ %f Y+ %f = 0"%x_waypointRef[0] %y_waypointRef[0] %c)
    A = slop
    B = -1
    current_x,current_y = a
    return A,B,c

def update_state(x,y,yaw,v):
    x_current_car = []
    y_current_car = []
    yaw_current_car = []
    v_current = []
    x_current_car = x 
    y_current_car = y 
    yaw_current_car = yaw 
    v_current = v  

    return x_current_car,yaw_current_car,yaw_current_car

    
def cte(x_car,y_car,A,B,c):
    cross_track_error = abs((A*x_car)+(B*y_car)+c)/math.sqrt(A**2+B**2)
    if cross_track_error != 0:
        print("CTE is %f" %cross_track_error)
    elif cross_track_error == 0:
        print("No error")

if __name__ == '__main__':
    #=============Serial position from rover =================================
    while True:
        data = ser.readline()
    #output = b'$GNGGA,173534.70,1339.04546,N,10029.60344,E,1,12,0.56,6.3,M,-27.8,M,,*63\r\n'
        gngga_data = data.split(b",")
    # gngga_data[0] = b'$GNGGA'
        if gngga_data[0] == b"$GNGGA":
            newmsg=pynmea2.parse(data.decode("utf-8"))
            lat=newmsg.latitude
            lng=newmsg.longitude
            gps = [lat,lng] 
        #gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
            #print(gps)   #show data
            time.sleep(0.2)  # sleep(second) 5Hz = 1/5=0.2sec
            lat_x = lat
            lng_y = lng
            a,b,c = main()
            yaw = steer_input()

            cte(lat_x,lng_y,a,b,c)
            update_state(lat_x,lng_y)



