from socket import timeout
import serial
import time
import pynmea2
import matplotlib.pyplot as plt
import math
# ser = serial.Serial('COM6', baudrate=115200, timeout=1)
# ser_bytes = ser.readline()
port = "COM6"  
ser = serial.Serial(port, baudrate = 115200)
while True:
    data = ser.readline()
    #output = b'$GNGGA,173534.70,1339.04546,N,10029.60344,E,1,12,0.56,6.3,M,-27.8,M,,*63\r\n'
    gngga_data = data.split(b",")
    # gngga_data[0] = b'$GNGGA'
    if gngga_data[0] == b"$GNGGA":
        #print(data)
        newmsg=pynmea2.parse(data.decode("utf-8"))
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
        print(gps)
        
        
    
    
    
    