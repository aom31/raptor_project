import time
from numpy import angle
import math  
import serial
import pynmea2
import utm 
import pandas as pd
import matplotlib.pyplot as plt   
import numpy as np  
import csv

port1 = "COM35"                                  #port usb that connect rover in your computer for gnss f9p
ser = serial.Serial(port1, baudrate = 115200)


if __name__ == '__main__':
    while True:
        data = ser.readline()                           #output = b'$GNGGA,173534.70,1339.04546,N,10029.60344,E,1,12,0.56,6.3,M,-27.8,M,,*63\r\n'
        gngga_data = data.split(b",")
        if gngga_data[0] == b"$GNGGA":
            newmsg=pynmea2.parse(data.decode("utf-8"))
            lat=newmsg.latitude                         #get data from gnss is latitude and longtitude
            lng=newmsg.longitude
            gps = lat,lng
            xy = utm.from_latlon(lat,lng)               #convert data lat lng to utm-xy 
            x_north = xy[1]                             #for check cte 
            y_east = xy[0]                              #for track forward linear
            y_east_track = y_east +0.42
            utem_position = x_north,y_east_track 
            #print(utem_position)                       
            # acceptable error value : side each 0.15 sum = 0.3 line trajectory
            # left side=  661487.1453383866 /cte value acceptable =0.14975929571138333      // right side =661487.013138559,0.1470221906921283      /cte acceptable = 
            # test_see1 = gps
            # with open('refLinearCurve_latlng.csv', 'a', newline='') as f:  
            #     writer = csv.writer(f,delimiter=",")    
            #     writer.writerow(test_see1) 

            test_see2 = gps
            with open('ref_linear2_latlng.csv', 'a', newline='') as f:  
                writer = csv.writer(f,delimiter=",")    
                writer.writerow(test_see2)
            test_see2 = utem_position
            with open('ref_linear2_UTM.csv', 'a', newline='') as f:  
                writer = csv.writer(f,delimiter=",")    
                writer.writerow(test_see2)
            #===================#
            # 1 (1509653.4503615787, 661486.9866814984) start
            # 1 (1509602.1929960449, 661487.1577268437) stop
            #2 (1509600.7917398307, 661487.1663461719) start curve 
            #2 stop (1509593.9769328258, 661488.362279179)
            #3 start curve2 (1509591.5631623606, 661488.6295672964)
            #3 stop curve 2  (1509586.3937104405, 661493.5298645048)
            #4 start linear2 (1509586.419246495, 661494.6837220492)
            #4 stop linear2  (1509586.5847669786, 661518.5924456262)

