
import time
from numpy import angle
import numpy as np  
import pandas as pd
import math
import matplotlib.pyplot as plt     
import serial
ser2 = serial.Serial(
    port = 'COM17', #usbพอร์ทจอย ด้านซ้ายบน พอร์ทมอเตอร์ ด้านหลัง /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser2.isOpen()

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
        time.sleep(1)
        while ser2.inWaiting() > 0:
            output = ser2.read(1)
            out += str(output.hex())
        if out != '':
            print(out)
        


if __name__ == '__main__':
    for i in range(0,600,10):

        steer_input(i)
        print(i)
        time.sleep(1)
            
    
    
    
    
