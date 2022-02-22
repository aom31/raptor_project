# python 3.8
#run roscore,  rosrun joy joy_node ,rosrun steering_pkg test.py
import time
import serial
ser = serial.Serial(
    port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0', #usbพอร์ทจอย ด้านซ้ายบน พอร์ทมอเตอร์ ด้านหลัง
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS
)
ser.isOpen()
while 1 :
    out = ''
    angle = input(">>")
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

