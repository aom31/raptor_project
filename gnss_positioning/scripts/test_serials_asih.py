# Explain thing will use word explain >>>> Explain
# Extra thing that i will comment such as use with ubuntu command and what ever >>>> Extra
# ........................................................................................
# Explain >>>> get data from serial then use serial
import serial
# Explain >>>> This one use for access to data of time
import time
# Explain >>>> common string operation as i note seperate for review
import string
# Explain >>>> This is got nmea protocal
import pynmea2
# Explain >>>>
import utm
# from pyproj import Proj
import math
# Extra >>>> use this command (dmesg | grep tty ) in command panel to get which port is use connected
# Extra >>>> now i get (/dev/ttyACM0)
# Explain >>>> i make port value same port whichi i connected (left side lower one on PC board)
port="/dev/ttyACM0"
# Depend on which type information i want selfdrivecar will may be use GGA or RMC enough
# Interpreted sentences
# $GPBOD - Bearing, origin to destination
# $GPBWC - Bearing and distance to waypoint, great circle
# $GPGGA - Global Positioning System Fix Data
# $GPGLL - Geographic position, latitude / longitude
# $GPGSA - GPS DOP and active satellites
# $GPGSV - GPS Satellites in view
# $GPHDT - Heading, True
# $GPR00 - List of waypoints in currently active route
# $GPRMA - Recommended minimum specific Loran-C data
# $GPRMB - Recommended minimum navigation info
# $GPRMC - Recommended minimum specific GPS/Transit data
# $GPRTE - Routes
# $GPTRF - Transit Fix Data
# $GPSTN - Multiple Data ID
# $GPVBW - Dual Ground / Water Speed
# $GPVTG - Track made good and ground speed
# $GPWPL - Waypoint location
# $GPXTE - Cross-track error, Measured
# $GPZDA - Date & Time
# NMEA PROTOCAL STRUCTURE
# 11
# 1 2 3 4 5 6 7 8 9 10 | 12 13 14 15
# | | | | | | | | | | | | | | |
# $--GGA,hhmmss.ss,ddmm.mm,a,ddmm.mm,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
# Example two letterwill tell me satelite and three last letter depend on which type i want GGA
# GGA - Global Positioning System Fix Data
# This is one of the sentences commonly emitted by GPS units.
# Time, Position and fix related data for a GPS receiver.
# GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47
# Field Number:
# 1 UTC of this position report, hh is hours, mm is minutes, ss.ss is seconds.
# 2 Latitude, dd is degrees, mm.mm is minutes
# 3 N or S (North or South)
# 4 Longitude, dd is degrees, mm.mm is minutes
# 5 E or W (East or West)
# 6 GPS Quality Indicator (non null)
# 6.0 - fix not available,
# 6.1 - GPS fix,
# 6.2 - Differential GPS fix (values above 2 are 2.3 features)
# 6.3 = PPS fix
# 6.4 = Real Time Kinematic
# 6.5 = Float RTK
# 6.6 = estimated (dead reckoning)
# 6.7 = Manual input mode
# 6.8 = Simulation mode
# 7 Number of satellites in use, 00 - 12
# 8 Horizontal Dilution of precision (meters)
# 9 Antenna Altitude above/below mean-sea-level (geoid) (in meters)
# 10 Units of antenna altitude, meters
# 11 Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
# 12 Units of geoidal separation, meters
# 13 Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
# 14 Differential reference station ID, 0000-1023
# 15 Checksum
# Explain >>>> baudrate 115200 recommend as Ublox GNSS F9R datasheet

# Explain >>>> parity
# Explain >>>> stopbits Number of stop bits. Possible values
# Explain >>>> serial.STOPBITS_ONE
# Explain >>>> bytesize
ser=serial.Serial(port, baudrate=115200, parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
while True:
    port="/dev/ttyACM0"
    ser=serial.Serial(port, baudrate=115200, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    ##print(newdata)
    #print(newdata[0:6] == "$GPRMC")
    # print(newdata[0:6] == "$GNGGA")
    # #if (newdata[0:6] == b"$GPRMC"):
    if (newdata[0:6] == b"$GPGGA"):
        print("UbloxGnssF9R")
        newmsg=pynmea2.parse(newdata.decode("utf-8"))
        ##print(newmsg)
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
        print(gps)
        # From here start converting
        # The syntax is utm.from_latlon(LATITUDE, LONGITUDE)
        # utm.from_latlon(lat, lng)
        # output should be In the result EASTING and NORTHING will have the same shape. ZONE_NUMBER and ZONE_LETTER
        # Output format 0 x 1 y
        # Study UTM
        # Usage
        # import utm
        # Convert a (latitude, longitude) tuple into an UTM coordinate:
        # utm.from_latlon(51.2, 7.5)
        # >>> (395201.3103811303, 5673135.241182375, 32, 'U')
        # The syntax is utm.from_latlon(LATITUDE, LONGITUDE).
        # The return has the form (EASTING, NORTHING, ZONE NUMBER, ZONE LETTER).
        xy = utm.from_latlon(lat,lng)
        print(xy)
        print ( "UTM zone is ", xy[2],xy[3])
        print ("UTM Northing is ", xy[0])
        print ("UTM Easting is", xy[1])
        print ( "Value X and Value Y are " ,xy[0] , xy[1] )