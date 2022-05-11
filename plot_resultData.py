import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

x1 = []
y1 = []
x2 = [] 
y2 =[]
ref_latlng ='refLinear_latlng.csv' #gps= lat,lng
ref_utm ='refLinear_utm.csv'       #utm = xnorth,yeast
#================= Reference Trajectory =========================================================
# with open (ref_latlng) as f:
#     ref_data = pd.read_table(f, sep=',', header=0, names=['x','y'], lineterminator='\n')
    
# ref_lng = ref_data.x
# ref_lat = ref_data.y
# with open (ref_utm ) as f:
#     ref_dataU = pd.read_table(f, sep=',', header=0, names=['x','y'], lineterminator='\n')
    
# ref_xNorth = ref_dataU.x
# ref_yEast = ref_dataU.y
#===========================Result ================================================================
# FileData1 = '3result_CTElinear2AndYAW.csv'
# with open (FileData1) as f:
#     data_result1 = pd.read_table(f, sep=',', header=0, names=['x','y'], lineterminator='\n')
    
# cte = data_result1.x
# yaw = data_result1.y

FileData3 = 'ref_linear1_UTM.csv'
with open (FileData3) as f:
    data_result1 = pd.read_table(f, sep=',', header=0, names=['x','y'], lineterminator='\n')
    
xNorth = data_result1.x
yEast = data_result1.y

FileData2 = 'ref_linear1_latlng.csv'
with open (FileData2) as f:
    data_result2 = pd.read_table(f, sep=',', header=0, names=['x','y'], lineterminator='\n')
    
lng = data_result2.x
lat = data_result2.y

#==== PLot ref 
# fig, axs = plt.subplots(2)
# fig.suptitle('Tracking Trajectory1 ')
# axs[0].plot(lat,'r')  #, y1,'r'
# axs[1].plot(lng,'g')   #, -y2,'r'

# # naming the x axis
# plt.xlabel('Time (second)')
# # naming the y axis
# axs[0].set( ylabel='Latitude (Degree)')
# plt.ylabel('Longitude (Degree)')

#=========Plot CTE / YAW ======================================

# fig, axs = plt.subplots(2)
# fig.suptitle('Error of Tracking Trajectory1 ')
# axs[0].plot(cte,'r')  #, y1,'r'
# axs[1].plot(yaw,'g')   #, -y2,'r'

# # naming the x axis
# plt.xlabel('Time (second)')
# # naming the y axis
# axs[0].set( ylabel='CTE (Meter)')
# plt.ylabel('Yaw (Degree)')



#==================================================

#==== Plot Real Track with Ref Track [2,2] four graph =====
fig, axs = plt.subplots(2, 2)
fig.suptitle('Tracking Trajectory1 ')
axs[0, 0].plot(lat ,label="line1")
axs[0, 0].plot(ref_lat ,label="line2")
axs[0, 0].set_title('Longitude (degree)')
axs[0, 1].plot(xNorth, 'tab:green')
axs[0, 1].set_title('UTM North (meter)')
axs[1, 0].plot(lng, 'tab:orange')
axs[1, 1].plot(yEast, 'tab:red')


for ax in axs.flat:
    ax.set(xlabel='Time(second)')
axs[1,0].set( ylabel='Latitude (Degree)')
axs[1,1].set( ylabel='UTM East (meter)')

#Hide x labels and tick labels for top plots and y ticks for right plots.

#========================================================
# giving a title to my graph
# plt.title('Linear1 ')
 
# show a legend on the plot
plt.legend()
 
# function to show the plot
plt.show()
