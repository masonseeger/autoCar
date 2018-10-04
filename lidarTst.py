from rplidar import RPLidar
import numpy as np
import time
import math

circle = np.zeros(360)

lidar = RPLidar("/dev/ttyUSB0")
time.sleep(1)

#try:
info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

try:
    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurements' % (i,len(scan)))
        for j in scan:
            dist = j[2]/1000
            deg = math.floor(j[1])
            print("distance of " +str(dist)+"m at " + str(deg) + " degrees")
except:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    exit()


lidar.stop()
lidar.stop_motor()
lidar.disconnect()
