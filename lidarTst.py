from rplidar import RPLidar
import numpy as np
import time
import math

def lidarDistances(scan):
    try:
        obstacle = []
        print('Got %d measurements' % (len(scan)))
        for j in scan:
            dist = j[2]/1000
            deg = math.floor(j[1])
            #print("distance of " +str(dist)+"m at " + str(deg) + " degrees")
            if dist <1:
                print("adding obstacle")
                obstacle.append([dist, deg])

        print(obstacle)
        return(obstacle, 1)

    except:
        print("error in function")
        return([], 0)

def main():
    circle = np.zeros(360)

    lidar = RPLidar("/dev/ttyUSB1")
    time.sleep(1)

    go = 1

    #try:
    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)

    iterator = lidar.iter_scans()
    obstacle = []
    try:
        while go:
            obstacle, go = lidarDistances(next(iterator))
            print("made it past return")
            if obstacle:
                print("obstacle returned true")
                for i in obstacle:
                    print("Watch Out! Obstacle " + str(i[0]) + "m away at " + str(i[1]) + " degrees.")
            print("made it past if")
    except:
        print("error occured in main")

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()



main()
