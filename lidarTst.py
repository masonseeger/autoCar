from rplidar import RPLidar
import numpy as np
import time
import math
from statistics import *

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

def autoSteer(sectors):

    try:
        sectorMins = [0,0,0,0,0,0]
        sectorMean = [0,0,0,0,0,0]
        j = 0
        for i in sectors:
            sectorMins[j] = min(i)
            sectorMean[j] = mean(i)
            j+=1
        newAngle = (sectorMean[5] - sectorMean[1])
        if newAngle>4:
            newAngle = 4
        elif newAngle<-4:
            newAngle = -4
        #print(newAngle)
        if newAngle>=0 and sectorMins[5]>1:
            print("turning left")
            #steer.angle = aligned + (math.sqrt(newAngle))*20
        elif sectorMins[1]>1:
            print("turning rigth")
            #steer.angle = aligned -  math.sqrt(abs(newAngle))*20
        else:
            print("straight")
            #steer.angle = aligned
    except Exception as e:
        print(e)
        print("autoSteer")

def autoSpeed(sectors):
    try:

        sectorMins = [100,100,100,100,100,100]
        sectorMean = [100,100,100,100,100,100]
        j=0
        for i in sectors:
            sectorMins[j] = min(i)
            sectorMean[j] = mean(i)
            j+=1

        print(sectorMins)
        print(sectorMean)
        if sectorMins[0]<.5:
            print("sector 0 min < .5")
            #still()
            if sectorMins[5]>.5 and sectorMean[5]>sectorMean[1]:
                print("rightbackturn")
                #rightBackTurn()
            elif sectorMins[1]>.5:
                print("leftbackturn")
                #leftBackTurn()
        else:
            print("forward")
            #speed.throttle = .23
    except Exception as e:
        print(e)
        print('AutoSpeed')
        #still()


def lidarDistanceAndSector(scan):
    check = 0
    try:
        obstacles = []
        sectors = [[],[],[],[],[],[]]
        sector = 0
        for j in scan:
            dist = j[2]/1000
            deg = math.floor(j[1])
            obstacles.append([dist, deg])
            if deg>=330 or deg<=30:
                sector = 0
            else:
                sector = ((deg-30)//60)+1
            #print(str(deg) + '   ' + str(sector))
            sectors[sector].append(dist)
        #mean the inside of the sectors
        return(obstacles, sectors)

    except Exception as e:
        print(e)
        print("error in LidarDandS")
        #print(check)
        return([], [])

def main():
    circle = np.zeros(360)

    lidar = RPLidar("/dev/ttyUSB0")
    time.sleep(2)

    go = 1

    #try:
    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)

    iterator = lidar.iter_scans()
    obstacle = []
    sectors = [[],[],[],[],[],[]]
    try:
        while True:
            obstacles, sectors = lidarDistanceAndSector(next(iterator))
            if obstacles:
                autoSpeed(sectors)
                autoSteer(sectors)
            print('finished cycle')
    except:
        print("error occured in main")

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()



main()
