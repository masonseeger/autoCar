import time
import numpy as np
import math
from statistics import *
from rplidar import RPLidar
import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685
from adafruit_motor import servo
import curses

sp = 0
aligned = 115
br = True
cPos = 0
#circle = np.zeros(360)
obstacle = []
freeSpace = [1,1,1,1,1,1]

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.nodelay(True)

try:
    lidar = RPLidar("/dev/ttyUSB0")
    time.sleep(1)

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)

    speed_channel = pca.channels[4]
    steer_channel = pca.channels[5]
    pca.frequency = 60

    steer = servo.Servo(steer_channel, min_pulse = 1000, max_pulse = 2000)
    speed = servo.ContinuousServo(speed_channel, min_pulse = 1000, max_pulse = 2000)

    steer.angle = aligned
    speed.throttle = 0
    sp = 0
except Exception as e:
    print("something went wrong initializing...")
    print(e)
    curses.endwin()
    br = False


def still():
    steer.angle = aligned
    speed.throttle = 0
    time.sleep(.01)
    sp = 0

#makes sure that I can travel in reverse
def setLow():
    speed.throttle = -.5
    time.sleep(.05)
    speed.throttle = 0
    time.sleep(.05)

def leftBackTurn():
    #stdscr.addstr(cPos,0,"reversing left turn")
    #cPos+=1
    steer.angle = 160
    setLow()
    speed.throttle = -.3
    time.sleep(1)

    steer.angle = aligned
    speed.throttle = .2
    time.sleep(.2)
    still()

def rightBackTurn():
    #stdscr.addstr(cPos,0,"reversing right turn")
    #cPos+=1
    steer.angle = 55
    setLow()
    speed.throttle = -.3
    time.sleep(1)

    steer.angle = aligned
    speed.throttle = .2
    time.sleep(.2)
    still()

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
            steer.angle = aligned + (math.sqrt(newAngle))*20
        elif sectorMins[1]>1:
            steer.angle = aligned -  math.sqrt(abs(newAngle))*20
        else:
            steer.angle = aligned
    except Exception as e:
        print(e)
        print("autoSteer")

def autoSpeed(sectors):
    try:
        sectorMins = [0,0,0,0,0,0]
        sectorMean = [0,0,0,0,0,0]
        j=0
        for i in sectors:
            sectorMins[j] = min(i)
            sectorMean[j] = mean(i)
        if sectorMins[0]<.5:
            still()
            if sectorMins[5]>.5 and sectorMean[5]>sectorMean[1]:
                rightBackTurn()
            elif sectorMins[1]>.5:
                leftBackTurn()
        else:
            speed.throttle = .23
    except Exception as e:
        print(e)
        print('AutoSpeed')
        still()


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

def autoDrive():
    #stdscr.addstr(cPos, 0, 'Now driving autonomously')
    #cPos+=1
    keyInterrupt = stdscr.getch()
    while keyInterrupt == -1:
        keyInterrupt = stdscr.getch()
        obstacles, sectors = lidarDistanceAndSector(next(iterator))
        if obstacles:
            autoSpeed(sectors)
            autoSteer(sectors)
    still()

def lidarObsDistances(scan):
    try:
        obstacle = []
        for j in scan:
            dist = j[2]/1000
            deg = math.floor(j[1])
            #print("distance of " +str(dist)+"m at " + str(deg) + " degrees")
            if dist <1:
                #print("adding obstacle")
                obstacle.append([dist, deg])
                freeSpace[deg//60] = 0

        #print(obstacle)
        return(obstacle, 1)

    except:
        print("error in function")
        return([], 0)

iterator = lidar.iter_scans()
try:
    #print("This is what you are looking for:::  "+str(speed.throttle))
    while br:
        if cPos>30:
            stdscr.clear()
            cPos = 0
        press = stdscr.getch()
        if press == 27: #ESC
            stdscr.addstr(cPos,0,"Exiting program... Stopping car...")
            cPos+=1
            still()
            break
        elif press == 97: #a
            stdscr.addstr(cPos,0,"Turning left, servo at " + str(steer.angle) + " degrees")
            cPos+=1
            if steer.angle<170:
                steer.angle += 5
        elif press == 100: #d
            stdscr.addstr(cPos,0,"Turning right, servo at " + str(steer.angle) + " degrees")
            cPos+=1
            if steer.angle>55:
                steer.angle -= 5
        elif press == 115: #s
            stdscr.addstr(cPos,0,"Decreasing velocity, throttle at " + str(speed.throttle))
            cPos+=1
            if speed.throttle >= -0.5:
                sp-=.05
                stdscr.addstr(cPos,0, "decrease in sp")
                cPos+=1
                speed.throttle -=.05
        elif press == 119: #w
            stdscr.addstr(cPos,0,"Increasing velocity, throttle at " + str(speed.throttle))
            cPos+=1
            if speed.throttle <= 0.5:
                sp+=.05
                speed.throttle +=.05
        elif press == 98: #b
            stdscr.addstr(cPos,0,"Braking")
            cPos+=1
            if speed.throttle>0:
                sp = 0
                setLow()
                speed.throttle = -.5
                time.sleep(.3)
                speed.throttle = 0.001
            elif speed.throttle<0:
                sp = 0
                speed.throttle = .5
                time.sleep(.2)
                speed.throttle = 0.001
        elif press == 108: #l(eft)
            stdscr.addstr(cPos,0,"reversing left turn")
            cPos+=1
            leftBackTurn()
            still()

        elif press == 114: #r(ight)
            stdscr.addstr(cPos,0,"reversing right turn")
            cPos+=1
            rightBackTurn()
            still()

        elif press == 32: #space
            stdscr.addstr(cPos,0,"Stopping motor and realigning wheels")
            setLow()
            cPos+=1
            still()

        elif press == 103: #g(o into auto)
            autoDrive()

        freeSpace = [1,1,1,1,1,1]
        obstacle, br = lidarObsDistances(next(iterator))
        #if obstacle:
            #j = 0
            #for i in freeSpace:
                #stdscr.addstr(cPos,0, "Watch Out! Obstacle in Sector " + str(i))
                #cPos+=1
                #if cPos>30:
                #    stdscr.clear()
                #    cPos = 0
                #j+=1
        #stdscr.addstr(cPos, 0 , "obs count: " + str(len(obstacle)) + ' ' + str(bool(not(1 in freeSpace))))
        #cPos +=1


        if not(1 in freeSpace):
            stdscr.nodelay(False)
            stdscr.addstr(cPos,0,"WARNING, ALL ROUTES BLOCKED, NEED USER ASSISTANCE!!!")
            stdscr.addstr(cPos+1, 0, "Press any key when area is clear")
            still()
            press = stdscr.getch()
            cPos +=2
            stdscr.nodelay(True)


except Exception as e:
    print(e)
    curses.endwin()
    speed.throttle = 0
    time.sleep(.01)
    still()
    pca.deinit()
    print("Something broke")

pca.deinit()
curses.endwin()
GPIO.cleanup()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()

if not(br):
    print('hi')

'''
    Could be helpful to make a function for braking based on how fast the car is already going.
    todo: clean up prints; remove sp; make more function calls; etc
'''
