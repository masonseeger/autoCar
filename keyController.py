import time
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

try:
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

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
except:
    print("something went wrong initializing...")
    curses.endwin()
    br = False


def still():
    steer.angle = aligned
    speed.throttle = 0
    time.sleep(.01)
    sp = 0

#makes sure that I can travel in reverse
def setLow():
    speed.throttle= -.5
    speed.throttle = 0

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
            if steer.angle>70:
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
                speed.throttle = -.5
                time.sleep(.05)
                speed.throttle = 0
                time.sleep(.05)
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
            steer.angle = 160
            speed.throttle = -.3
            time.sleep(1.6)

            steer.angle = aligned
            speed.throttle = .2
            time.sleep(.2)

            still()
        elif press == 114: #r(ight)
            stdscr.addstr(cPos,0,"reversing right turn")
            cPos+=1
            steer.angle = 70
            speed.throttle = -.3
            time.sleep(1.5)

            steer.angle = aligned
            speed.throttle = .2
            time.sleep(.2)

            still()


        elif press == 32: #space
            stdscr.addstr(cPos,0,"Stopping motor and realigning wheels")
            cPos+=1
            still()
except:
    curses.endwin()
    speed.throttle = 0
    time.sleep(.01)
    still()
    pca.deinit()

pca.deinit()
curses.endwin()
GPIO.cleanup()

'''
    Could be helpful to make a function for braking based on how fast the car is already going.

    Reverse problems may be a part of safety features of the esc...
you may need to go into reverse, neutral, then reverse again in order to get things going...
not sure but it could be helpful

    Make a sort of set low function to automatically be put into reverse
if not already. It is a safety feature of the ESC

    Get rid of the stupid sp variable, it doesn't do its job very well.


    todo: clean up prints; remove sp; make more function calls; etc
'''
