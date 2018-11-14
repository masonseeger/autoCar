import time
import RPi.GPIO as GPIO
import board
import busio
from gpiozero import DistanceSensor
from multiprocessing import Process

def us():
    try:
        #GPIO.setmode(GPIO.BOARD)

        trigP = 17
        echoP = 27
        ultrasonic = DistanceSensor(echoP, trigP)
        while(True):
            print("Distance: ", ultrasonic.distance, "m")
            time.sleep(.01)
            if (ultrasonic.distance>.2):
                print("danger, dropoff detected")
                break
    except():
        print("good")

p = Process(target = us)
p.start()
print('hi')
p.join()
