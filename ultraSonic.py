import time
import RPi.GPIO as GPIO
import board
import busio


try:

    #GPIO.setmode(GPIO.BOARD)

    trigP = 17
    echoP = 27

    GPIO.setup(trigP, GPIO.OUT)
    GPIO.setup(echoP, GPIO.IN)

    GPIO.output(trigP, GPIO.LOW)

    print ("Waiting for sensor to settle")

    time.sleep(2)
    print("calculating distance...")

    GPIO.output(trigP, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigP, GPIO.LOW)

    while GPIO.input(echoP)==0:
        pulse_start_time = time.time()
    while GPIO.input(echoP)==1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration*17150, 2)
    print("Distance: ", distance, "cm")

finally:
    GPIO.cleanup()
