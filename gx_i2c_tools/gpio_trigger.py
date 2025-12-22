#!/usr/bin/python3

import RPi.GPIO as GPIO

import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(21,GPIO.OUT)

#while True:
for num in range(0,10):

    GPIO.output(21,GPIO.HIGH)

    time.sleep(0.001)

    GPIO.output(21,GPIO.LOW)

    time.sleep(1)
    
    print (num)


GPIO.cleanup()