#!/usr/bin/python3

import RPi.GPIO as GPIO

import time

GPIO.setmode(GPIO.BCM)


GPIO.setup(21,GPIO.OUT,initial=GPIO.LOW)


print ("ready? rising coming in 3 seconds")

time.sleep(3)

GPIO.output(21,GPIO.HIGH)

time.sleep(0.01)

GPIO.output(21,GPIO.LOW)



GPIO.cleanup()
