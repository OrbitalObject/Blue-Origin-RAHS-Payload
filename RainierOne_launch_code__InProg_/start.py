import picamera
import RPi.GPIO as GPIO
import os
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.IN) #start pin
GPIO.setup(7, GPIO.IN)  #stop pin

cam = picamera.PiCamera()
camStarted = False

if cam:
	print('Camera connected.')
	while True:
		if (GPIO.input(11) && !camStarted): #start video
			cam.start_recording('RAHSFlightVideo.h264')
			camStarted = True
			print('Recording started.')
		elif (GPIO.input(7) && camStarted): #stop video
			cam.stop_recording()
			print('Recording stopped.')
else:
	print('No camera found, aborting.')