import time
import numpy
import navtest
import MotorControl
#from MotorControl import Handler
from marvelmind import MarvelmindHedge
from time import sleep
import sys


global hedge
global drive
drive = True
hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=4, debug=False)
hedge.start() # start thread
MotorControl.main()
def main(X, Y, drive):
	while drive:
		c = navtest.position(hedge)
		X_diff = X  - c[1]
		Y_diff = Y - c[2]
		m = numpy.sqrt((X_diff*X_diff)+(Y_diff*Y_diff))
		newX = X_diff / m
		newY = Y_diff / m
		if(X == c[1] and Y == c[2]):
			print("here")
			drive = False
			break

		print("X1: " + str(X))
		print("Y1: " + str(Y))
		print("X2: " + str(newX))
		print("Y2: " + str(newY))
		time.sleep(1)
		#MotorControl.Go(newX, newY, drive)

main(1, 2, drive
MotorControl.Done()