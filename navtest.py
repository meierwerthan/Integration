
from marvelmind import MarvelmindHedge
from time import sleep
import sys

global pos


def position(HEDGE):
	try:
      		# print (hedge.position()) # get last position and print
      		pos =  HEDGE.position()
		print("Addr: " + str(pos[0]))
		print("X: " + str(pos[1]))
		print("Y: " + str(pos[2]))
		print("Time: " + str(pos[5]))
		return pos
	except KeyboardInterrupt:
		hedge.stop()

