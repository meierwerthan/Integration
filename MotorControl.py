import time
import RPi.GPIO as GPIO
import navtest
from marvelmind import MarvelmindHedge
import numpy

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=4, debug=False)
hedge.start() # start thread

DRIVE_1 = 37
DRIVE_2 = 36
DRIVE_3 = 13
DRIVE_4 = 15

pwm_pin1 = 12
pwm_pin2 = 16
pwm_pin3 = 18
pwm_pin4 = 22

GPIO.setup(DRIVE_1, GPIO.OUT)
GPIO.setup(DRIVE_2, GPIO.OUT)
GPIO.setup(DRIVE_3, GPIO.OUT)
GPIO.setup(DRIVE_4, GPIO.OUT)

GPIO.setup(pwm_pin1, GPIO.OUT)
GPIO.setup(pwm_pin2, GPIO.OUT)
GPIO.setup(pwm_pin3, GPIO.OUT)
GPIO.setup(pwm_pin4, GPIO.OUT)


pwm1 = GPIO.PWM(pwm_pin1, 100)
pwm2 = GPIO.PWM(pwm_pin2, 100)
pwm3 = GPIO.PWM(pwm_pin3, 100)
pwm4 = GPIO.PWM(pwm_pin4, 100)


pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)


class Motor():
	def __init__(self, motor):
		self.m = motor
	def direction(self, direction):
		self.d = direction
		if(self.m == "ONE"):
			if(self.d == "forward"):
				GPIO.output(DRIVE_1, GPIO.HIGH)
			elif(self.d == "reverse"):
				GPIO.output(DRIVE_1, GPIO.LOW)
		elif(self.m == "TWO"):
			if(self.d == "forward"):
				GPIO.output(DRIVE_2, GPIO.HIGH)
			elif(self.d == "reverse"):
				GPIO.output(DRIVE_2, GPIO.LOW)
		elif(self.m == "THREE"):
			if(self.d == "forward"):
				GPIO.output(DRIVE_3, GPIO.LOW)
			elif(self.d == "reverse"):
				GPIO.output(DRIVE_3, GPIO.HIGH)
		elif(self.m == "FOUR"):
			if(self.d == "forward"):
				GPIO.output(DRIVE_4, GPIO.LOW)
			elif(self.d == "reverse"):
				GPIO.output(DRIVE_4, GPIO.HIGH)
m1 = Motor("ONE")
m2 = Motor("TWO")
m3 = Motor("THREE")
m4 = Motor("FOUR")

def direction(direction):
	if(direction == "forward"):
		m1.direction("forward")
		m3.direction("forward")
		print("##### Direction #####")
		print("Forward")
		print("##### Motor Configuration (1,3)#####")
		print("HIGH Motor1: ", sc1.x)
		print("LOW Motor3: ", sc3.x)
	elif(direction == "reverse"):
		m1.direction("reverse")
		m3.direction("reverse")
		print("##### Direction #####")
		print("Reverse")
		print("##### Motor Configuration (1,3)#####")
		print("LOW Motor1: ", sc1.x)
		print("HIGH Motor3: ", sc3.x)
	elif(direction == "left"):
		m2.direction("forward")
		m4.direction("forward")
		print("##### Direction #####")
		print("Left")
		print("##### Motor Configuration (2,4)#####")
		print("HIGH Motor2: ", sc2.x)
		print("LOW Motor4: ", sc4.x)
	elif(direction == "right"):
		m2.direction("reverse")
		m4.direction("reverse")
		print("##### Direction #####")
		print("Right")
		print("##### Motor Configuration (2,4)#####")
		print("LOW Motor2: ", sc2.x)
		print("HIGH Motor4: ", sc4.x)

global newEvent1
global newEvent2
newEvent1 = False
newEvent2 = False

class SpeedCount():
	def __init__(self):
		self.x = 0
		self.o = 0
	def get(self):
		return self.x
	def reValue(self, newValue, speedfactor):
		if(0 <= self.x <= 90 and self.x < abs(newValue*100)*speedfactor ):
			self.x = self.x + abs(10*newValue)
		elif(10 <= self.x <= 100 and self.x > abs(newValue*100)*speedfactor):
			self.x = self.x - abs(10*newValue)
		elif(self.x < 0):
			print('Minimum Speed Reached')
		elif(self.x > 100):
			print('Maximum Speed Reached')

sc1 = SpeedCount()
sc2 = SpeedCount()
sc3 = SpeedCount()
sc4 = SpeedCount()

def MotorOff():
	GPIO.output(DRIVE_1, GPIO.LOW)
	GPIO.output(DRIVE_2, GPIO.LOW)
	GPIO.output(DRIVE_3, GPIO.LOW)
	GPIO.output(DRIVE_4, GPIO.LOW)


axisUpDownInverted = True ##Set true if u/d swapped
axisLeftRightInverted = False
pause = 0.1


global moveUp
global moveDown
global moveDone
global moveLeft
global moveRight

moveUp = False
moveDown = False
moveDone = False
moveLeft = False
moveRight = False

##Function to handle the Events
def Handler(leftRight, upDown, speedfactor):
	#print(upDown)
	#print(leftRight)
	global newEvent1
	global newEvent2
	global moveUp
	global moveDown
	global moveDone
	global moveLeft
	global moveRight

	if axisUpDownInverted:
		upDown = -upDown
	if axisLeftRightInverted:
		leftRight = -leftRight
	if upDown < -0.1:
		newEvent1 = True
		moveUp = True
		moveDown = False
		sc1.reValue(upDown, speedfactor)
		sc3.reValue(upDown, speedfactor)
		pwm1.ChangeDutyCycle(sc1.get())
		pwm3.ChangeDutyCycle(sc3.get())
	elif upDown > 0.1:
		newEvent1 = True
		moveUp = False
		moveDown = True
		sc1.reValue(upDown, speedfactor)
		sc3.reValue(upDown, speedfactor)
		pwm1.ChangeDutyCycle(sc1.get())
		pwm3.ChangeDutyCycle(sc3.get())
	else:
		if(-0.1 <= upDown <= 0.1):
			sc1.reValue(0, speedfactor)
			sc3.reValue(0, speedfactor)
			pwm1.ChangeDutyCycle(sc1.get())
			pwm3.ChangeDutyCycle(sc3.get())
		moveUp = False
		moveDown = False
		MotorOff()
	if leftRight < -0.1:
		newEvent2 = True
		moveLeft = True
		moveRight = False
		sc2.reValue(leftRight, speedfactor)
		sc4.reValue(leftRight, speedfactor)
		pwm2.ChangeDutyCycle(sc2.get())
		pwm4.ChangeDutyCycle(sc4.get())
	elif leftRight > 0.1:
		newEvent2 = True
		moveLeft = False
		moveRight = True
		sc2.reValue(leftRight, speedfactor)
		sc4.reValue(leftRight, speedfactor)
		pwm2.ChangeDutyCycle(sc2.get())
		pwm4.ChangeDutyCycle(sc4.get())
	else:
		if(-0.1 <= leftRight <= 0.1):
			sc2.reValue(0, speedfactor)
			sc4.reValue(0, speedfactor)
			pwm2.ChangeDutyCycle(sc2.get())
			pwm4.ChangeDutyCycle(sc4.get())
		moveLeft = False
		moveRight = False
		#newEvent2 = False
		MotorOff()

def main(X, Y, Z):
	global newEvent1
	global newEvent2
	newEvent1 = False
	newEvent2 = False
	c = navtest.position(hedge)
	X_diff = X  - c[1]
	Y_diff = Y - c[2]
	m1 = numpy.sqrt((X_diff*X_diff)+(Y_diff*Y_diff))
	try:
		degrees = numpy.arctan(X_diff/Y_diff) * 180 / numpy.pi
	except ZeroDivisionError:
		degrees = 0


	print('Press Ctrl+c to quit')
	while True:
		try:
			#c = navtest.position(hedge)
			X_diff = X  - c[1]
			Y_diff = Y - c[2]
			m2 = numpy.sqrt((X_diff*X_diff)+(Y_diff*Y_diff))
			try:
				degreesNew = numpy.arctan(X_diff/Y_diff) * 180 / numpy.pi
			except:
				degreesNew = 0
			if(X_diff == 0):
				newX = 0
			else:
				newX = X_diff / m2
			if(Y_diff == 0):
				newY = 0
			else:
				newY = Y_diff / m2
			speedfactor = m2/m1
			if(X == c[1] and Y == c[2]):
				print("here")
				break
			print("##### Goal Position #####")
			print("X1: " + str(X))
			print("Y1: " + str(Y))
			print("Initial Angle: " + str(degrees))
			print("#####Current Position#####")
			print("X0: " + str(c[1]))
			print("Y0: " + str(c[2]))
			print("Current Angle: " + str(degreesNew))
			error = degrees - degreesNew
			print("ERROR: " + str(error))
			Handler(newX, newY, speedfactor)
			if newEvent1:
				newEvent1 = False
				if moveUp:
					direction("forward")
				elif moveDown:
					direction("reverse")
				else:
					MotorOff()
			if newEvent2:
				newEvent2 = False
				if moveLeft:
					direction("left")
				elif moveRight:
					direction("right")
				else:
					MotorOff()

			time.sleep(1)
			if(Z == 1):
				c[2] = c[2]+1
			elif(Z == 2):
				c[1] = c[1] + 1
				c[2] = c[2] + 1
			elif(Z == 3):
				c[2] = c[2] - 1
			elif(Z == 4):
				c[1] = c[1] - 1
		except KeyboardInterrupt:
			print("interrupted")
			MotorOff()
			pwm1.stop()
			pwm2.stop()
			pwm3.stop()
			pwm4.stop()
			MotorOff()
			GPIO.cleanup()

#main(0, 10, 1)
main(10, 10, 2)
main(10, 0, 3)
main(0, 0, 4)
MotorOff()
pwm1.stop()
pwm2.stop()
pwm3.stop()
pwm4.stop()
MotorOff()
GPIO.cleanup()
