import serial 
import time


port_m1 = '/dev/ttyACM11'
m1_s = serial.Serial(port_m1, 9600, timeout=5)
time.sleep(10)

m1_s.flush()
while True:
	msg = m1_s.readline(m1_s.inWaiting())
	print("Message from arduino: ")
	print(str(msg))
	time.sleep(5)