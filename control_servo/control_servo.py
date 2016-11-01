import serial
import time
ser = serial.Serial('/dev/ttyACM0', 115200)
def write_servo(angle):
	ser.write(b'W')
	angle = str(angle)
	ser.write(angle)


def read_servo():
	ser.write(b'R')
	print ser.readline()


while True:
	option = input("Enter 1 for writing and 2 for reading\n")
	if(option==1):
		angle = input("Enter the angle where you want to move\n")
		write_servo(angle)
	elif(option==2):
		read_servo()
