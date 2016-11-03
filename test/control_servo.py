import serial

ser = serial.Serial('/dev/ttyACM0', 115200)

def write_servo(angle):
	ser.write(b'W')
	angle = str(angle)
	ser.write(angle)


def read_servo():
	ser.write(b'R')
	print ser.readline()
