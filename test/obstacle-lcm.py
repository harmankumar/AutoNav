import lcm
from exlcm import example_t

def main():
	counter = int(raw_input())
	lc = lcm.LCM()  # Create object of lcm library
	msg = example_t()
	msg.currImage = counter
	lc.publish("PROCESSING_SEND", msg.encode())


if __name__ == '__main__':
    main()
