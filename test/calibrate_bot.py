import cgbot
from cgbot.commands import cmd
import time


def main():
    print "Start"
    # cmd.forward(speed=6000, duration = 5)
    cmd.forward(speed=5000)
    time.sleep(10)
    print "Done"
    cmd.stop()
    # cmd.stop()
    # cmd.turn(0.8)
    # time.sleep(2)
    # cmd.stop()


if __name__ == '__main__':
    main()
