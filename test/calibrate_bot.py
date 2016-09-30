import cgbot
from cgbot.commands import cmd
import time


def main():

    cmd.forward(speed=6000)
    # cmd.reverse(speed=6000)
    time.sleep(3)
    # cmd.stop()
    cmd.turn(0.8)
    time.sleep(2)
    cmd.stop()


if __name__ == '__main__':
    main()
