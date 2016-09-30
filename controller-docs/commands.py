from ..logs import Log
log = Log()
from ..redisdb import rdb


class Default():

    CH_CMD = "CG_CMD"
    CH_FLW = "CG_FOL"
    CH_ENC = "CG_ENL"
    CH_GPS = "CG_GPS"

    STOP_DELAY = 0
    FLW_DELAY = 0
    GOTO_DELAY = 0

    FWD_TIME = 0
    REV_TIME = 0
    MOVE_TIME = 0

    ACCEL = 12800.0
    SPEED = 9600.0
    TURN = 0.0
    TURN_L = 90.0
    TURN_R = 90.0
    HEAD_FLIP = 1

    SPEED_UP_STEP = 200.0
    SPEED_DOWN_STEP = 200.0

    LAT = None
    LON = None


class Command(object):

    def __init__(self):
        self._CH_LIVE = "CGBOT_LIVE"

    def live(self, status=None):
        if status is None:
            return bool(rdb.get(self._CH_LIVE))
        if bool(status):
            rdb[self._CH_LIVE] = "True"
        else:
            rdb[self._CH_LIVE] = ""
        return bool(status)


    def stop(self, accel=Default.ACCEL, delay=Default.STOP_DELAY):
        """
        instructs to stop after `delay` time
        """
        cmd = "stop({}, {}, {})".format(accel, accel, delay)
        self.live(False)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def move(self, acc_l=Default.ACCEL, spd_l=Default.SPEED, acc_r=Default.ACCEL, spd_r=Default.SPEED, duration=Default.MOVE_TIME):
        """
        instructs to move forward for `duration` time
        a value of zero means perpetual motion
        """

        cmd = "move({}, {}, {}, {}, {})".format(acc_l, spd_l, acc_r, spd_r, duration)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def forward(self, accel=Default.ACCEL, speed=Default.SPEED, duration=Default.FWD_TIME):
        """
        instructs to move forward for `duration` time
        a value of zero means perpetual motion
        """
        cmd = "forward({}, {}, {})".format(accel, speed, duration)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def reverse(self, accel=Default.ACCEL, speed=Default.SPEED, duration=Default.REV_TIME):
        """
        instructs to move forward for `duration` time
        a value of zero means perpetual motion
        """

        cmd = "reverse({}, {}, {})".format(accel, speed, duration)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))

    def turn(self, amount=Default.TURN):
        """
        instructs to turn left/right with `amount` intensity (synonymous to torque)
        0 means no turn
        +1.0 means turn at full capacity to the right
        -1.0 means turn at full capacity to the left
        """
        amt = amount if abs(amount) <= 1 else amount/abs(amount)
        cmd = "turn({})".format(amt)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def turn_left(self, angle=Default.TURN_L):
        """
        instructs to turn a predefined `angle` to the left
        """
        cmd = "turn_left({})".format(angle)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def turn_right(self, angle=Default.TURN_R):
        """
        instructs to turn a predefined `angle` to the right
        """
        cmd = "turn_right({})".format(angle)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def switch_head(self, times=Default.HEAD_FLIP):
        """
        instructs to switch the head `times` times
        """
        cmd = "switch_head({})".format(times)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def speed_up(self, amount=Default.SPEED_UP_STEP):
        """
        instructs to speed up a certain `amount`
        """
        cmd = "speed_up({})".format(amount)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def speed_down(self, amount=Default.SPEED_DOWN_STEP):
        """
        instructs to speed down a certain `amount`
        """
        cmd = "speed_down({})".format(amount)
        rdb.publish(Default.CH_CMD, cmd)
        log.debug("Instruction - {}".format(cmd))


    def follow(self, delay=Default.FLW_DELAY):
        """
        instructs to follow traced path after `delay`
        """
        cmd = "follow({})".format(delay)
        rdb.publish(Default.CH_FLW, cmd)
        log.debug("Instruction - {}".format(cmd))


    def goto_point(self, lat=Default.LAT, lon=Default.LON, delay=Default.GOTO_DELAY):
        """
        Instructs to move to gps coordinate
        """
        if lat and lon and (abs(lat) <= 90) and (abs(lon) <= 180):
            cmd = "goto_point({}, {}, {})".format(lat, lon, delay)
            rdb.publish(Default.CH_GPS, cmd)
            log.debug("Instruction - {}".format(cmd))
        else:
            log.error("Instruction error - invalid coordinates")
