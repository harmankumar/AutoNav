import lcm
from exlcm import example_t


def my_handler(channel, data):
    msg = example_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   currImage   = %s" % str(msg.currImage))
    print("   distance   = %s" % str(msg.distance))
    print("   angle   = %s" % str(msg.angle))

lc = lcm.LCM()
subscription = lc.subscribe("EXAMPLE", my_handler)

try:
    while True:
		lc.handle()
except KeyboardInterrupt:
    pass
