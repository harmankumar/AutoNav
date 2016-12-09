import socket, traceback
import netifaces as ni
#ni.ifaddresses('wlan0')
host = ni.ifaddresses('wlp6s0')[2][0]['addr']
#print ip  # should print "192.168.100.37
# Note that the IP Address and Port in this script and the script on the Mobile Phone should match.
#host = "10.192.46.131"   #IP Address of this Machine
port = 3400
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))
while 1:
    try:
        message, address = s.recvfrom(4096)
        message_components = message.split()
        print "Timestamp:", message_components[0]
        print "Orientation(radians):", message_components[1]
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
