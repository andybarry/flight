import lcm
import sys

lc = lcm.LCM()

#sys.path.insert(0, '../')
sys.path.insert(0, '/home/abarry/realtime/LCM/')

from lcmt import timestamp

msg = timestamp();

msg.timestamp = 0;

lc.publish("state-machine-go-autonomous", msg.encode())
