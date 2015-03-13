import lcm
import sys
import time

lc = lcm.LCM()

sys.path.insert(0, '../')

from lcmt_debug import lcmt_debug

msg = lcmt_debug();

msg.utime = int(time.time() * 1000000);

msg.debug = "testing debug";

lc.publish("debug", msg.encode())
