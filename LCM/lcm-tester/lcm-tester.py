import lcm
import sys

lc = lcm.LCM()

sys.path.insert(0, '../')
sys.path.insert(0, '../lcmt')
sys.path.insert(0, '../mav')

from pose_t import pose_t

msg = pose_t();

msg.utime = 42;

lc.publish("test", msg.encode())
