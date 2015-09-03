import lcm
import sys

lc = lcm.LCM()

#sys.path.insert(0, '../')
sys.path.insert(0, '/home/abarry/realtime/LCM/')

from lcmt import tvlqr_controller_action

msg = tvlqr_controller_action();

msg.timestamp = 0;
msg.trajectory_number = 0;

lc.publish("rc-trajectory-commands", msg.encode())
