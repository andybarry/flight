import lcm
import sys

lc = lcm.LCM()

sys.path.insert(0, '../')
sys.path.insert(0, '../lcmt')

from rc_switch_action import rc_switch_action

msg = rc_switch_action();

msg.timestamp = 0;

msg.action = 1;

lc.publish("rc-switch-action", msg.encode())
