import lcm
import sys

lc = lcm.LCM()

#sys.path.insert(0, '../')
sys.path.insert(0, '/home/odroid/realtime/LCM/')

from lcmt_midi import lcmt_midi

msg = lcmt_midi();

msg.timestamp = 0;

msg.event[0] = 176;
msg.event[1] = 0;
msg.event[2] = 65;

lc.publish("midi-out", msg.encode())
