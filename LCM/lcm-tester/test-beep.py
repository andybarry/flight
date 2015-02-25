import lcm
import sys

if len(sys.argv) != 2:
    print 'Must supply 1 or 0 for beep or no beep.  Example: python test-beep.py 1'
    exit(1)

beep_int = int(sys.argv[1])

if beep_int > 1 or beep_int < 0:
   print 'Error: must supply 0 or 1 as an argument'
   exit(1)

lc = lcm.LCM()

sys.path.insert(0, '../')

from lcmt_beep import lcmt_beep

msg = lcmt_beep();

msg.timestamp = 0;

msg.beep = beep_int;

lc.publish("beep", msg.encode())
