import lcm
import time
from drake import lcmt_iiwa_status
#import drake.lcmt_iiwa_status

prev_t=time.time()

def my_handler(channel, data):
    global prev_t
    cur_t = time.time()
    print("freq ",1.0/(cur_t-prev_t))
    prev_t = cur_t
    msg = lcmt_iiwa_status.decode(data)
    #print("Received message on channel \"%s\"" % channel)
    #print("   timestamp   = %s" % str(msg.utime))
    timestamp = msg.utime
    print("   forces   = %s" % str(msg.joint_torque_measured))


lc = lcm.LCM()
subscription = lc.subscribe("IIWA_STATUS", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
