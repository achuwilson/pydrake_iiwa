import select
import lcm
import time
from drake import lcmt_iiwa_status, lcmt_iiwa_command
#import drake.lcmt_iiwa_status

prev_t=time.time()
msg_count = 0
intPosAcquired  = False
initPos=None

lc = lcm.LCM()

def my_handler(channel, data):
    global msg_count
    global intPosAcquired
    global prev_t
    msg_count = msg_count+1
    
    cur_t = time.time()
    print("freq ",1.0/(cur_t-prev_t))
    prev_t = cur_t
    msg = lcmt_iiwa_status.decode(data)
    if(msg_count==5):
        lc.unsubscribe(subscription)
        initPos= msg.joint_position_measured
        intPosAcquired = True
        print("Acquired Inital Pos", initPos)
        msg_cmd = lcmt_iiwa_command()
        msg_cmd.utime = int(time.time()*1000000)
        msg_cmd.num_joints = 7
        #for jn in range(6):
        #    msg_cmd.joint_position[jn] = initPos[jn]
        msg_cmd.joint_position = (initPos[0],initPos[1],initPos[2],initPos[3] ,initPos[4],initPos[5],1.0)

        msg_cmd.num_torques = 0
        lc.publish("IIWA_COMMAND", msg_cmd.encode())

        #time.sleep(5)


    print("Received message on channel \"%s\"" % channel)
    #print("   timestamp   = %s" % str(msg.utime))
    print("   forces   = %s" % str(msg.joint_position_measured[6]))



subscription = lc.subscribe("IIWA_STATUS", my_handler)




try:
    while True:
        lc.handle()

except KeyboardInterrupt:
    pass


print("ended")
#lc.unsubscribe(subscription)
