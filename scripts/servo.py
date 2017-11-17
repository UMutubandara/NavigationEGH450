#!/usr/bin/env python
#!/usr/bin/env python

# license removed for brevity
import rospy

from mavros_msgs.msg import OverrideRCIn
import serial
import time
def talker():
     pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
     rospy.init_node('servo_ctrl', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     msg_out = OverrideRCIn()
     msg_out.channels[0] = msg_out.CHAN_NOCHANGE
     msg_out.channels[1] = msg_out.CHAN_NOCHANGE
     msg_out.channels[2] = msg_out.CHAN_NOCHANGE
     msg_out.channels[3] = msg_out.CHAN_NOCHANGE
     msg_out.channels[4] = msg_out.CHAN_NOCHANGE
     msg_out.channels[5] = msg_out.CHAN_NOCHANGE
     msg_out.channels[6] = msg_out.CHAN_NOCHANGE

     while not rospy.is_shutdown():
         msg_out.channels[7] = 2000
         rospy.loginfo(msg_out)
         pub.publish(msg_out)
         rate.sleep()
if __name__ == '__main__':


    try:
        talker()
    except rospy.ROSInterruptException:
        pass
