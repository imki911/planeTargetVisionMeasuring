#!/usr/bin/env python
# -- coding: utf-8 --

#2019年8月12日
#Ver1.0
#---Guofeng-----
#gf@gfshen.cn
#package: delay_test
#用於測試月延時的服務端,
#訂閱 發布帶header的msg

#related msg: DelayTestMsg.msg ( from delay_test.msg import DelayTestMsg)




import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from delay_test.msg import DelayTestMsg  #std_msgs/Header header float32 linearSpeed  float32 angularSpeed

import os
import numpy as np
import threading
import time

#--------------------
msg = """

"""

def periodicallyPubMsg(msg):
    global next_call
    msg.header.stamp=rospy.Time.now()
    pub.publish(msg)
    next_call = next_call+0.2
    if not rospy.is_shutdown():
        threading.Timer(next_call-time.time(), periodicallyPubMsg, [msg] ).start()

if __name__=="__main__":

    print('This is msg-delay test server..')
    #messageSubscribe() #订阅话题
    rospy.init_node('msgDelayTestServer', anonymous=True)
    pub = rospy.Publisher('DelayTestMsg', DelayTestMsg, queue_size = 1)
    msgToPub = DelayTestMsg()
    next_call = time.time()
    periodicallyPubMsg(msgToPub)
    
    rospy.spin()

