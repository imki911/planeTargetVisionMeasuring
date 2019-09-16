#!/usr/bin/env python
# -- coding: utf-8 --

#2019年8月12日
#Ver1.0
#---Guofeng-----
#gf@gfshen.cn
#package: delay_test
#用於測試月延時的[客戶端],
#接收帶時間戳的msg, 與本地ros時間對比,得到時延並記錄,保存爲.npz格式文件

#related msg: DelayTestMsg.msg ( from delay_test.msg import DelayTestMsg)




import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from delay_test.msg import DelayTestMsg  #std_msgs/Header header float32 linearSpeed  float32 angularSpeed

import os
import numpy as np

import time

#采样达到某一数量后结束
sampleTiems = 300

#--------------------
msg = """

"""
def callback(data):
    global delayValueStorage
    delay=float( (rospy.Time.now() - data.header.stamp).to_sec() )
    delayValueStorage.append(delay)
    print('msg delay: {}'.format(delay))
    if len(delayValueStorage) == sampleTiems: #测量的延迟采样数量达到预期, 主动关闭ros节点
        rospy.signal_shutdown('test sample enough,existing')
        


def shutdownHook():
    #在节点关闭之前,存储测试所得的延迟结果
    #TO DO!!!!!
    result = np.array(delayValueStorage)
    result.dump('delayTestResult.npz') #load: numpy.load('delayTestResult.npz')
    print('shutting down')


def periodicallyPubMsg(msg):
    global next_call
    msg.header.stamp=rospy.Time.now()
    pub.publish(msg)
    next_call = next_call+0.2
    threading.Timer(next_call-time.time(), periodicallyPubMsg, [msg] ).start()

if __name__=="__main__":

    print('This is msg-delay test server..')
    #messageSubscribe() #订阅话题
    rospy.init_node('msgDelayTestServer', anonymous=True)
    rospy.on_shutdown(shutdownHook)
    delayValueStorage=[]

    pub = rospy.Subscriber('/DelayTestMsg', DelayTestMsg, callback,tcp_nodelay=True)
       
    rospy.spin()

