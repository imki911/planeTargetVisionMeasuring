#!/usr/bin/env python
# -- coding: utf-8 --

#2019年8月12日
#Ver1.0
#---Guofeng-----
#gf@gfshen.cn

#延迟结果绘图.

#读入numpy array文件,将结果绘图





import matplotlib.pyplot as plt
import numpy as np


#--------------------
msg = """

"""
def callback(data):
    global delayValueStorage
    delay=float( (rospy.Time.now() - data.header.stamp).to_sec() )
    delayValueStorage.append(delay)
    print('msg delay: {}'.format(delay))


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

    
    
    result1 = np.load('delayTestResult.npz')
    #result1 = np.load('1kongzai.npz')
    result1 = result1*1000 
    #绘图:https://matplotlib.org/3.1.1/gallery/lines_bars_and_markers/nan_test.html#sphx-glr-gallery-lines-bars-and-markers-nan-test-py
    len=result1.shape[0]
    axisX=np.arange(len)*0.2
    plt.subplot(1,1,1)
    plt.plot(axisX, result1)
    plt.xlabel('time, s ')
    plt.ylabel('delay, ms')
    plt.show()

