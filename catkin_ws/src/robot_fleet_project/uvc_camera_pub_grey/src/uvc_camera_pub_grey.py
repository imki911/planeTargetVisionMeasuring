#!/usr/bin/env python
# -- coding: utf-8 --
#2019-07-010
#--Guofeng Shen gf@gfshen.cn---


# 为了降低图像传输的延迟，在机器人端将原始的uvc_camera image_raw 消息转换为灰度，体积缩小到1/3
# 经测试，480*320, 5Hz下，延迟在0.2-0.3s



#订阅的话题:riki16/image_raw
#发布的话题：riki16/camera/image_raw_grey

#处理内容：从riki16/image_raw话题中读取uvc_camera的图像数据，转换成灰度图像，写到riki16/camera/image_raw_grey中，header时间戳保持不变

#在树莓派上可能出现调用cv_bridge()时的错误，提示找不到opencvCore.so，谷歌搜索一两个小时结合自身经验积累即可解决
#libopencvCore.so文件的位置：
#/opt/ros/kinetic/lib/arm-linux-gnueabihf

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
IMAGE_WIDTH=1241
IMAGE_HEIGHT=376



def camera_callback(data):
    image2pub=data
    #将data中的数据转换成灰度 (mono8 类型)
    cv_img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img_gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
    #灰度数据放入Image消息中
    image2pub.data = np.array(img_gray).tostring()
    image2pub.encoding = 'mono8'
    #发布灰度消息，header保留原先uvc中的时间戳
    image_pubulish.publish(image2pub)
    print'grey image data ', image2pub.header.seq



'''
def publish_image(imgdata):
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height=IMAGE_HEIGHT
    image_temp.width=IMAGE_WIDTH
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tostring()
    #print(imgdata)
    #image_temp.is_bigendian=True
    image_temp.header=header
    image_temp.step=1241*3
    image_pubulish.publish(image_temp)
'''

    
if __name__=="__main__":

    print('This is node: uvc_camera_pub_grey')
    bridge = CvBridge()
    rospy.init_node("uvc_camera_pub_grey", anonymous=True)
    #订阅原始的uvc相机数据
    rospy.Subscriber("riki16/image_raw", Image,camera_callback,queue_size = 1)
    #发布处理后的灰度图像数据
    image_pubulish=rospy.Publisher('riki16/camera/image_raw_grey',Image,queue_size=1)

    rospy.spin()

