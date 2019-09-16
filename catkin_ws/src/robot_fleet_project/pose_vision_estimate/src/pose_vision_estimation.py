#!/usr/bin/env python
# -- coding: utf-8 --

#2019年6月1日
#Ver1.0
#---Guofeng-----
#gf@gfshen.cn
#package: pose_vision_estimation
#根据摄像机采集的图像数据，识别前车aruco标志板的相对位姿
#describe: riki16/image_raw
#publish: PoseEstimated.msg

#依赖环境运行环境 numpy, cv2，cv2.aruco


import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from pose_vision_estimate.msg import PoseEstimated #customed msg,  float32 x, float32 y, float32 theta


#-----图像处理相关--------------
import cv2
import cv2.aruco as aruco
import os
import numpy as np
#import paramiko
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#--------------------
msg = """

"""


def rotationMatrixToEulerAngles(R) :
    '''
    From rotation matrix to Euler Angles(x-y-z)
    In this application use y(roll)
    https://blog.csdn.net/liuxiaoheng1992/article/details/85273765
    '''
    #assert(isRotationMatrix(R))
     
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])


def camera_callback(data):
    print "Processing frame | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec()
    mtx=np.array([[532.061593, 0, 235.433934], [0, 532.132096, 202.152328], [0.0, 0.0, 1.0]])
    dist=np.array([0.119386, -0.402118, -0.010684, 0.001014, 0])
    global redLower1
    global redUpper1
    global redLower2 
    global redUpper2
    global leftTurn
    global rightTurn
    global stopSign
    try:
        #cv_img = bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_img = bridge.imgmsg_to_cv2(data, 'mono8')
    except CvBridgeError as e:
        print(e)

    ##cv2.imshow("Raw Image", cv_img)
    (rows, cols) = cv_img.shape
    
    #img_gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
    img_gray =  cv_img

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    

    if np.all(ids != None):
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], 0.096, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
        rmat=cv2.Rodrigues(rvec)[0]
        pose=(tvec[0,0,0],tvec[0,0,2],-rotationMatrixToEulerAngles(rmat)[1])
        print(pose)
        (poseMsg.x, poseMsg.y, poseMsg.theta) = pose
        
        poseMsg.delay = float( (rospy.Time.now() - data.header.stamp).to_sec() )
        pub.publish(poseMsg)
        #(rvec-tvec).any() # get rid of that nasty numpy value array error
        
        ###### DRAW axis on the marker ####

        #aruco.drawAxis(cv_img, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
        #aruco.drawDetectedMarkers(cv_img, corners) #Draw A square around the markers
        
        ###### DRAW ID #####

        #cv2.putText(cv_img, "Rob:" + str(ids), (int((corners[0][0][0][0]))-30, int((corners[0][0][0][1]))-30), font, 0.5, (0,255,0),1,cv2.LINE_AA)
        #cv2.putText(cv_img, "Dis:" +str((tvec[0,0,2])) , (int((corners[0][0][3][0])), int((corners[0][0][3][1]))-20), font, 0.5, (0,255,0),1,cv2.LINE_AA)
       
    # Display the processed image
    
    #cv2.imshow('cv_img',cv_img)
    #cv2.waitKey(3)
    
    ###functuion end######
   



    
''' 
    #twist = Twist()
    twist.linear.x = data.linear.x; twist.linear.y = data.linear.y; twist.linear.z = data.linear.z;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = data.angular.z
    pub.publish(twist)

''' 
def messageSubscribe():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #init  node
    rospy.init_node('pose_vision_estimate', anonymous=True)
    #本车的摄像机话题
    rospy.Subscriber("/riki16/camera/image_raw_grey", Image,camera_callback,queue_size = 1,buff_size=2**24)
    # spin() simply keeps python from exiting until this node is stopped
    

    
if __name__=="__main__":

    #initMsgQueue(msgQueue) #初始化用于缓存cmd_cvl消息的队列
    bridge = CvBridge()
    print('This is Rob_rear1.')
    messageSubscribe() #订阅图像话题
    pub = rospy.Publisher('pose_estimate', PoseEstimated, queue_size = 10)
    poseMsg = PoseEstimated()

    
    rospy.spin()

