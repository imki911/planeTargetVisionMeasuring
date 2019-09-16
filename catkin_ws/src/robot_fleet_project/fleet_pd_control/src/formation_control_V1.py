#!/usr/bin/env python
# -- coding: utf-8 --
#2019-06-01
#--Guofeng Shen gf@gfshen.cn---
#雾计算模式下的机器人编队控制，
# 领航者释放速度控制信息，在该程序中缓存为队列。
# 结合视觉估计模块所得的跟随者相对位姿，使用PD控制修正跟随者的控制队列信息
# 以领航者的/cmd_vel为驱动脉冲，延时释放修正后的速度信息给跟随者，达到轨迹跟踪的效果。 

#订阅话题： 1. 视觉估计节点的相对位姿信息 pose_estimate 类型：PoseEstimated， [x, y, theta]
#         2. 前车的cmd_vel话题 ，类型 Twist
#发布话题： 跟随者命名空间下的 cmd_vel


#2019-07-12
#1. 加入了圖像延遲的補償算法
#2. 改進CmdMsgList類. 




import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#import msg
from geometry_msgs.msg import Twist
from pose_vision_estimate.msg import PoseEstimated #customed msg,  float32 x, float32 y, float32 theta
from std_msgs.msg import String

import numpy as np

import speedList2LocationV1_2 as speed2list


from cv_bridge import CvBridge, CvBridgeError

# ---------全局变量------

#视觉修正的cmd_cvl信号

forwardLeft  =[0.2088,0,0,0,0,0.3] #左转前进修正信号
forwardRight =[0.2088,0,0,0,0,-0.3] #右转前进修正信号
stop=[0,0,0,0,0,0] #刹车
#视觉识别相关变量和参数


redLower1 = np.array([170,100,80])
redUpper1 = np.array([180,255,255])
redLower2 = np.array([0,100,80])
redUpper2 = np.array([10,255,255])
#--------------------
msg = """
Hello World, hola kevin
"""


#存储速度控制消息的类
class CmdMsgList():
    def __init__(self,initList = None):

        #leader's raw cmd
        if initList == None:
            # self.leaderCmdList  最新的在後面
            self.leaderCmdList = np.array([[0.0,0]]*10)
            self.leaderCmdList = np.append(self.leaderCmdList,[[0.3,0]]*15,axis=0) # 0.9 meter
            self.leaderCmdList = np.append(self.leaderCmdList,[[0,0]]*5,axis=0) # 補上一些零，用於應對圖像延遲補償時算法 對啓動時產生的影響
        else:
            self.leaderCmdList = initList
            self.leaderCmdList = np.append(self.leaderCmdList,[[0,0]]*5,axis=0) # 同上，補一些零
        
        self.leaderCmdListHistory = np.array([[0.0,0]]*15) #15個控制數據的緩存,從後面添加。
        # 用於pd矯正的數據：
        # [leaderCmdListHistory] | [leaderCmdList]  <- 新的數據
        #
        
        #深拷貝一份 控制數據和歷史數據，用於存儲修正後的數據
        self.modifiedCmd = self.leaderCmdList.copy()
        self.modifiedCmdHistory = self.leaderCmdListHistory.copy()

    
        
    def push(self, singleCmd):
        #push a cmd to the end of leaderCmdList
        self.leaderCmdList = np.append(self.leaderCmdList,singleCmd ,axis=0)
        self.modifiedCmd = np.append(self.modifiedCmd,singleCmd ,axis=0)
        
        return

    def get(self):
        #get a cmd from the top of self.modifiedCmd list, and move it to each historyList
        popCmdModified = self.modifiedCmd[0]
        popCmdRaw = self.leaderCmdList[0]
        self.modifiedCmd = np.delete(self.modifiedCmd,0,0) 
        self.leaderCmdList = np.delete(self.leaderCmdList,0,0)
        assert(self.modifiedCmd.shape==self.leaderCmdList.shape) #always make sure shape of two list equals. 
        
        #將彈出的數據放到歷史數據中
        self.leaderCmdListHistory= np.append( self.leaderCmdListHistory[:-1],popCmdModified.reshape(1,2) , axis=0)
        self.modifiedCmdHistory= np.append( self.modifiedCmdHistory[:-1],popCmdRaw.reshape(1,2) , axis=0)
        return popCmdModified
    
    def fetchCmdListWithDelay(self,delayTime=0, delta=0.2):
        '''
        用於在估計當前領航者位置，考慮圖像傳輸時延
        在考慮圖像傳輸時延時delaytime之後，取得相片拍攝時，跟隨着與領航者之間相差的控制命令cmdWithDelay
        輸入: delayTime,圖像傳輸的延遲; delta, 速度控制命令的間隔
        '''
        historyCmdNumber = int(round(delayTime/delta)) 
        if historyCmdNumber >15:
            print('WARN: Too large delay')
            return (None, None, None)

        if historyCmdNumber > 0 and historyCmdNumber <=15:
            leaderCmdTemp = np.append(self.leaderCmdListHistory[-(historyCmdNumber):],self.leaderCmdList[:-(historyCmdNumber)]) #拍攝到現在，領航者走過到指令
            leaderHistCmd = self.leaderCmdListHistory[-(historyCmdNumber):] #拍攝時到現在的坐標遠點，領航者走過到速度指令
            followerHistCmd = self.modifiedCmdHistory[-(historyCmdNumber):] #拍攝時到現在，跟隨着走過的速度指令
            return ( leaderCmdTemp, leaderHistCmd, followerHistCmd ) 

        if historyCmdNumber==0: #圖像沒有延遲
            return (self.leaderCmdList, None, None)


    
    #def clearModifiedCmd:
        #clear up modified







def pose_estimate_callback(pose):
    '''
    Call back function of pose_estimate  topic
    在这里运行PD修正算法
    传入参数pose是PoseEstimated类型的消息,包含 float32 x, float32 y, float32 theta 
    '''
    #print (pose.x, pose.y, pose.theta)
    global cmdMsgList #存储历史数据队列的实例，在main函数中创建
    


    ###1.考慮圖像傳輸延遲，計算跟隨着當前的位置
    imageDelayTime=pose.delay #獲得圖像的延遲時間

    leaderCmdTemp, leaderHistCmd, followerHistCmd = cmdMsgList.fetchCmdListWithDelay(imageDelayTime)
    print('leaderCmdTemp', leaderCmdTemp)
    
    #1.1如果圖像延遲過大，直接放棄這次圖像估計
    if leaderCmdTemp == None: 
        print('WARN: Delay of Image too large')
        return 
    #1.2 根據領航者的軌跡和攝像頭信息，計算跟隨者的位置
    p_l_delay=speed2list.locationCalV1_2(leaderCmdTemp,0.2) #計算圖像拍攝時刻的領航者軌跡點
    #p_l=speed2list.locationCalV1_2(cmdMsgList.leaderCmdList,0.2)
    leaderCurrentPose=p_l_delay[-1,:] #根据历史数据队列，解算出的领航者当前的坐标和姿态(以圖像拍攝時刻，跟隨者的期望坐標爲遠點)
    #print(leaderCurrentPose)
    '''
    #根据视觉估计结果，计算跟随者的当前坐标和姿态
    #设领航者底盘位姿: [x_l, y_l, theta_l]
    #领航者标志板位姿：[x_m, y_m, theta_m=theta_l]

    #跟随者底盘位姿： [x_f, y_f, theta_f]
    #跟随者相机位姿： [x_c, y_c, theta_c=theta_f]

    #视觉测量相对位姿态(跟谁者相机相对于领航者车尾)：delta_x,delta_y,delta_theta
    #相机到底盘长度为l=0.14m
    则有： theta_f=theta_l-delta_theta
          
    '''
    ###########通过视觉测量，估计跟随者在[圖像拍攝時刻]全局坐标系中的位姿，需要考慮標志板、攝像機到各自地盤中心的距離###########
    #领航者底盘位姿
    theta_m=theta_l = leaderCurrentPose[2]
    pos_l = np.array([leaderCurrentPose[0],leaderCurrentPose[1]])
    #print('leader pos_l')
    #print(pos_l,theta_m)
    #视觉估计所得相对位姿
    delta_theta = pose.theta
    pos_delta = np.array([pose.x, pose.y])
    #相机到底盘的距离
    l=0.14 
    #领航者标志板的坐标
    pos_m = pos_l-l*np.array([-np.sin(theta_m),np.cos(theta_m)])
    #跟随者的航角
    theta_c = theta_f=theta_l-delta_theta
    #计算相机的全局坐标 pos_c
    matTemp = np.array([[np.cos(theta_f),-np.sin(theta_f)],[np.sin(theta_f),np.cos(theta_f)]])
    #print(matTemp.dot(pos_delta))
    pos_c = pos_m-matTemp.dot(pos_delta)
    #根据pos_c转换到跟随者的全局坐标pos_f
    pos_f = pos_c-l*np.array([-np.sin(theta_c),np.cos(theta_c)])
    #print('follower pos_f')
    #print(pos_f,theta_f)
    followerInitPos = np.append(pos_f,theta_f) #！！！計算得到啦圖像拍攝時刻，跟隨者的初始偏差位姿
    #print(followerInitPos)

    #1.3 利用從[圖像拍攝時刻]到[當前時刻]領航者和跟隨着走過的路徑，換算出[當前時刻]全局坐標系中跟隨者的初始位置
    if leaderHistCmd != None: #此時需要考慮到圖像傳輸存在延時
        pass
        leaderHist_Tra = speed2list.locationCalV1_2( leaderHistCmd,0.2) #領航者在圖像延遲時間內走過的軌跡
        followerHist_Tra = speed2list.locationCalV1_2(followerHistCmd, 0.2, followerInitPos) #跟隨者在圖像延遲時間內走過的軌跡
        leaderLastPose = leaderHist_Tra[-1] 
        followerLastPose = followerHist_Tra[-1]
        print("follower Pos at shot time: ",followerInitPos) #视觉估计所得，当前跟随者相对航线起始点的位姿偏差

        thetaGlobal2leader = leaderLastPose[2]
        matrixTexp=np.array([[np.cos(thetaGlobal2leader),np.sin(thetaGlobal2leader)],[-np.sin(thetaGlobal2leader),np.cos(thetaGlobal2leader)]])
        
        follower2leaderPos = matrixTexp.dot((followerLastPose[0:2]-leaderLastPose[0:2]) )
        follower2leaderTheta = followerLastPose[2]- leaderLastPose[2]
        followerInitPos = np.append(follower2leaderPos,follower2leaderTheta)
        print("follower Pos at current :  ",followerInitPos) #视觉估计所得，当前跟随者相对航线起始点的位姿偏差
        print ('    ')

        ####TODO: 計算followerLastPose相對於leaderLastPose的位姿
        # followerInitPos=?
    pass
    
    ######得到了跟随者当前的全局姿态followerInit，下一步使PD进行修正##########

    ###简单判断PD修正是否要介入#########
    #前后车距离大于0.8m，后车偏差在5-35 degrees时进行修正
    p_l=speed2list.locationCalV1_2(cmdMsgList.leaderCmdList,0.2)#領航者的軌跡

    #print("follower init Pos: ",followerInitPos) #视觉估计所得，当前跟随者相对航线起始点的位姿偏差
    if (np.abs(followerInitPos[1])>0.04 or np.abs(followerInitPos[0])>0.04) and 0.0373 < np.abs(followerInitPos[2]) and np.abs(followerInitPos[2]) < 0.685: 
        print('PD control involved.')
        print("follower init Pos: ",followerInitPos)
        #retrive raw cmd list
        cmd_raw=cmdMsgList.leaderCmdList
        cmdMsgList.modifiedCmd = speed2list.pid_V1_2(cmd_raw,followerInitPos,p_l)



    pass
    
def cmd_vel_callback(data): #cmd_vel消息的回调函数
    #push latest recevied cmd_vel msg in to cmdMsgList
    cmdMsgList.push([[data.linear.x, data.angular.z]])
    
    #get/pop a cmd used to play
    cmdToPlay=cmdMsgList.get()
    twist.linear.x = cmdToPlay[0]
    twist.angular.z = cmdToPlay[1]
    #print(twist)
    pub.publish(twist)

    
''' 
    #twist = Twist()
    twist.linear.x = data.linear.x; twist.linear.y = data.linear.y; twist.linear.z = data.linear.z;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = data.angular.z
    pub.publish(twist)

''' 
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('formation_control_rare1', anonymous=True)
    #前车的控制话题
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    #本车的摄像机话题
    rospy.Subscriber("pose_estimate", PoseEstimated,pose_estimate_callback,queue_size = 1)
    # spin() simply keeps python from exiting until this node is stopped
    
    
    
    
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

    
    
if __name__=="__main__":

    #initMsgQueue(msgQueue) #初始化用于缓存cmd_cvl消息的队列
    #rint(speed2list.debugFlag)
    print('This is formation control form Rob_rear1.')
    cmdMsgList=CmdMsgList()
    listener() #注册订阅者，在函数内部设置参数
    #本车的控制话题
    pub = rospy.Publisher('riki16/cmd_vel', Twist, queue_size = 1)
    twist = Twist()
    rospy.spin()

