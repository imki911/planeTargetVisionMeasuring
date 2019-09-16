#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#2019年05月19日
#对应《公式推导V1.2.docx》中的计算方法，
#用P_l(k)=[x(k),y(k),alpha(k)],1<=k<=N 来表示领航者的一个轨迹点

import time


import pickle
import numpy as np
import matplotlib.pyplot as mat_plot
#from geometry_msgs.msg import Twist

#import sys, select, termios, tty

debugFlag=0


msg = """
Transfer speend List to relative location  
"""



'''

def pubTwistMsg(twist,cmdHistory):
	#readin cmdHistory and publish twist msg. 
	# create a attribute as iterator 
	if not hasattr(pubTwistMsg,'itera'):
		pubTwistMsg.itera=0;
	if pubTwistMsg.itera<len(cmdHistory) :
		twist = cmdHistory[pubTwistMsg.itera]
		pubTwistMsg.itera += 1 
		#print(pubTwistMsg.itera)
		pub.publish(twist)
		#start another thread to achieve periodically timer callback.
		timer = threading.Timer(0.2, pubTwistMsg, [twist,cmdHistory] )
		timer.start()
	else:
		print('playback all the history cmd, exit.')
		exit() 

'''


	
def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn) 
	# 左转为正，右转为负

def readInData(dataPath='./cmdHistoryPyList.txt'):
	'''
	数据读入的接口函数，为了给locationCal()函数提供以列表存储的历史速度变量
	返回numpy array，格式：[linearSpeed, angularSpeed],最早的在最前面，最新的在后面。
	'''
	f = open(dataPath, 'rb')
	cmdHistory=pickle.load(f)
	f.close()
	print('length of cmdHistory: %d.' % len(cmdHistory))
	print(cmdHistory)
	#except:
	#	print 'error'

	return cmdHistory #[[linear,speed]...]




def locationCalV1_2(cmdHistoryList, deltaTime=0.2, p_0=np.array([0.0, 0.0, 0.0]) ):
	'''
	根据速度控制指令数据计算轨迹点

	输入：历史数据列表[[linear,angular],[]...],最早的在前面; 采样间隔(s);初始姿态,格式(nparray:[x,y,alpha])
	输出：返回轨迹点坐标和姿态 p(k)，0<=k<=N,其中p_0是初始姿态
	'''
	# you don't need to figure out how it works, neither you cant. 
	#0）开辟变量
	cmdHistoryList=np.array(cmdHistoryList).reshape([-1,2]) #从list转换为numpy array
	N=cmdHistoryLength=len(cmdHistoryList) #控制数据的长度，也就是N的值
	tracePoint=np.zeros([N+1,3]) #存储轨迹点的array,行数为N+1,每行表示一个轨迹点坐标和姿态
	tracePoint[0,:]=p_0

	subTrace=np.zeros([N,3]) #subTrace([i,:])为在坐标系i下，第i+1段轨迹的坐标和姿态

	#开始计算
	#长度为N+1*N+1的矩阵，用于存储x_ij的值，使用时下标与公式中实际下标对应
	#x_ij_matrix=np.zeros([listLength+1,listLength+1])
	#y_ij_matrix=np.zeros([listLength+1,listLength+1])

	#1.1) 分离出线速度和角速度
	v_array=cmdHistoryList[:,0].copy() #线速度，0～N-1
	w_array=cmdHistoryList[:,1].copy() #角速度，0～N-1

	#记录每个deltaT内的运动类型，将运动情况分为四种：“linear”，“rotate”，“normal”，“stop”
	motionType=list(['']*N)
	for i in range(N):
		if v_array[i] != 0 and w_array[i] != 0: 
	
			motionType[i]='normal'
		elif v_array[i]!= 0 and w_array[i]== 0:
			motionType[i]='linear'
		elif v_array[i]== 0 and w_array[i]!= 0:
			motionType[i]='rotate'
		else:
			motionType[i]='stop'
	#isStright_array=np.array([ (True if i==0  else False ) for i in w_array ])

	#1.2) 旋转的角度 theta_i
	theta_array=w_array*deltaTime #delta t 内，各子轨迹的旋转角度
	subTrace[:,2]=theta_array #每个子轨迹的偏转角度赋值

	#theta_0_n[k],第k个坐标系相对与全局原点的旋转角度
	theta_0_n=theta_array.copy()*0 
	theta_0_n=np.insert(theta_0_n,0,values=p_0[2],axis=0)
	#print(theta_0_n.shape)

	for i in  range(len(theta_array)):
		theta_0_n[i+1]=theta_array[i]+theta_0_n[i]  #对于p(0)不在全局坐标系原点的情况

	#mat_plot.plot(theta_0_n)
	#mat_plot.show()

	#1.3) 计算x_i-1,i 和 y_i-1,i，存入 x_ij_matrix，y_ij_matrix
	for i in range(N): #i从0开始，到N-1，代表坐标系的编号
		if motionType[i]=='normal': #直线加转弯
			r_i=(float(v_array[i])/float(w_array[i]))
			
			x = r_i *( np.cos(theta_array[i]) -1)
			y = r_i * np.sin( theta_array[i])
			subTrace[i,0:2]=[x,y] 
			pass

		elif motionType[i]=='linear': #直线运动
			x =0
			y =v_array[i] * deltaTime
			subTrace[i,0:2]=[x,y] 
			pass

		elif motionType[i]=='rotate': #原地旋转
			x =0
			y =0
			subTrace[i,0:2]=[x,y]
			pass

		else:   #停止状态
			x =0
			y =0
			subTrace[i,0:2]=[x,y]
			pass

	#1.4) 根据递推公式，计算p(k), k=0,1,2,...N

	
	for index in range (1,N+1): #从1到N循环

		g_inv = np.array([ [np.cos(theta_0_n[index-1]),-np.sin(theta_0_n[index-1]),0], \
			[np.sin(theta_0_n[index-1]),np.cos(theta_0_n[index-1]),0],\
				[0,0,1]])
		tracePoint[index,:] = g_inv.dot(subTrace[index-1,:])+tracePoint[index-1,:]

		

	#1.5) 取出结果

	#print(locationHistoryX[-1],locationHistoryY[-1])
	return tracePoint#[:,0],tracePoint[:,1]

	
	pass
	if debugFlag:
		mat_plot.plot(theta_array)
		mat_plot.show()
	



def calculatePose_0_k(p_0=np.array([0,0,0]),u=[0,0], deltaTime=0.2):
	'''
	根据初始姿态和控制命令，返回k时刻机器人在全局坐标系中的朝向
	 输入： 初始位置p_0（numpy array type,[x,y,sita]）；
		  控制命令u （N*2）长度为N
		  采样时间deltaTime
	 输出： sita_0_k,类型：一维numpy数组，长度为N+1. 
	'''
	if not isinstance(u,np.ndarray):
		try:
			cmd=np.array(u)
			pass
		except:
			print('function input error')
			raise
	N=u.shape[0] #输出控制数据的长度
	sita_0_k=np.zeros([N+1])
	cmd=u.copy()
	sita_0_k[0]=p_0[2]
	for i in range(1,N+1):
		sita_0_k[i] = sita_0_k[i-1] + cmd[i-1,1]* deltaTime
	

	return sita_0_k

def pid_V1_2(u_f,p_f_0,tracePoints,deltaTime=0.2):
	'''
	一次PD修正误差。以领航者的轨迹为期望轨迹，以视觉估计所得的跟随者位置为p_f(0). 
	 跟随者修正u_l(k),得到u_f(k)，使得跟随者能跟踪领航者的轨迹
	输入：跟随者本次迭代的控制速度信息u_f (类型为numpyarray，长度为N)；跟随者的当前坐标p_f_0；领航者的轨迹点集tracePoints(长度为N+1)。
	输出：跟随者迭代之后的速度控制信息u_f_i
	'''
	N=u_f.shape[0]
	u_f_nexti = u_f.copy() #修正后的跟随者速度控制命令
	
	#---迭代参数设置-----
	K_p=1
	K_i=0.01
	K_d=1
	L_P=0.1
	#---迭代参数设置↑-----


	#开始逐个更新u_f 
	for k in range(N):
		topkCmd=u_f_nexti[0:k+1,:] #取出前k个控制指令
		p_f_k=locationCalV1_2(topkCmd, 0.2, p_f_0 ) #前k个指令产生的轨迹
		error_leaderMinusFollwer = tracePoints[0:k+2,:]-p_f_k[0:k+2,:] #误差
		sita_0_k=calculatePose_0_k(p_f_0,topkCmd) #第k个子轨迹开始时的朝向
		
		error_p=error_leaderMinusFollwer[k+1,:] #比例项误差
		error_d=error_leaderMinusFollwer[k,:]-error_leaderMinusFollwer[k+1,:] #差分项误差
		#const=np.array([0, sita_0_k[k] ])
		D=np.array([[-np.sin(sita_0_k[k]),np.cos(sita_0_k[k]),0],[-np.cos(sita_0_k[k]),np.sin(sita_0_k[k]),1]])
		#更新u_f第k个数据
		u_f_nexti[k,:]=u_f_nexti[k,:] + K_p * D.dot(error_p ) + K_d * D.dot(error_d )
		#u_f_nexti[k,1]=u_f_nexti[k,1] + K_p * (error_p[2]-np.cos(sita_0_k[k])*error_p[0]+ np.sin(sita_0_k[k])*error_p[1])
		'''
		u_f_nexti[k,0]=u_f_nexti[k,0] + K_p * np.cos(error_p[2]) * np.linalg.norm(error_p[0:2])#np.sum(np.abs(error_p[0:2]))
		u_f_nexti[k,1]=u_f_nexti[k,1] + K_p * (error_p[2]-error_p[0] )
		print('sita_0_k'+str(k)+':'+str(sita_0_k[k]))
		print('error_p[2]'+str(k)+':'+str(error_p[2]))
		'''
	return u_f_nexti


def pid_p(u_f,p_f_0,tracePoints,deltaTime=0.2):
	'''
	一次PID-P迭代学习。以领航者的轨迹为期望轨迹，以视觉估计所得的跟随者位置为p_f(0). 
	 跟随者修正u_l(k),得到u_f(k)，使得跟随者能跟踪领航者的轨迹
	输入：跟随者本次迭代的控制速度信息u_f (长度为N)；跟随者的当前坐标p_f_0；领航者的轨迹点集tracePoints(长度为N+1)。
	输出：跟随者迭代之后的速度控制信息u_f_i
	'''
	N=u_f.shape[0]
	u_f_nexti = u_f.copy() #修正后的跟随者速度控制命令
	
	#---迭代参数设置-----
	K_p=0.4
	K_i=0.01
	K_d=-0.1
	L_P=0.1
	#---迭代参数设置↑-----
	#↓----创建C_k矩阵------↓
	C_k=np.zeros([N,2,3]) #公式中的矩阵C(k), C_k[k,:,:]返回第k个尺寸为2*3的矩阵
	'''
	sita_0_k=p_f_0[2] #
	C_k[0,:,:]=np.array([[-np.sin(sita_0_k),np.cos(sita_0_k),0],\
					[-0.1,0,1]])
	for i in range(1,N):
		sita_0_k = sita_0_k + u_f[i-1,1] * deltaTime
		C_k[i,:,:]=np.array([[-np.sin(sita_0_k),np.cos(sita_0_k),0],\
						[-0.1,0,1]])

	'''

	#'''
	sita_0_k=p_f_0[2] #
	C_k[0,:,:]=np.array([[0,np.cos(sita_0_k),0],\
					[-np.sin(sita_0_k),0,1]])
	for i in range(1,N):
		sita_0_k = sita_0_k + u_f[i-1,1] * deltaTime
		C_k[i,:,:]=np.array([[0,np.cos(sita_0_k),0],\
						[-np.sin(sita_0_k),0,1]])

	#'''
	#↑----创建C_k矩阵----↑

	#1)计算p_f_i   ↓
	p_f_k=locationCalV1_2(u_f, 0.2, p_f_0 )
	#2)计算误差e_i，e_i相比tracePoint去除了原点，因此长度为N  ↓
	e_i=(tracePoints-p_f_k)[1:,:]
	#print(e_i.shape)
	
	assert e_i.shape[0]==N

	#PID更新u_f(k)
	for k in range(N-1):
		#积分项中的误差累加
		sum_e_k=np.sum(e_i[0:k,:],0)
		u_f_nexti[k,:]=u_f[k,:]+C_k[k,:,:].dot(\
			K_p*e_i[k+1,:] + K_d*(e_i[k+1,:]-e_i[k,:]) + K_i*sum_e_k )
	
	#更新P项
	p_f_k=locationCalV1_2(u_f_nexti, 0.2, p_f_0 )
	e_i1 = (tracePoints-p_f_k)[1:,:] #e_i+1
	for k in range(N):
		u_f_nexti[k,:]=u_f_nexti[k,:] +C_k[k,:,:].dot(L_P* e_i1[k,:])
	






	return u_f_nexti





if __name__=="__main__":
	time_start=time.time()
	
	cmdList=readInData('./cmdHistoryPyList.txt')
	follower_init_pos=np.array([0.1,0.1,0.2])
	#(p_dx,p_dy)=locationCal(cmdList,0.2)
	#领航者的轨迹
	p_l=locationCalV1_2(cmdList,0.2) #,np.array([-0.3,0.2,1.56]))
	print('p_l length', p_l.shape)
	#跟随者的原始轨迹
	p_f_0=locationCalV1_2(cmdList, 0.2, follower_init_pos)

	#PD迭代后的轨迹
	u_f_1_pd = pid_V1_2(np.array(cmdList),follower_init_pos,p_l)
	p_f_1_pd=locationCalV1_2(u_f_1_pd, 0.2, follower_init_pos)

	'''
	#一次迭代更新跟随者指令
	u_f_1=pid_p(np.array(cmdList),follower_init_pos,p_l)
	#一次迭代之后跟随者的轨迹
	p_f_1=locationCalV1_2(u_f_1, 0.2, follower_init_pos)

	#多次迭代
	u_fi=u_f_1
	for i in range(1,900):
		u_fi=pid_p(u_fi,follower_init_pos,p_l)

	#多次迭代之后的轨迹

	print('--u_fi--------')
	print(u_fi)
	p_f_final=locationCalV1_2(u_fi, 0.2, follower_init_pos)
	'''

	
	
	
	
	time_end=time.time()
	print('totally cost',time_end-time_start)
	print('--u_f_1_pd--------')
	print(u_f_1_pd )
	mat_plot.figure()
	mat_plot.plot(p_l[:,0],p_l[:,1],label='leader, expected trajectory ')
	mat_plot.plot(p_f_0[:,0],p_f_0[:,1],label="follower's initial trajectory")
	mat_plot.plot(p_f_1_pd[:,0],p_f_1_pd[:,1],label=' after PD iterating')
	#mat_plot.plot(p_f_1[:,0],p_f_1[:,1],label=' iterater 1 time')
	#mat_plot.plot(p_f_final[:,0],p_f_final[:,1],label='10 times')


	#mat_plot.xticks(np.arange(-2,2,-0.2))
	#mat_plot.yticks(np.arange(-2,2,-0.2))
	mat_plot.legend()
	mat_plot.minorticks_on()
	mat_plot.axis('equal')
	mat_plot.xlabel('x')
	mat_plot.ylabel('y')
	mat_plot.grid()
	mat_plot.show()
	
	print()
	#print (cmdList) 

	


