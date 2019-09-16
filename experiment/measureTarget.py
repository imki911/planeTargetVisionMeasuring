#!/usr/bin/ python
# -*- coding: utf-8 -*- 

# 使用視覺方法測量目標在世界坐標系中的坐標
# 首先估計相機姿態,然後測算目標marker中心點在世界坐標系中的位置.
# 使用方法:
# 1. 相機校準,
# 2. 在空間中放置4個以上的基準坐標點,在程序中給定這些點的信息,包括ID和世界坐標
# 3. 被測目標使用marker標記,在程序中給定這些點的markerID
# 4. 拍攝錄像,確保4個標志點在視野內.
# 5. 運行程序處理視頻幀
# CR@ Shen Guofeng, mailto:gf@gfshen.cn
# 
# ------版本歷史---
# ---V1.0
# ---2019年7月19日
#    初次編寫



import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot  as plt




def estimateCameraPose(cameraMtx, dist, refMarkerArray,corners,markerIDs):
    '''
    根据基准点的marker，解算相机的旋转向量rvecs和平移向量tvecs，(solvePnP(）实现)
    并将rvecs转换为旋转矩阵输出(通过Rodrigues())
    输入：
        cameraMtx内参矩阵，
        dist畸变系数。
        当前处理的图像帧frame，
        用于定位世界坐标系的参考点refMarkerArray.  py字典类型,需要len(refMarkerArray)>=3, 格式：{ID:[X, Y, Z], ID:[X,Y,Z]..}
        corners, detectMarkers()函數的輸出
        markerIDs, detectMarkers()函數的輸出
    输出：旋转矩阵rMatrix, 平移向量tVecs
    '''
    marker_count = len(refMarkerArray)
    if marker_count<4: #标志板少于四个
        raise RuntimeError('at least 3 pair of points required when invoking solvePnP')
    

    corners=corners; ids=markerIDs
    print('ids:\n')
    print(ids)
    print('corners:\n')
    print(corners)

    objectPoints=[]
    imagePoints=[]
    #检查是否探测到了所有预期的基准marker
    if len(ids) !=0: #檢測到了marker,存儲marker的世界坐標到objectPoints，構建對應的圖像平面坐標列表 imagePoints
        print('------detected ref markers----')
        for i in range(len(ids)): #遍歷探測到的marker ID,
            if ids[i][0] in refMarkerArray: #如果是參考點的標志，提取基准点的图像坐标，用于构建solvePnP()的输入

                print('id:\n ' + str(ids[i][0]))
                print('cornors: \n '+ str(corners[i][0]))
                objectPoints.append(refMarkerArray[ ids[i][0] ])
                imagePoints.append(corners[i][0][0].tolist()) #提取marker的左上點
        objectPoints=np.array(objectPoints)
        imagePoints=np.array(imagePoints)
            
        print('------------------------------\n')
        print('objectPoints:\n'+str(objectPoints))
        print('imagePoints:\n'+str(imagePoints))
        pass
    else:
        return False, None, None

    #如果檢測到的基準參考點大於3個，可以解算相機的姿態啦
    if len(objectPoints)>=4:
        #至少需要4個點
        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMtx, dist)
        rMatrix,jacobian = cv2.Rodrigues(rvec)
        return True, rMatrix, tvec
    else:
        return False, None, None


    #返回值
    #return rMatrix=[], tVecs=[]



def detectTarget(cameraMatrix, dist, rMatrix, tvec, targetMarker, corners, markerIDs,zWorld = 0.0):
    '''
    測算目標marker中心在世界坐標系中的位置
    輸入:

    輸出:
        與markerIDs長度相等的列表,包含位置確定的目標坐標,未檢測到填None,例如[None,[x2,y2,z2]]
    '''

    targets_count=len(targetMarker)
    if targets_count == 0:
        raise Exception('targets empty, areyou dou?')

    #創建與targetMarker相同尺寸的列表,用於存儲解算所得到目標的世界坐標
    targetsWorldPoint=[None] * targets_count
    if rMatrix==[]:
        return targetsWorldPoint
    for i in range(len(markerIDs)): #遍歷探測到的marker ID,
        markerIDThisIterate = markerIDs[i][0]
        if markerIDThisIterate in targetMarker: #如果是目標marker的ID
            #獲得當前處理的marker在targetMarker中的下標,用於填充targetsWorldPoint
            targetIndex = targetMarker.index(markerIDThisIterate)
        else:
            continue
        #計算marker中心的圖像坐標
        markerCenter = corners[i][0].sum(0)/4.0
        #畸變較正,轉換到相機坐標系,得到(u,v,1)
        #https://stackoverflow.com/questions/39394785/opencv-get-3d-coordinates-from-2d
        markerCenterIdeal=cv2.undistortPoints(markerCenter.reshape([1,-1,2]),cameraMatrix,dist)
        markerCameraCoodinate=np.append(markerCenterIdeal[0][0],[1])
        print('++++++++markerCameraCoodinate')
        print(markerCameraCoodinate)

        #marker的坐標從相機轉換到世界坐標
        markerWorldCoodinate = np.linalg.inv(rMatrix).dot((markerCameraCoodinate-tvec.reshape(3)) )
        print('++++++++markerworldCoodinate')
        print(markerWorldCoodinate)
        #將相機的坐標原點轉換到世界坐標系
        originWorldCoodinate = np.linalg.inv(rMatrix).dot((np.array([0, 0, 0.0])-tvec.reshape(3)) )
        #兩點確定了一條直線 (x-x0)/(x0-x1) = (y-y0)/(y0-y1) = (z-z0)/(z0-z1) 
        #當z=0時,算得x,y
        delta = originWorldCoodinate-markerWorldCoodinate
        #zWorld = 0.0
        xWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[0] + originWorldCoodinate[0]
        yWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[1] + originWorldCoodinate[1]
        targetsWorldPoint[targetIndex]=[xWorld,yWorld,zWorld]
        
        print('-=-=-=\n Target Position '+ str(targetsWorldPoint[targetIndex]) )
        pass
    return targetsWorldPoint





if __name__ == '__main__':
    videoSource = './video/note3/square_large_Corner_MOV_0071.mp4'
    cap = cv2.VideoCapture(videoSource)
    try:
        npzfile = np.load('./calibrateDataNote3.npz')
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        raise Exception('cant find calibration data, do that first')
    
    #保存基準點的信息,檢測到之後會更新.
    rMatrix=[]
    tvec=[]
    #######
    
    ###繪圖相關##########
    plt.ion() #開啓交互模式
    fig = plt.figure(1)
    axes = plt.gca()
    axes.set_xlim([0,6])
    axes.set_ylim([0,8])
    axes.set_aspect('equal', adjustable='box')
    #plt.scatter(0,0,c='g',alpha=0.5, label='leader')
    #plt.scatter(0,0,c='r',alpha=0.5,label='follower')
    #plt.legend()
    #plt.pause(0.01)

    followerPos=[]
    leaderPos=[]

    #ax=fig.add_subplot(111)
    
    #逐幀處理視頻畫面
    frameCount=0 #視頻幀計數
    while(True):    
        ret, frame = cap.read()
        frameCount += 1
        cv2.namedWindow('image',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', 1280,720)
        cv2.imshow("image",frame)
        

        ##process and measure target position
        #0.1. 指定基準點的marker ID和世界坐標
        # [[marker ID, X, Y, Z]..]
        refMarkerArray={   \
            0: [4.0, 6.0, 0.0], \
            1: [4.0, 2.0, 0.0], \
            2: [2.0, 2.0, 0.0], \
            3: [2.0, 6.0, 0.0], \
        }
        #0.2 指定目標的markr ID
        targetMarker =[10,11]

        #1. 估計camera pose 
        #1.1 detect aruco markers
        img_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
       
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)
        ####!!!!!hack coding HERE!!!!#######
        ##marker 0 cant be detected, so manually add it 
        if not 0 in ids:
            ids=np.append(ids,np.array([0],dtype=np.int32)).reshape(-1,1)
            corners.append(np.array([[[1191,177],[1238,192],[1196,192],[1192,178]]], dtype=np.float32))
        print('!!!Hack Coding!!!!! 不是常規操作,非戰鬥人員請注銷這塊代碼')
        #print('cornetrs shape:\n',corners.shape())
        #print('corners\n',corners)
        
        #########!!!!hack coding above!!!!##########
        aruco.drawDetectedMarkers(img_gray, corners) #Draw A square around the markers
        
        cv2.namedWindow('detect',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('detect', 1280,720)
        cv2.imshow("detect",img_gray)
        
        #1.2 estimate camera pose
        gotCameraPose, rMatrixTemp, tvecTemp = estimateCameraPose(mtx, dist, refMarkerArray,corners,ids)

        #1.3 updata R, T to static value 
        if gotCameraPose: 
            rMatrix = rMatrixTemp
            tvec = tvecTemp
            print('rMatrix\n'+str(rMatrixTemp))
            print('tvec\n'+str(tvecTemp))

        #2. 根據目標的marker來計算世界坐標系坐標
        targets = detectTarget(mtx, dist, rMatrix, tvec, targetMarker, corners, ids)
        if targets[0] != None:
            pass
            followerPos.append(targets[0])
            #plt.scatter(targets[0][0],targets[0][1],c='g',alpha=0.5)
            #plt.pause(0.0001)
            
            #plt.show()
            #ax.scatter(targets[0][0],targets[0][1],'-r')
        if targets[1] != None:
            pass
            #plt.scatter(targets[1][0],targets[1][1],c='r',alpha=0.5)
            leaderPos.append(targets[1])
            #plt.legend()
            #plt.pause(0.0001)
            
            
            #plt.show()
            #ax.scatter(targets[1][0],targets[1][1],'-g')
        
        
        if frameCount%30 == 0:
            plt.cla()
            plt.gca().set_xlim([0,6])
            plt.gca().set_ylim([0,8])
            plt.gca().set_aspect('equal', adjustable='box')
            plt.plot(np.array(followerPos)[:,0],np.array(followerPos)[:,1],c='g',alpha=0.5 , label='follower')
            plt.plot(np.array(leaderPos)[:,0],np.array(leaderPos)[:,1],c='r',alpha=0.5, label='leader')
            #plt.scatter(np.array(followerPos)[:,0],np.array(followerPos)[:,1],c='g',alpha=0.5 ,linewidths=0, label='leader')
            #plt.scatter(np.array(leaderPos)[:,0],np.array(leaderPos)[:,1],c='r',alpha=0.5,linewidths=0,label='follower')
            plt.legend()
            plt.pause(0.01)
            plt.savefig("Trajectory.png")
            
            
        
        if ( cv2.waitKey(10) & 0xFF ) == ord('q'):
            plt.cla()
            plt.gca().set_xlim([0,6])
            plt.gca().set_ylim([0,8])
            plt.gca().set_aspect('equal', adjustable='box')
            plt.plot(np.array(followerPos)[:,0],np.array(followerPos)[:,1],c='g',alpha=0.5 , label='follower')
            plt.plot(np.array(leaderPos)[:,0],np.array(leaderPos)[:,1],c='r',alpha=0.5, label='leader')
            plt.savefig("Trajectory.png")
            
            cap.release()
            cv2.destroyAllWindows()
            break
    
    cap.release()
    cv2.destroyAllWindows()
