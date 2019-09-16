#!/usr/bin/ python
# -*- coding: utf-8 -*- 
#從文件夾讀取棋盤圖片,校準相機,將參數保存到文件.
#2019年7月17日
#Shen Guofeng, mailto:gf@gfshen.cn
#---------------------------------------
import numpy as np
import cv2
import glob

#ref: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html

'''
def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    print('roi ', roi)

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + w]

    return dst
'''

def  calibrate(cornerX=8,cornerY=6, squareSize=24.5,images=glob.glob('*.jpg')  ):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cornerX*cornerY,3), np.float32)
    objp[:,:2] = np.mgrid[0:cornerX,0:cornerY].T.reshape(-1,2) * squareSize  #如果squareSize设错，对mtx和dist的估计没有影响，但对rvecs和tvecs有影响

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

   

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (cornerX,cornerY),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (cornerX,cornerY), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    #开始标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    if ret:
        print('mtx: \n',mtx)
        print('dist: \n', dist)
        #print('rvecs: \n', rvecs)
        #print('tvecs: \n',tvecs)
        np.savez('calibrateData.npz', mtx=mtx, dist=dist)

        
    
    
    
    return ret, mtx, dist, rvecs, tvecs




def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + w]

    return dst


if __name__ == '__main__':

    image=glob.glob('./chessBoardNote3/*.bmp')
    if not image:
        raise RuntimeError('Cant find image,exting')
        
    #用于存储内参矩阵和畸变参数
    mtx = []
    dist = []
    try:
        npzfile = np.load('calibrate.npz')
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        ret, mtx, dist, rvecs, tvecs = calibrate( cornerX=8,cornerY=6, squareSize=24.5,images= image  )
    print('calibration finished, result saved as .npz file ')

