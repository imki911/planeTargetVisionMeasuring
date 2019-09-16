#!/usr/bin/env python
# -- coding: utf-8 --

#2019年8月14日
#Ver1.0
#---Guofeng-----
#gf@gfshen.cn

#将npz数据转换成xls格式






import sys
import numpy as np
import matplotlib.pyplot as plt 
msg = 'Usage: '+str(sys.argv[0])+' <npz fileName>' 



if __name__=="__main__":
    
    #check if arguments met
    if len(sys.argv) != 2:
        print(msg)
        raise Exception('npz file not given')
    
    #try to readin npz file
    try:
        result = np.load(sys.argv[1]) * 1000
    except:
        raise Exception('open npz file error!')
        
    #wirte to txt fileName
    with open('delayData.txt','w') as f:
        for item in result:
            f.write(str(item)+'\n')
    
    
    #绘图:https://matplotlib.org/3.1.1/gallery/lines_bars_and_markers/nan_test.html#sphx-glr-gallery-lines-bars-and-markers-nan-test-py
    len=result.shape[0]
    axisX=np.arange(len)*0.2
    plt.subplot(1,1,1)
    plt.plot(axisX, result)
    plt.xlabel('time, s ')
    plt.ylabel('delay, ms')
    plt.show()

