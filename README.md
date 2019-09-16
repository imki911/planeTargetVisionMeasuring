# planeTargetVisionMeasuring
基于视觉的平面目标测量。
# 文件说明:
* calibrate.py 
  根据棋盘图校准相机,结果存储为npz文件
* measureTarget.py 
  基于视觉的平面位置识别. 在平面上放置已知世界坐标系的marker, 机器人顶端张贴marker,拍摄成录像. 程序读入录像,通过点面定位,获得机器人的世界坐标系.
