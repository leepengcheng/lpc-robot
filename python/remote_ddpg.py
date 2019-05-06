#coding:utf-8
import vrep
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import *
from itertools import product

res=512
angle=60
cx = cy = res * 0.5
fx = fy = cx / np.tan(angle / 2)



IMG_RGB=1
IMG_GRAY=2
IMG_TYPE=IMG_RGB  #当前图片采集类型为RGB



vrep.simxFinish(-1) #关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)#开启连接

if clientID!=-1:
    print('Connected to remote API server')
    #获得相机句柄
    _, sensorHandle = vrep.simxGetObjectHandle(clientID, 'car_sensor', vrep.simx_opmode_oneshot_wait)
    _, sphereHandle = vrep.simxGetObjectHandle(clientID, 'top_camera', vrep.simx_opmode_oneshot_wait)
    #获得深度相机的近景和远景距离
    _,nearClip=vrep.simxGetObjectFloatParameter(clientID,sensorHandle,vrep.sim_visionfloatparam_near_clipping,vrep.simx_opmode_oneshot_wait)
    _,farClip=vrep.simxGetObjectFloatParameter(clientID,sensorHandle,vrep.sim_visionfloatparam_far_clipping,vrep.simx_opmode_oneshot_wait)
    print("farclip:%s  nearclip:%s"%(farClip,nearClip))
    #深度相机位置
    _,sensorPostion=vrep.simxGetObjectPosition(clientID,sensorHandle,-1,vrep.simx_opmode_oneshot_wait)
    #发送流传送命令
    if IMG_TYPE==IMG_RGB:
        _, resolution, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_streaming)
    else:
        _, resolution, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_streaming)
    time.sleep(2)
    while (vrep.simxGetConnectionId(clientID) != -1):
        #参数2,options为1时表示灰度图,为0时RGB图,官方教程错误
        err, resolution, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            if IMG_TYPE==IMG_RGB:#RGB图
                img=np.array(img,dtype=np.uint8)
                img.resize([resolution[1],resolution[0],3])
            else:
                img=np.array(img,dtype=np.uint8)
                img.resize([resolution[1],resolution[0]])
            img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
            img=cv2.flip(img,0)
            cv2.imshow('image',img) 
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == vrep.simx_return_novalue_flag:
            print("waiting..")
        else:
            print(err)
else:
  print("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)

cv2.destroyAllWindows()

