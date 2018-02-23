#coding:utf-8
import vrep
import cv2
import tf
import numpy as np
from math import tan,radians
from vrepUtil import *
import time

#禁用科学计数法
np.set_printoptions(precision=4, suppress=True) 


def genMarker(index,size=200):
    maker_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker=cv2.aruco.drawMarker(maker_dict,index,size,1)
    return marker,maker_dict


def getMarkerPoseImage(img,maker_dict,camMat,distMat):
    '''在图片中标注marker的位姿'''
    image=np.copy(img)
    corners,ids,rejectpts=cv2.aruco.detectMarkers(image,maker_dict)
    image=cv2.aruco.drawDetectedMarkers(image,corners,ids)
    # img=cv2.aruco.drawDetectedMarkers(img,corners,ids)
    rvecs,tvecs,objpts=cv2.aruco.estimatePoseSingleMarkers(corners,0.1,camMat,distMat)
    rotMats,tranMats=[],[]
    if rvecs is not None:
        for x in range(len(rvecs)):
            #返回的是旋转向量
            cv2.aruco.drawAxis(image,camMat,distMat,rvecs[x],tvecs[x],0.1)
            #将旋转向量转换为旋转矩阵
            rotMats.append(cv2.Rodrigues(rvecs[x].flatten())[0])
            tranMats.append(tvecs[x].flatten().tolist())
    return image,rotMats,tranMats
    


IMG_RGB=1
IMG_DEPTH=2
IMG_GRAY=3
##################
IMG_TYPE=IMG_RGB  #当前图片采集类型为RGB
DISP_IMAGE=True
maker_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

vrep.simxFinish(-1) #关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)#开启连接

if clientID!=-1:
    print('Connected to remote API server')
    #获得相机句柄
    res, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor#', vrep.simx_opmode_oneshot_wait)
    res, boxHandle = vrep.simxGetObjectHandle(clientID, 'aruco_box', vrep.simx_opmode_oneshot_wait)
    #获得深度相机的近景/远景距离/视场角/分辨率
    nearClip,farClip,angle,resolution=getCameraParameter(clientID,sensorHandle)
    camMat=getCameraMatrix(angle,resolution,False)            #相机内参
    distMat=np.array([0,0,0,0,0],dtype=np.float64) #畸变系数
    #相机全局坐标系下的位置和姿态
    err,sensorPostion=vrep.simxGetObjectPosition(clientID,sensorHandle,-1,vrep.simx_opmode_oneshot_wait)
    err,sensorEulerAng=vrep.simxGetObjectOrientation(clientID,sensorHandle,-1,vrep.simx_opmode_oneshot_wait)
    sensorEulerAng.append("rxyz")
    cam_homMat=tf.transformations.euler_matrix(*sensorEulerAng)
    #发送流传送命令
    if IMG_TYPE==IMG_RGB:
        err, res_, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_streaming)
    elif IMG_TYPE==IMG_DEPTH:
        err, res_, image=vrep.simxGetVisionSensorDepthBuffer(clientID, sensorHandle, vrep.simx_opmode_streaming)
    else:
        err, res_, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_streaming)
    if err==0:
        print("First Image Get Ok")
    time.sleep(2)

    while (vrep.simxGetConnectionId(clientID) != -1):
        if IMG_TYPE==IMG_RGB:
            err, res_, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_buffer)
        elif IMG_TYPE==IMG_DEPTH:
            err, res_, img=vrep.simxGetVisionSensorDepthBuffer(clientID, sensorHandle, vrep.simx_opmode_buffer)
        else:
            err, res_, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_buffer)
        #获取Box的位置和姿态(全局)
        _,box_pos=vrep.simxGetObjectPosition(clientID,boxHandle,-1,vrep.simx_opmode_oneshot_wait)
        _,box_ori=vrep.simxGetObjectOrientation(clientID,boxHandle,-1,vrep.simx_opmode_oneshot_wait)
        print("VREP:",np.degrees(box_ori))
        if err == vrep.simx_return_ok:
            if IMG_TYPE==IMG_RGB:#RGB图
                np_img=np.array(img,dtype=np.uint8)
                np_img.resize([resolution[1],resolution[0],3]) #转换为RGB图 #转换为ndarray
            elif IMG_TYPE==IMG_DEPTH:#深度图
                np_img=np.array(img,dtype=np.float32)
                np_img.resize([resolution[1],resolution[0]]) #转换为深度图,resolution=W*H,而图片size为H*W
            else:#灰度图
                np_img=np.array(img,dtype=np.uint8)
                np_img.resize([resolution[1],resolution[0]]) #转换为灰度图
            
            np_img=cv2.flip(np_img,0)
            if DISP_IMAGE:
                #显示时需要转换为RGB
                np_img=cv2.cvtColor(np_img,cv2.COLOR_BGR2RGB)
                #获取marker的图片和位姿(相对与相机的)
                np_img,rots,trans=getMarkerPoseImage(np_img,maker_dict,camMat,distMat)
                for m in range(len(rots)):
                    rot=cam_homMat[:3,:3].dot(rots[m])
                    euler=tf.transformations.euler_from_matrix(rot,'rxyz')
                    print("ARUCO",np.degrees(euler))
                cv2.imshow('image',np_img) 
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        elif err == vrep.simx_return_novalue_flag:
            print("no image yet")
        else:
            print(err)
else:
  print("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)

cv2.destroyAllWindows()    


