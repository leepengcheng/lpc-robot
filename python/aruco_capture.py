#coding:utf-8
from __future__ import print_function
import cv2 as cv
import numpy as np
from math import tan, radians
import time
import vrep
import tf
from vrepUtil import getVirtualCamInternalMatrix

np.set_printoptions(precision=4, suppress=True)


def genMarker(index, size=200):
    "生成marker"
    maker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    marker = cv.aruco.drawMarker(maker_dict, index, size, 1)
    return marker, maker_dict

def filterMatrix(rots, trans, ids):
    # global faceEulers,facePostions
    ids_str=["c%s"%x for x in ids]
    n=len(ids_str)
    eulerOrders=[]
    posOrders=[]
    for i in range(n):
        name=ids_str[i]
        eulerOrders.append(np.linalg.norm(rots[i]-faceEulers[name]))
        posOrders.append(np.linalg.norm(trans[i]-facePostions[name]))
        faceEulers[name]=rots[i]
        facePostions[name]=trans[i]
    index1,index2=np.argmin(eulerOrders),np.argmin(posOrders)
    # print(min(eulerOrders),max(eulerOrders))
    if eulerOrders[index1]<=eulerStepRange[0] or eulerOrders[index1]>=eulerStepRange[1]:
        return None,None,None
    else:
        return rots[index1],trans[index1],ids[index1]


    

        


def getMarkerPoseImage(img, maker_dict, camInternalMat, camDistortMat):
    '''在图片中标注marker的位姿'''
    image = np.copy(img)
    corners, ids, rejectpts = cv.aruco.detectMarkers(image, maker_dict)
    image = cv.aruco.drawDetectedMarkers(image, corners, ids)
    # img=cv.aruco.drawDetectedMarkers(img,corners,ids)
    rvecs, tvecs, objpts = cv.aruco.estimatePoseSingleMarkers(
        corners, 0.1, camInternalMat, camDistortMat)
    rotMats, tranMats = [], []
    if rvecs is not None:
        for x in range(len(rvecs)):
            #返回的是旋转向量
            cv.aruco.drawAxis(image, camInternalMat, camDistortMat, rvecs[x],
                              tvecs[x], 0.1)
            #将旋转向量转换为旋转矩阵
            rotMats.append(cv.Rodrigues(rvecs[x].flatten())[0])
            tranMats.append(tvecs[x].flatten())
    if ids is None:
        ids=np.array([])
    return image, rotMats, tranMats, ids.flatten() 


####################未进行标定，参数瞎写的##############################################
angle = np.deg2rad(60)  #
resolution = (480, 640)  #相机分辨率
camInternalMat = getVirtualCamInternalMatrix(angle, resolution, False)  #相机内参
camDistortMat = np.array([0, 0, 0, 0, 0], dtype=np.float64)  #畸变系数
################################################################################

maker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)  #aruco 字典
cap = cv.VideoCapture(0)
emptyBuff = bytearray()

#vrep 连接初始化
vrep.simxFinish(-1)  # 关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 开启连接

if clientID == -1:
    raise NameError("Could not connect Vrep")

res, sensorHandle = vrep.simxGetObjectHandle(clientID, 'usbCam',
                                             vrep.simx_opmode_oneshot_wait)
res, box1Handle = vrep.simxGetObjectHandle(clientID, 'aruco_box_1',
                                           vrep.simx_opmode_oneshot_wait)

# faceHandles = {}
MIN=999
MAX=0
eulerStepRange=(0.05,1.0)
postionThrehold=0.1
faceNames=("c10", "c16", "c32", "c36", "c38", "c42")
faceEulers   ={}
facePostions ={}
for name in faceNames:
    faceEulers[name]=np.zeros((3,3))
    facePostions[name]=np.zeros(3)
#     res, faceHandles[name] = vrep.simxGetObjectHandle(
#         clientID, name, vrep.simx_opmode_oneshot_wait)
    # res, Txyz = vrep.simxGetObjectPosition(
    #     clientID, box1Handle, faceHandles[name], vrep.simx_opmode_oneshot_wait)
    # res, Rxyz = vrep.simxGetObjectOrientation(
    #     clientID, box1Handle, faceHandles[name], vrep.simx_opmode_oneshot_wait)

while vrep.simxGetConnectionId(clientID) != -1:
    status, img = cap.read()
    if status:
        img, rots, trans, ids = getMarkerPoseImage(img, maker_dict, camInternalMat, camDistortMat)
        if len(ids):
            R,T,ID_=filterMatrix(rots, trans, ids)
            if R is None:
                continue
            data=[]
            for i in range(3):
                for j in range(4):
                    if j==3:
                        data.append(T[i])
                    else:
                        data.append(R[i,j])
                        
            vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript,'setPosition', [], data, ['c%s'%ID_], emptyBuff,vrep.simx_opmode_oneshot)
        # for m in range(len(rots)):
        #     rot = rots[m]
        #     ori_w = tf.transformations.euler_from_matrix(rot, 'rxyz')
        #     pos_w = trans[m]
        cv.imshow("Video", img)
        k = cv.waitKey(1)
        if k == 27:
            break
    else:
        print("Image Capture Error")
        break


