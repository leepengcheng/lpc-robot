#coding:utf-8
from __future__ import print_function
import cv2 as cv
import numpy as np
from math import tan,radians
import time
from vrepUtil import getCameraMatrix

np.set_printoptions(precision=4, suppress=True)

def genMarker(index,size=200):
    "生成marker"
    maker_dict=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    marker=cv.aruco.drawMarker(maker_dict,index,size,1)
    return marker,maker_dict


def getMarkerPoseImage(img,maker_dict,camMat,distMat):
    '''在图片中标注marker的位姿'''
    image=np.copy(img)
    corners,ids,rejectpts=cv.aruco.detectMarkers(image,maker_dict)
    image=cv.aruco.drawDetectedMarkers(image,corners,ids)
    # img=cv.aruco.drawDetectedMarkers(img,corners,ids)
    rvecs,tvecs,objpts=cv.aruco.estimatePoseSingleMarkers(corners,0.1,camMat,distMat)
    rotMats,tranMats=[],[]
    if rvecs is not None:
        for x in range(len(rvecs)):
            #返回的是旋转向量
            cv.aruco.drawAxis(image,camMat,distMat,rvecs[x],tvecs[x],0.1)
            #将旋转向量转换为旋转矩阵
            rotMats.append(cv.Rodrigues(rvecs[x].flatten())[0])
            tranMats.append(tvecs[x].flatten().tolist())
    return image,rotMats,tranMats


angle=np.deg2rad(60)
resolution=(480,640)
maker_dict=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

camMat=getCameraMatrix(angle,resolution,False)            #相机内参
distMat=np.array([0,0,0,0,0],dtype=np.float64) #畸变系数
cap = cv.VideoCapture(0)
cv.namedWindow("Video")
while True:
    status, img = cap.read()
    if status:
        img,rots,trans=getMarkerPoseImage(img,maker_dict,camMat,distMat)
        cv.imshow("Video", img)
        k = cv.waitKey(1)
        if k == 27:
            break
    else:
        print("Error")
        break
