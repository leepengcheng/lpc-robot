#coding:utf-8
import cv2
import tf
import numpy as np
from math import tan,radians
import vrep
import time

def genMarker(index,size=200):
    maker_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker=cv2.aruco.drawMarker(maker_dict,index,size,1)
    return marker,maker_dict


def getMarkerPoseImage(img,maker_dict):
    image=np.copy(img)
    corners,ids,rejectpts=cv2.aruco.detectMarkers(image,maker_dict)
    image=cv2.aruco.drawDetectedMarkers(image,corners,ids)
    # img=cv2.aruco.drawDetectedMarkers(img,corners,ids)
    cx=cy=512
    fx=fy=cx/tan(radians(60.0) / 2)
    camMat=np.array([
        [fx,0,cx],
        [0,fy,cy],
        [0,0,1]
    ],dtype=np.float64)
    distMat=np.array([0,0,0,0,0],dtype=np.float64)
    rvecs,tvecs,objpts=cv2.aruco.estimatePoseSingleMarkers(corners,0.1,camMat,distMat)
    if rvecs is not None:
        for x in range(len(rvecs)):
            cv2.aruco.drawAxis(image,camMat,distMat,rvecs[x],tvecs[x],0.1)
            rot_matrix=cv2.Rodrigues(rvecs[x].flatten())[0]
            euler_ang=tf.transformations.euler_from_matrix(rot_matrix)
            print("ARUCO:",euler_ang,tvecs[x].flatten().tolist())
    return image
    



# marker,maker_dict=genMarker(49,200)
# # cv2.imshow("maeker",marker)
# # cv2.imwrite("./marker_49_250.png",marker)
# # cv2.waitKey(0)
# img=cv2.imread("/home/zen/图片/aruco.png")
# corners,ids,rejectpts=cv2.aruco.detectMarkers(img,maker_dict)



# rvecs,tvecs,objpts=cv2.aruco.estimatePoseSingleMarkers(corners,0.1,camMat,distMat)
# for x in range(len(rvecs)):
#     img=cv2.aruco.drawAxis(img,camMat,distMat,rvecs[x],tvecs[x],0.1)
# cv2.imshow("maeker",img)
# cv2.waitKey(0)












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
    #获得深度相机的近景和远景距离
    err,nearClip=vrep.simxGetObjectFloatParameter(clientID,sensorHandle,vrep.sim_visionfloatparam_near_clipping,vrep.simx_opmode_oneshot_wait)
    err,farClip=vrep.simxGetObjectFloatParameter(clientID,sensorHandle,vrep.sim_visionfloatparam_far_clipping,vrep.simx_opmode_oneshot_wait)
    print("farclip:%s  nearclip:%s"%(farClip,nearClip))
    #深度相机位置
    err,sensorPostion=vrep.simxGetObjectPosition(clientID,sensorHandle,-1,vrep.simx_opmode_oneshot_wait)
    #发送流传送命令
    if IMG_TYPE==IMG_RGB:
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_streaming)
    elif IMG_TYPE==IMG_DEPTH:
        err, resolution, image=vrep.simxGetVisionSensorDepthBuffer(clientID, sensorHandle, vrep.simx_opmode_streaming)
    else:
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_streaming)
    if err==0:
        print("First Image Get Ok")
    time.sleep(2)

    while (vrep.simxGetConnectionId(clientID) != -1):
        if IMG_TYPE==IMG_RGB:
            err, resolution, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_buffer)
        elif IMG_TYPE==IMG_DEPTH:
            err, resolution, img=vrep.simxGetVisionSensorDepthBuffer(clientID, sensorHandle, vrep.simx_opmode_buffer)
        else:
            err, resolution, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_buffer)
        _,box_pos=vrep.simxGetObjectPosition(clientID,boxHandle,sensorHandle,vrep.simx_opmode_oneshot_wait)
        _,box_ori=vrep.simxGetObjectOrientation(clientID,boxHandle,sensorHandle,vrep.simx_opmode_oneshot_wait)
        print("VREP:",box_ori,box_pos)
        if err == vrep.simx_return_ok:
            if IMG_TYPE==IMG_RGB:#RGB图q
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
                np_img=cv2.cvtColor(np_img,cv2.COLOR_BGR2RGB)
                np_img=getMarkerPoseImage(np_img,maker_dict)
                cv2.imshow('image',np_img) 
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        elif err == vrep.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
            print(err)
else:
  print("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)

# cv2.destroyAllWindows()    


