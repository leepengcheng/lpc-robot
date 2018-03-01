#coding:utf-8
import vrep
import time
#import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import *
from itertools import product

res=512
ux=uy=res*0.5   #图像中心位置
f=0.008  #相机焦距:单位mm
ang=60.0 #视场角
dx=dy=tan(radians(ang/2))*f/(res*0.5)  #单个像素大小:默认图片大小1024*1024
ratio=100                                  #点云位置放大系数



emptyBuff = bytearray()



num=0
datalist=[]
MAXFRMAE=10
IMG_RGB=1
IMG_DEPTH=2
IMG_GRAY=3
FRAME_NUM=0
# plt.ion() #开启绘图交互模式


IMG_TYPE=IMG_DEPTH  #当前图片采集类型为RGB

def drawObjectPosition(data):
    '''动态绘制数据'''
    global datalist,num
    datalist.append(data)
    if len(datalist)<MAXFRMAE:
        return
    plt.figure("Position")
    plt.clf()
    plt.title('Training...')
    plt.xlabel("Time")
    plt.ylabel("Position")
    plt.ylim(0.95,0.96) #设置Y轴范围
    xdata=range(num,num+MAXFRMAE)
    ydata=datalist[num:num+MAXFRMAE]
    ymean=np.mean(ydata)
    plt.plot(xdata,ydata)
    #绘制平均值
    if num%MAXFRMAE==0:
        # plt.plot(ymean*MAXFRMAE,"-mo")
        print("Mean Value: %s"%ymean)
    plt.pause(0.001)
    num+=1



vrep.simxFinish(-1) #关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)#开启连接

if clientID!=-1:
    print('Connected to remote API server')
    #获得相机句柄
    res, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor#', vrep.simx_opmode_oneshot_wait)
    res, sphereHandle = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_oneshot_wait)
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

    #读取视觉传感器的状态,packs={intensity, red, green, blue, depth value}*{min,max,average}
    # err, state, packs=vrep.simxReadVisionSensor(clientID,sensorHandle,vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetObjectPosition(clientID,sensorHandle,sensorHandle,center)
    while (vrep.simxGetConnectionId(clientID) != -1):
        FRAME_NUM=FRAME_NUM+1
        if IMG_TYPE==IMG_RGB:
            #参数2,options为1时表示灰度图,为0时RGB图,官方教程错误
            err, resolution, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 0, vrep.simx_opmode_buffer)
        elif IMG_TYPE==IMG_DEPTH:
            err, resolution, img=vrep.simxGetVisionSensorDepthBuffer(clientID, sensorHandle, vrep.simx_opmode_buffer)
        else:
            err, resolution, img = vrep.simxGetVisionSensorImage(clientID, sensorHandle, 1, vrep.simx_opmode_buffer)

        if err == vrep.simx_return_ok:
            if IMG_TYPE==IMG_RGB:#RGB图
                img=np.array(img,dtype=np.uint8)
                img.resize([resolution[1],resolution[0],3]) #转换为RGB图 #转换为ndarray
            elif IMG_TYPE==IMG_DEPTH:#深度图
                bin_img=np.array(img,dtype=np.float)
                bin_img.resize([resolution[1],resolution[0]]) #转换为深度图,resolution=W*H,而图片size为H*W
                if FRAME_NUM==10:
                    point_cloud=[] #点云图
                    for cy,cx in product(range(resolution[1]),range(resolution[0])):
                        #近距离和远距离不计
                        if 0.5<bin_img[cy,cx]<0.8:
                            if 0.3*512<cx<0.6*512:
                                x=(resolution[0]-cx+ux)*dx*ratio
                                y=(resolution[1]-cy+uy)*dy*ratio
                                z=nearClip+bin_img[cy,cx]*(farClip-nearClip)#深度图算出的实际位置
                                point_cloud.extend([z,x,y]) #0.5为相机高度
                    vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'insertPointCloud_function', [], point_cloud, ['myPointCloud'], emptyBuff, vrep.simx_opmode_blocking)
                img=(bin_img*255).astype(np.uint8)            #映射到0~255深度图
            else:#灰度图
                img=np.array(img,dtype=np.uint8)
                img.resize([resolution[1],resolution[0]]) #转换为灰度图
                

            
            # # img=cv2.flip(img,1) #水平反转图片
            # if IMG_TYPE==IMG_RGB:
            #     region=np.where(cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)<0.5*255)
            # else:
            #     region=np.where(img<0.5*255)
            # if not 0 in map(lambda x:len(x),region):
            #     _cy,_cx=map(lambda x:x.mean(),region) #目标位置
            #     cy,cx=int(_cy),int(_cx)
            #     if 0<=cx<=resolution[0] and 0<=cy<=resolution[1]: 
            #         objectZpos=nearClip+img[cy,cx]*(farClip-nearClip)+sensorPostion[-1]+0.05 #深度图算出的实际位置
            #         # drawObjectPosition(objectZpos)
            #     radius=sum(map(lambda x:x.max()-x.min(),region))
            #     #画图只能在3通道的图片上画,故转换为3通道
            #     # if IMG_TYPE!=IMG_RGB:
            #     #     img=cv2.merge([img,img,img])
            #     print("UV Position: (%.3f %.3f)"%(_cx,_cy))
                # img=cv2.circle(img,(cx,cy),int(radius),(0,0,255),2) #画外边界圆
            # cv2.imshow('image',img) 
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
        elif err == vrep.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
            print(err)
else:
  print("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)

# cv2.destroyAllWindows()    
# plt.ioff()

