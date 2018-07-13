#coding:utf-8
import vrep
import numpy as np
from math import tan

def getVirtualCamInternalMatrix(angle,resolution,flatten=True):
    '''获取相机的内参和投影矩阵
    @angle为视场角的弧度
    @flatten:为true时转换为1维list
    '''
    res =resolution
    cx = cy = res[0] * 0.5  
    #方法1:间接求
    # f = 0.008  # 相机焦距:单位mm(虚拟焦距)
    # dx = dy = tan(angle / 2) * f / (cx * 0.5)  # 单个像素大小
    # fx = f / dx #焦距的像素长度
    # fy = f / dy
    #方法2:直接求
    fx=fy=cx/tan(angle / 2) 
    Tx = Ty = 1
    K = np.array([[fx, 0, cx],
                [0, fy, cy],
                [0, 0,   1]],dtype=np.float64)  # 相机内参
    P = np.array([[fx, 0, cx, 0],
                [0, fy, cy, 0],
                [0, 0,   1, 0]],dtype=np.float64) # 投影矩阵
    if not flatten:
        return K
    return map(lambda x:x.flatten().tolist(),(K,P))


def getVirtualCamAdditionalMatrix(clientID, sensorHandle):
    "获得深度相机的近景/远景距离/视场角/分辨率"
    err, nearClip = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_near_clipping, vrep.simx_opmode_oneshot_wait)
    err, farClip = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_far_clipping, vrep.simx_opmode_oneshot_wait)
    err, angle = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_perspective_angle, vrep.simx_opmode_oneshot_wait)
    err, res_x = vrep.simxGetObjectIntParameter(
        clientID, sensorHandle, vrep.sim_visionintparam_resolution_x, vrep.simx_opmode_oneshot_wait)
    err, res_y = vrep.simxGetObjectIntParameter(
        clientID, sensorHandle, vrep.sim_visionintparam_resolution_y, vrep.simx_opmode_oneshot_wait)
    return  nearClip,farClip,angle,(res_x,res_y)