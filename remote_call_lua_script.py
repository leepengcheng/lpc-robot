# coding:utf-8


import vrep

import sys
import ctypes
import  random


vrep.simxFinish(-1)  # 关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # 连接到VREP
if clientID != -1:
    print ('Connected to remote API server')

    # 1.显示消息并返回反馈
    emptyBuff = bytearray()
    # clientID,脚本依附对象名称,脚本类型,函数名称,输入int型,输入float型,输入string型,输入bytearray型(应当为空),阻赛模式
    # returnCode,输出int型,输出float型,输出string型,输出bytearray
    # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(
    #     clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'displayText_function', [], [], ['Hello world!'], emptyBuff, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print ('Return string: ', retStrings[0])
    # else:
    #     print ('Remote function call failed')

    # # 2:远程创建对象
    # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'createDummy_function', [
    # ], [0.1, 0.2, 0.3], ['MyDummyName'], emptyBuff, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print ('Dummy handle: ', retInts[0])
    # else:
    #     print ('Remote function call failed')

    # # 3.直接发送函数体到客户端执行
    # code = "local octreeHandle=simCreateOctree(0.5,0,1)\n" \
    #     "simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})\n" \
    #     "return 'done'"
    # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(
    #     clientID, "remoteApiCommandServer", vrep.sim_scripttype_childscript, 'executeCode_function', [], [], [code], emptyBuff, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print ('Code execution returned: ', retStrings[0])
    # else:
    #     print ('远程执行函数失败')

    # 4.创建点云
    # point_cloud=[random.random()*5-2.5 for x in range(3000)] #1000个随机点
    point_cloud=[]
    with open("/home/zen/apps/Matlab/toolbox/vision/visiondata/teapot.ply",'rt') as f:
        pts=f.readlines()[7:]
        pt=[float(x) for p in pts for x in p.strip().split()] #尺寸扩大10倍
        point_cloud.extend(pt)
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'insertPointCloud_function', [
    ], point_cloud, ['myPointCloud'], emptyBuff, vrep.simx_opmode_blocking)
    if res == vrep.simx_return_ok:
        print ('Point handle: ', retInts[0])
        pass
    else:
        print ('Remote function call failed')
        
    # 5.清除点云
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'clearPointCloud_function', [
    ], point_cloud, ['myPointCloud'], emptyBuff, vrep.simx_opmode_blocking)
    if res == vrep.simx_return_ok:
        print ('Point handle: ', retInts[0])
        pass
    else:
        print ('Remote function call failed')

    # 关闭所有连接
    vrep.simxFinish(clientID)
else:
    print ('无法连接到VREP')
print ('程序结束')
