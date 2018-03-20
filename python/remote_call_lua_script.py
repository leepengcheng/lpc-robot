# coding:utf-8
import vrep
import sys
import ctypes
import random
from lxml.etree import ElementTree,Element
from lxml import etree
import numpy as np
from tf.transformations import  compose_matrix,decompose_matrix,quaternion_from_euler,euler_from_quaternion
from collections import OrderedDict

# 原始的urdf文件
urdf_file="/opt/ros/kinetic/share/ur_description/urdf/ur5_robot_bhand.urdf"
robotXmlString='''
<joints name="arm">
    <joint name="world_joint">
        <joint name="shoulder_pan_joint">
            <joint name="shoulder_lift_joint">
                <joint name="elbow_joint">
                    <joint name="wrist_1_joint">
                        <joint name="wrist_2_joint">
                            <joint name="wrist_3_joint">
                                <joint name="ee_fixed_joint">
                                    <joint name="bh_base_joint">
                                        <joint name="bh_j11_joint">
                                            <joint name="bh_j12_joint">
                                                <joint name="bh_j13_joint" />
                                            </joint>
                                        </joint>
                                        <joint name="bh_j21_joint">
                                            <joint name="bh_j22_joint">
                                                <joint name="bh_j23_joint" />
                                            </joint>
                                        </joint>
                                        <joint name="bh_j31_joint">
                                            <joint name="bh_j32_joint">
                                                <joint name="bh_j33_joint" />
                                            </joint>
                                        </joint>
                                    </joint>
                                </joint>
                            </joint>
                        </joint>
                    </joint>
                </joint>
            </joint>
        </joint>
    </joint>
</joints>
'''
armLinkNames = ["base_link",
                 "shoulder_link",
                 "upper_arm_link",
                 "forearm_link",
                 "wrist_1_link",
                 "wrist_2_link",
                 "wrist_3_link",
                 "ee_link"]
# 机械手连杆名称
handLinkNames = ["bh_base_link",
                 "bh_finger_11_link",
                 "bh_finger_12_link",
                 "bh_finger_13_link",
                 "bh_finger_21_link",
                 "bh_finger_22_link",
                 "bh_finger_23_link",
                 "bh_finger_31_link",
                 "bh_finger_32_link",
                 "bh_finger_33_link"]

def parseUrdfJoint(xml,jointNames,jonintTree):
    '''获取每个关节的实际位姿'''
    matrix=OrderedDict()
    result=OrderedDict()
    qmatrix=OrderedDict()
    # axises=[]
    #获取关节参数
    for jname in jointNames:
        jointNode=xml.xpath("//joint[@name='%s']"%jname)[0]
        originNode=jointNode.find("origin") 
        axisNode=jointNode.find("axis")
        origin_xyz=[0.0,0.0,0.0] if originNode is None else  map(float,originNode.attrib['xyz'].split())
        origin_rpy=[0.0,0.0,0.0] if originNode is None else  map(float,originNode.attrib['rpy'].split())
        # axis_xyz=[0.0,0.0,1.0] if axisNode is None else  map(float,axisNode.attrib['xyz'].split())
        mat=compose_matrix(angles=origin_rpy,translate=origin_xyz)
        pname=getParentJointName(jonintTree,jname) #获取当前关节的父关节名称
        if pname is None:
            matrix[jname]=mat
        else:
            matrix[jname]=matrix[pname].dot(mat)
        scale,shear,angles,translate,perspective=decompose_matrix(matrix[jname])
        result[jname]=[np.array(translate).round(8),np.array(angles).round(8)]
        qmatrix[jname]=[translate.round(8),quaternion_from_euler(*angles).round(8)]
        # axises.append(axis_xyz)
    return  result


def getParentJointName(xml,jointName):
    jnode=xml.xpath("//joint[@name='%s']"%jointName)
    if len(jnode)==0:
        raise NameError("joint does not exists")
    pnode=jnode[0].getparent()
    if pnode.tag=="joints":
        return None
    else:
        return pnode.attrib['name']
    



def parseUrdfLink(xml,linkNames,robot_data):
    num=0
    for lname in linkNames:
        print(lname)
        data=map(float,robot_data[num*10:(num+1)*10])
        collisionNode=xml.xpath("//link[@name='%s']/collision"%lname)[0]
        ###############originNode
        originNode=collisionNode.find("origin") #添加originNode
        if originNode is None:
            originNode=etree.SubElement(collisionNode,"origin")
        originNode.attrib['xyz']="%s %s %s"%(tuple(data[0:3]))
        ####从VREP获得的euler角是rxyz
        # q=quaternion_from_euler(*data[3:6],axes="rxyz")
        rpy_sxyz=euler_from_quaternion(data[3:7],axes="sxyz")
        originNode.attrib['rpy']="%s %s %s"%(tuple(rpy_sxyz)) 
        # originNode.attrib['rpy']="%s %s %s"%(tuple(data[3:6]))
        ###########geometryNode
        geometryNode=collisionNode.find("geometry")
        meshNode=geometryNode.find("mesh") 
        if meshNode is not None:
            geometryNode.remove(meshNode) #删除meshnode
        boxNode=geometryNode.find("box") #添加boxnode
        if boxNode is None:
            boxNode=etree.SubElement(geometryNode,"box")
        boxNode.attrib['size']="%s %s %s"%(tuple(data[7:10]))
        num=num+1
    return xml

urdfTree = ElementTree()
urdfTree.parse(urdf_file)
robotTree=etree.fromstring(robotXmlString) #机械臂本体

robotJointNames= [node.attrib['name'] for node in robotTree.xpath("//joint")]

robotLinkNames = armLinkNames+handLinkNames


jointPoses=parseUrdfJoint(urdfTree,robotJointNames,robotTree)

vrep.simxFinish(-1)  # 关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # 连接到VREP
if clientID != -1:
    print ('Connected to remote API server')
    
    # 关闭动力学
    vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_dynamics_handling_enabled,False,vrep.simx_opmode_blocking)
    
    virtualJHandles=[]
    for  jname,jpose in jointPoses.items():
        translate,angles=jpose
        res,dummy=vrep.simxCreateDummy(clientID,0.05,None,vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(clientID,dummy,-1,translate,vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(clientID,dummy,-1,angles,vrep.simx_opmode_blocking)
        virtualJHandles.append(dummy)


    # 1.显示消息并返回反馈
    emptyBuff = bytearray()
    # clientID,脚本依附对象名称,脚本类型,函数名称,输入int型,输入float型,输入string型,输入bytearray型(应当为空),阻赛模式
    # returnCode,输出int型,输出float型,输出string型,输出bytearray
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(
        clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'writeRobot', virtualJHandles, [], robotLinkNames, emptyBuff, vrep.simx_opmode_blocking)
    if res == vrep.simx_return_ok:
        retFloats=np.array(retFloats).round(6)
        urdfTree=parseUrdfLink(urdfTree,robotLinkNames,retFloats)
        print ('Generate new URDF')
        urdfTree.write("myrobot.urdf", xml_declaration=True,encoding="utf-8", pretty_print=True)
        
    else:
        print ('Remote function call failed')



    
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
    # point_cloud=[]
    # with open("/home/zen/apps/Matlab/toolbox/vision/visiondata/teapot.ply",'rt') as f:
    #     pts=f.readlines()[7:]
    #     pt=[float(x) for p in pts for x in p.strip().split()] #尺寸扩大10倍
    #     point_cloud.extend(pt)
    # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'insertPointCloud_function', [
    # ], point_cloud, ['myPointCloud'], emptyBuff, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print ('Point handle: ', retInts[0])
    #     pass
    # else:
    #     print ('Remote function call failed')

    # 5.清除点云
    # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'clearPointCloud_function', [
    # ], point_cloud, ['myPointCloud'], emptyBuff, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print ('Point handle: ', retInts[0])
    #     pass
    # else:
    #     print ('Remote function call failed')

    # 关闭所有连接
    vrep.simxFinish(clientID)
else:
    print ('无法连接到VREP')
print ('程序结束')
