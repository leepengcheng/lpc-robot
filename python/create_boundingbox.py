#coding:utf-8
import sys
from lxml.etree import ElementTree,Element
from lxml import etree
import numpy as np
from tf.transformations import  compose_matrix,decompose_matrix,quaternion_from_euler


configfile="/media/zen/workspace/GitWorkSpace/vrep-control/lua/config.lua"
urdffile="/opt/ros/kinetic/share/ur_description/urdf/ur5_robot_bhand.urdf"
robot_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/robot.txt"
hand_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/hand.txt"

# configfile,urdffile=sys.argv[1:3]
exec(open(configfile,'rt').read().replace("--","##").replace("{","[").replace("}","]")) #载入配置文件的参数


xml = ElementTree()
xml.parse(urdffile)



matrix=[]
qmatrix=[]
axises=[]
#获取关节参数
for jname in roboJointNames:
    jointNode=xml.xpath("//joint[@name='%s']"%jname)[0]
    originNode=jointNode.find("origin") 
    axisNode=jointNode.find("axis")
    origin_xyz=[0.0,0.0,0.0] if originNode is None else  map(float,originNode.attrib['xyz'].split())
    origin_rpy=[0.0,0.0,0.0] if originNode is None else  map(float,originNode.attrib['rpy'].split())
    axis_xyz=[0.0,0.0,1.0] if axisNode is None else  map(float,axisNode.attrib['xyz'].split())
    mat=compose_matrix(angles=origin_rpy,translate=origin_xyz)
    if len(matrix):
        matrix.append(matrix[-1].dot(mat))
    else:
        matrix.append(mat)
    scale,shear,angles,translate,perspective=decompose_matrix(matrix[-1])
    qmatrix.append([translate.round(8),quaternion_from_euler(*angles).round(8)])
    axises.append(axis_xyz)






num=0
robot_data=open(robot_out,'rt').readlines()
for lname in roboLinkNames:
    data=map(float,robot_data[num].split())
    collisionNode=xml.xpath("//link[@name='%s']/collision"%lname)[0]
    ###############originNode
    originNode=collisionNode.find("origin") #添加originNode
    if not originNode:
        originNode=etree.SubElement(collisionNode,"origin")
    if axises[num]=="0.0 1.0 0.0":
        originNode.attrib['xyz']="%s %s %s"%(data[0],data[2],-data[1])
        originNode.attrib['rpy']="%s %s %s"%(data[3],data[5],-data[4])
    else:
        originNode.attrib['xyz']="%s %s %s"%(tuple(data[0:3]))
        originNode.attrib['rpy']="%s %s %s"%(tuple(data[3:6]))
    ###########geometryNode
    geometryNode=collisionNode.find("geometry")
    meshNode=geometryNode.find("mesh") 
    if meshNode is not None:
       geometryNode.remove(meshNode) #删除meshnode
    boxNode=geometryNode.find("box") #添加boxnode
    if boxNode is None:
        boxNode=etree.SubElement(geometryNode,"box")
    boxNode.attrib['size']="%s %s %s"%(tuple(data[6:9]))
    xml.write("myrobot.urdf", xml_declaration=True,
           encoding="utf-8", pretty_print=True)
    num=num+1