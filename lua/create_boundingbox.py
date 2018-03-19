#coding:utf-8
import sys
from lxml.etree import ElementTree,Element
from lxml import etree
import numpy as np


configfile="/media/zen/workspace/GitWorkSpace/vrep-control/lua/config.lua"
urdffile="/opt/ros/kinetic/share/ur_description/urdf/ur5_robot_bhand.urdf"
robot_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/robot.txt"
hand_out="/media/zen/workspace/GitWorkSpace/vrep-control/lua/hand.txt"

# configfile,urdffile=sys.argv[1:3]
exec(open(configfile,'rt').read().replace("--","##").replace("{","[").replace("}","]")) #载入配置文件的参数


xml = ElementTree()
xml.parse(urdffile)
robot_data=open(robot_out,'rt').readlines()
num=0
for lname in roboLinkNames:
    data=robot_data[num].split()
    lname=lname.replace("_visual","") #去除后缀(vrep自动添加的)
    collisionNode=xml.xpath("//link[@name='%s']/collision"%lname)[0]
    ###############originNode
    originNode=collisionNode.find("origin") #添加originNode
    if not originNode:
        originNode=etree.SubElement(collisionNode,"origin")
    originNode.attrib['xyz']=" ".join(data[0:3])
    originNode.attrib['rpy']=" ".join(data[3:6])
    ###########geometryNode
    geometryNode=collisionNode.find("geometry")
    meshNode=geometryNode.find("mesh") 
    if meshNode is not None:
       geometryNode.remove(meshNode) #删除meshnode
    boxNode=geometryNode.find("box") #添加boxnode
    if boxNode is None:
        boxNode=etree.SubElement(geometryNode,"box")
    boxNode.attrib['size']=" ".join(data[6:9])
    xml.write("myrobot.urdf", xml_declaration=True,
           encoding="utf-8", pretty_print=True)
    num=num+1