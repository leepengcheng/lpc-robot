# coding:utf-8
from lxml.etree import ElementTree
from lxml import etree
import numpy as np

JTypeMapping={
    "translational":"planar",
    "revolute":"continuous"
}


class Link():
    def __init__(self, name, center, mass, inertia, markersID):
        self.name = name
        self.center = center
        self.mass = mass
        self.inertia = inertia
        self.markersID = markersID
        self.xyz="0.0 0.0 0.0"


    def getInertiaMarker(self,markers):
        '''返回惯量的参考矩阵'''
        for mID  in self.markersID:
            marker=markers[mID]
            if marker.isrefID:
                return marker
        return None
    

    def setOriginXYZ(self,xyz):
        self.xyz=xyz
    


class Joint():
    def __init__(self, jname, jtype, markersID, jdata):
        self.name = jname
        self.jtype = jtype
        self.markersID = markersID
        # 暂时认为2个运动副同心,选取第1个zu
        self.jdata = jdata[0].strip().split()


    @property
    def axis(self):
        #旋转轴的向量
        return " ".join(self.jdata[3:6])

    @property
    def isFixed(self):
        return self.jtype=="fixed"
    
    def getMarkers(self,markers):
        "返回运动副所有的Marker"
        markerList=[]
        for mID in self.markersID:
            markerList.append(markers[mID])
        return markerList

    def getLinks(self,markers,links):
        "返回运动副的所有连杆"
        linksName=self.getLinksName(markers)
        return [links[name] for name in linksName]

    def getLinksName(self,markers):
        "返回运动副的所有连杆的名称"
        markerList=self.getMarkers(markers)
        return [m.linkName for m in markerList]

    def setLinkXYZ(self,markers,links,num=0):
        "设置运动副的连杆的初始位置"
        marker=self.getMarkers(markers)[num]
        link=self.getLinks(markers,links)[num]
        #取负号
        xyz=" ".join([str(-float(x)) for x in marker.transformXYZ.split()])
        link.setOriginXYZ(xyz)

    def getSortedLinksName(self,markers,linkTree):
        "连杆排序 父子"
        lname1,lname2=self.getLinksName(markers)
        lnode1=linkTree.xpath("//%s"%lname1)[0]
        childNode=lnode1.getchildren()
        for node in childNode:
            if node.tag==lname2:
                return linksName
        return [lname2,lname1]

    # def getAttachJointMarker(self,markers,linkTree):
    #     sortedLinksName=self.getSortedLinksName(self,markers,linkTree)
    #     plinkName=sortedLinksName[0]
   
    
    # def getjointsMarkersTransform(self):
    #     transform=Marker.getMarkersRelativeTransform()

class Marker():

    def __init__(self,markerID,isrefID,linkName,transform):
        self.markerID=markerID
        self.isrefID=isrefID
        self.linkName=linkName
        self.matrix=self.getTransform(transform)

    @property
    def transformXYZ(self):
        "变换矩阵的平移部分的字符串"
        xyz=self.matrix[:3,3].reshape([3,])
        return str(xyz).replace("[","").replace("]","").strip()

    def transformTo(self,marker):
        '''转换到另外的marker的变换矩阵'''
        return marker.dot(self.matrix.I)

    @staticmethod
    def getTransform(trans):
        '''将字符串转换为矩阵'''
        trans=[float(x) for x in trans.strip().split()]
        return np.mat(trans).reshape(4,4).T

    
    @staticmethod
    def getMarkersRelativeTransform(marker1,marker2):
        '''求2个字符串类型矩阵的变换矩阵'''
        mat1,mat2=map(lambda x:x.matrix,[marker1,marker2])
        trans=mat2.dot(mat1.I)
        return trans

    @staticmethod
    def  getTransformXYZ(matrix):
        '''变换矩阵的平移部分'''
        return matrix[:3,:3]


class Util():
    @staticmethod
    def getLinkJointTree(base,joints):
        "连杆及运动副的树形结构"
        baseLink=etree.Element(base)
        lastLinks =[baseLink]
        while len(joints):
            _links=[]
            _joints=[]
            for l in lastLinks: 
                for j in joints.keys():
                    lnames=joints[j]
                    if l.tag in lnames:
                        if not "joint" in l.attrib.keys():
                            l.attrib['joint']=""
                        if not j in l.attrib['joint']:
                            l.attrib['joint']+=" "+j
                        # l.attrib['joint']=j
                        if l.tag==j[0]:
                            _links.append(etree.SubElement(l,lnames[1]))
                        else:
                            _links.append(etree.SubElement(l,lnames[0]))
                        _joints.append(j)
            lastLinks=_links
            for j in _joints:
                joints.pop(j)
        return baseLink
    
    @staticmethod
    def  getParentJointName(jointName,linkJointTree):
        link=linksTree.xpath("//*[contains(@joint,'%s')]"%jointName)[0]
        plink=link.getparent()
        if plink is not None:
            #父运动副
            pjoint=plink.attrib['joint'].strip()
            return pjoint
        return None



xml = ElementTree()
xml.parse(r"E:\Work\NX\aaa.xml")
root = xml.getroot()
ns = {'ns': "http://www.plmxml.org/Schemas/PLMXMLSchema"}



#解析连杆
link_nodes = root.xpath("//ns:MechanismRevisionView", namespaces=ns)
links = {}       #连杆对象字典
markers={}       #marker点对象字典
markerLinks = {} #marker点关联的连杆名称
for node in link_nodes:
    if node.attrib['subType'] == "link":
        # 名称
        name = node.attrib['name']
        # 质心
        center = node.xpath(".//ns:CentreOfMass",
                            namespaces=ns)[0].attrib['value']
        # 质量
        mass = node.xpath(".//ns:ValueWithUnit",
                          namespaces=ns)[0].attrib['value']
        # 惯量、惯量参考坐标系
        inertia_node = node.xpath(".//ns:MechanismInertia", namespaces=ns)[0]
        inertia, inertiaID = inertia_node.attrib['value'], inertia_node.attrib['markerRef']
        marker_nodes = node.xpath(".//ns:Marker", namespaces=ns)
        #连杆关联的mark点信息字典
        markersID=[]
        for node in marker_nodes:
            mID=node.attrib['id']
            transform=node.find("ns:Transform", ns).text
            isrefID=True if mID==inertiaID else False
            #收集marker点
            markers[mID]=Marker(mID,isrefID,name,transform)
            #制定marker id对于的连杆名称
            markerLinks[mID]=name
            markersID.append(mID)
        links[name] = Link(name, center, mass, inertia, markersID)

#解析运动副
joint_nodes = root.xpath("//ns:ConstraintInstance", namespaces=ns)
joints = {}
for node in joint_nodes:
    jname = node.attrib['name']
    jtype = node.find("ns:JointData", ns).attrib['type']
    #运动副参考的Mark点
    jmarkersID = [ref.attrib['targetRef'].replace(
        "#", "") for ref in node.findall("ns:ConstraintTargetRef", ns)]
    #运动副的旋转向量
    jdata = [val.attrib['value']
                  for val in node.xpath(".//ns:UserValue", namespaces=ns)]
    #运动副关联的连杆名称
    joint=Joint(jname, jtype,jmarkersID, jdata)
    #设置运动副对应的连杆的起始位置
    joint.setLinkXYZ(markers,links)
    joints[jname] = joint


#基座名称
fixedLinkName=""
jointsDict={}
for joint in  joints.values():
    linksName=joint.getLinksName(markers)
    if len(linksName)==1:
        #基座
        fixedLinkName=linksName[0]
    else:
        jointsDict.update({joint.name:linksName})  

#连杆的树形结构
linksTree=Util.getLinkJointTree(fixedLinkName,jointsDict)


###############################生成urdf
robotNode = etree.Element("robot")
robotNode.attrib['name'] = "robot"
for link in links.values():
    linkNode = etree.SubElement(robotNode, "link")
    linkNode.attrib['name'] = link.name
    visualNode = etree.SubElement(linkNode, "visual")
    geometryNode = etree.SubElement(visualNode, "geometry")
    #stl文件位置
    meshNode = etree.SubElement(geometryNode, "mesh")
    meshNode.attrib['filename'] = "package://%s.stl" % link.name
    #比例系数vrep为m
    meshNode.attrib['scale'] = "0.001 0.001 0.001"
    #连杆的起始位置:相对于父Joint(WTF!)
    if link.name!=fixedLinkName:
        originNode=etree.SubElement(visualNode,"origin")
        originNode.attrib['xyz']=link.xyz
        # originNode.attrib['rpy']="0 0 0"





for joint in joints.values():
    if joint.jtype == "fixed":
        continue 
    jointNode = etree.SubElement(robotNode, "joint")
    jointNode.attrib['name'] = joint.name
    jointNode.attrib['type'] = JTypeMapping[joint.jtype]

    #获得运动副排序后的连杆名称
    sortedLinksName=joint.getSortedLinksName(markers,linksTree)
    parentJointname=Util.getParentJointName(joint.name,linksTree)
    if parentJointname is None:
        xyz=joint.getMarkers(markers)[0].transformXYZ
    else:
        marker1,marker2=map(lambda x:joints[x].getMarkers(markers)[0],[parentJointname,joint.name])
        transform=Marker.getMarkersRelativeTransform(marker1,marker2)
        xyz=transform[:3,3].reshape([3,])
        xyz=str(xyz).replace("[","").replace("]","").strip()
    parentNode = etree.SubElement(jointNode, "parent")
    parentNode.attrib['link'] = sortedLinksName[0]
    childNode = etree.SubElement(jointNode, "child")
    childNode.attrib['link'] = sortedLinksName[1]
    originNode = etree.SubElement(jointNode, "origin")
    originNode.attrib['xyz'] = xyz
    # originNode.attrib['rpy'] = joint.rpy
    axisNode = etree.SubElement(jointNode, "axis")
    #运动副的旋转轴
    axisNode.attrib['xyz'] = joint.axis


urdf = etree.ElementTree(robotNode)
urdf.write("myrobot.urdf", xml_declaration=True,
           encoding="utf-8", pretty_print=True)

print("finished!")

