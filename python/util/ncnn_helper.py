#coding:utf-8
import os
import cv2
from subprocess import Popen,PIPE
from lxml.etree import ElementTree
cmd="/media/lee/workspace/RepoUbuntu/ncnn-master/build/examples/squeezenet '%s'"
labels=open("/media/lee/workspace/RepoUbuntu/ncnn-master/examples/synset_words.txt").readlines()
labels=[l[9:].strip() for l in labels]
labelsDict={}
for x in range(len(labels)):
    labelsDict[str(x)]=labels[x]


IMAGE_PATH="/media/lee/workspace/Data/SlamData/data/VOCdevkit_Train/VOC2007"
ANNO_PATH=os.path.join(IMAGE_PATH,"Annotations")
JEPG_PATH=os.path.join(IMAGE_PATH,"JPEGImages")
RATIO=0.4

imageDict={}
tree=ElementTree()
for x in os.listdir(ANNO_PATH):
    f=os.path.join(ANNO_PATH,x)
    xml=tree.parse(f)
    jpg=os.path.join(JEPG_PATH,xml.xpath("//filename")[0].text)
    obj_names=[]
    for obj in xml.xpath("//object/name"):
        name=obj.text
        if not name in obj_names:
            obj_names.append(name)
    imageDict[jpg]=obj_names



for f,l in imageDict.items():
    img=cv2.imread(f)
    res=Popen(cmd%f,shell=True,stdin=PIPE,stdout=PIPE).stdout.read().strip().split()
    objnames="%s %s"%(labelsDict[res[0]],res[1])
    cv2.putText(img," ".join(l),(10,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1)
    #大于阈值才绘制
    if float(res[1])>=RATIO:
        num=2
        waittime=5000
        for objname in objnames.split(","):
            cv2.putText(img,objname,(10,num*50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),1)
            num+=1
    else:
        waittime=500
    cv2.imshow("",img)
    cv2.waitKey(waittime)
