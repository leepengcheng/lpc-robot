# coding:utf-8
import pickle
import time
import turtle as t
from lxml import etree
import numpy as np

# 解析SVG
ns = {'ns': "http://www.w3.org/2000/svg"}
doc = etree.parse("/media/zen/workspace/vrep/li.svg")
root_node = doc.getroot()
path_node = root_node.xpath("//ns:path", namespaces=ns)
if not len(path_node):
    print("no svg path was find!")
pathStr = path_node[0].attrib['d']


def getPath(pathStr):
    '''解析路径SVG'''
    path_pts = []
    path = pathStr.strip().split()
    nextpt_isstart = False
    # 初始位置
    last_pt = np.array([0, 0])
    for i in range(len(path)):
        p = path[i]
        # 当前为m时,下1个点为区域起点
        if p == "m":
            nextpt_isstart = True
            path_pts.append([])
            continue
        # 当前为z时,闭合区域起点
        if p == "z":
            t.pendown()
            # 加入区域起始点
            path_pts[-1].append(path_pts[-1][0])
            continue
        # 分割后转换为float型坐标
        p = [float(x) for x in p.split(",")]
        # 与前坐标累加(坐标是相对值)
        p = last_pt + p
        # 如果当前点时起始点,则记录区域起始点的坐标
        # 到区域起始点这一段不需要画直线,所以penup54
        if nextpt_isstart:
            nextpt_isstart = False
        # 保留上次的坐标
        last_pt = p
        # 加入区域起始点
        path_pts[-1].append(last_pt)
    return path_pts


def drawPath(pathlist,scale=100):
    '''绘制路径'''
    for path in pathlist:
        for i in range(len(path)):
            if i == 0:
                t.penup()
            else:
                t.pendown()
            t.goto(path[i][0]*scale,-path[i][1]*scale)
    t.mainloop()

def resizePath(pathlist,start,width,height):
    '''将SVG路径映射到指定范围'''
    if not isinstance(start,np.ndarray):
        start=np.array(start)
    pts=np.array(pathlist).all()
    min_val=np.min(pts,0)
    max_val=np.max(pts,0)
    scope=max_val-min_val
    x_scale=width/scope[0]
    y_scale=height/scope[1]
    scale=min((x_scale,y_scale))
    zero_pt=pathlist[0][0]
    pathlist=[(path-zero_pt)*scale+start for path in pathlist]
    # print(np.max(np.array(pathlist).all(),0))
    # print(np.min(np.array(pathlist).all(),0))
    return pathlist
        
    



pathlist = getPath(pathStr)
pathlist=resizePath(pathlist,[0.01,0.01],0.5,0.5)
drawPath(pathlist)
# time.sleep(1)
# 保存路径点为pickle
with open("path.pkl", "wb") as output:
    pickle.dump(pathlist, output)
    print("write path to pickle done!")

# 保存路径点为txt
with open("path.txt", "wt") as output:
    for pts in pathlist:
        for pt in pts:
            output.write("%s %s " % (pt[0], pt[1]))
        output.write("\n")
    print("write path to txt done!")

# 读入点
with open("path.pkl", "rb") as input:
    pathlist = pickle.load(input)

