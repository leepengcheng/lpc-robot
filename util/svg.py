# coding:utf-8
import pickle
import time
import turtle as t
from lxml import etree
import numpy as np

# 解析SVG
# ns = {'ns': "http://www.w3.org/2000/svg"}
# doc = etree.parse("/media/zen/workspace/vrep/li.svg")
# root_node = doc.getroot()
# path_node = root_node.xpath("//ns:path", namespaces=ns)
# if not len(path_node):
#     print("no svg path was find!")
# pathStr = path_node[0].attrib['d']


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


def drawPath(pathlist, scale=100):
    '''绘制路径'''
    for path in pathlist:
        for i in range(len(path)):
            if i == 0:
                t.penup()
            else:
                t.pendown()
            t.goto(path[i][0] * scale, -path[i][1] * scale)
    t.mainloop()


def resizePath(pathlist, start, width, height):
    '''将SVG路径映射到指定范围'''
    if not isinstance(start, np.ndarray):
        start = np.array(start)
    pts = np.array(pathlist).all()
    min_val = np.min(pts, 0)
    max_val = np.max(pts, 0)
    scope = max_val - min_val
    x_scale = width / scope[0]
    y_scale = height / scope[1]
    scale = min((x_scale, y_scale))
    zero_pt = pathlist[0][0]
    pathlist = [(path - zero_pt) * scale + start for path in pathlist]
    # print(np.max(np.array(pathlist).all(),0))
    # print(np.min(np.array(pathlist).all(),0))
    return pathlist


# pathlist = getPath(pathStr)
# pathlist=resizePath(pathlist,[0.01,0.01],0.5,0.5)
# drawPath(pathlist)
# # time.sleep(1)
# # 保存路径点为pickle
# with open("path.pkl", "wb") as output:
#     pickle.dump(pathlist, output)
#     print("write path to pickle done!")

# # 保存路径点为txt
# with open("path.txt", "wt") as output:
#     for pts in pathlist:
#         for pt in pts:
#             output.write("%s %s " % (pt[0], pt[1]))
#         output.write("\n")
#     print("write path to txt done!")

# # 读入点
# with open("path.pkl", "rb") as input:
#     pathlist = pickle.load(input)

data = '''
F M0 4 L0 4 L2 3 L14 0 L29 0 L51 0 L72 0 L88 0 L100 4 L103 5 L105 7 L106 8 L106 10 L106 14 L106 19 L106 25 L105 33 L100 39 L95 44 L89 50 L83 55 L80 56 L77 58 L78 58 L82 58 L91 58 L102 58 L129 72 L142 84 L153 95 L157 99 L159 103 L159 107 L159 111 L159 117 L158 124 L153 132 L148 140 L133 152 L122 161 L110 164 L99 166 L92 166 L84 167 L75 167 L66 167 L52 165 L45 165 L35 166 L31 166 L28 166 L26 166 L25 166 L24 165 L24 164
'''

P=data.split()[1:]

size=len(P)

pts=[]
for i in range(size):
    # if P[i].startswith("M"):
    #     pts.append((map(int,[P[i][1:],P[i+1]])))
    if P[i].startswith("L"):
        pts.append((map(int,[P[i][1:],P[i+1]])))
drawPath([pts],1)
print(len(data.split()))