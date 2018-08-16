#coding:utf-8
from __future__ import print_function
import pyrealsense2 as rs
import cv2
import numpy as np
from math import tan, radians
import time
np.set_printoptions(precision=4, suppress=True)

#控制量:是否使用VREP
USE_VREP = False
SHOW_MARKER=True
SHOW_AXIS=True


def getVirtualCamInternalMatrix(angle, resolution, flatten=True):
    '''获取相机的内参和投影矩阵
    @angle为视场角的弧度
    @flatten:为true时转换为1维list
    '''
    res = resolution
    cx = cy = res[0] * 0.5
    # 方法1:间接求
    # f = 0.008  # 相机焦距:单位mm(虚拟焦距)
    # dx = dy = tan(angle / 2) * f / (cx * 0.5)  # 单个像素大小
    # fx = f / dx #焦距的像素长度
    # fy = f / dy
    # 方法2:直接求
    fx = fy = cx / np.tan(angle / 2)
    # Tx = Ty = 1
    K = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)  # 相机内参
    P = np.array(
        [[fx, 0, cx, 0], [0, fy, cy, 0], [0, 0, 1, 0]],
        dtype=np.float64)  # 投影矩阵
    if not flatten:
        return K
    return map(lambda x: x.flatten().tolist(), (K, P))


def genMarker(index, size=200):
    "生成marker"
    maker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker = cv2.aruco.drawMarker(maker_dict, index, size, 1)
    return marker, maker_dict


#简单滤波
def filterMatrix(rots, trans, ids):
    # global faceEulers,facePostions
    ids_str = ["c%s" % x for x in ids if x in faceIds]
    n = len(ids_str)
    eulerOrders = []
    posOrders = []
    for i in range(n):
        name = ids_str[i]
        eulerOrders.append(np.linalg.norm(rots[i] - faceEulers[name]))
        posOrders.append(np.linalg.norm(trans[i] - facePostions[name]))
        faceEulers[name] = rots[i]
        facePostions[name] = trans[i]
    index1, index2 = np.argmin(eulerOrders), np.argmin(posOrders)
    if eulerOrders[index1] <= eulerStepRange[0] or eulerOrders[index1] >= eulerStepRange[1]:
        return None, None, None
    else:
        return rots[index1], trans[index1], ids[index1]


def getMarkerPoseImage(img, maker_dict, camInternalMat, camDistortMat):
    '''在图片中标注marker的位姿'''
    image = np.copy(img)
    corners, ids, rejectpts = cv2.aruco.detectMarkers(image, maker_dict)
    # image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
    rvecs, tvecs, objpts = cv2.aruco.estimatePoseSingleMarkers(
        corners, 0.1, camInternalMat, camDistortMat)
    rotMats, tranMats = [], []
    if rvecs is not None:
        for x in range(len(rvecs)):
            #返回的是旋转向量
            #将旋转向量转换为旋转矩阵
            rotMats.append(cv2.Rodrigues(rvecs[x].flatten())[0])
            tranMats.append(tvecs[x].flatten())
    if ids is None:
        ids = np.array([])
    return image, rotMats, tranMats, ids, (rvecs, tvecs, corners)


maker_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_6X6_250)  #aruco 字典
####################未进行标定，参数瞎写的##############################################
angle = np.deg2rad(60)  #市场角
resolution = (640, 480)  #相机分辨率
camInternalMat = getVirtualCamInternalMatrix(angle, resolution, False)  #相机内参
camDistortMat = np.array([0, 0, 0, 0, 0], dtype=np.float64)  #畸变系数
faceIds = [10, 16, 32, 36, 38, 42]
faceNames = ["c%s" % x for x in faceIds]
################################################################################

#vrep 连接初始化
if USE_VREP:
    import vrep
    from vrepUtil import getVirtualCamInternalMatrix
    import tf
    cap = cv2.VideoCapture(0)
    emptyBuff = bytearray()
    vrep.simxFinish(-1)  # 关闭所有连接
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 开启连接
    if clientID == -1:
        raise NameError("Could not connect Vrep")
    res, sensorHandle = vrep.simxGetObjectHandle(clientID, 'usbCam',
                                                 vrep.simx_opmode_oneshot_wait)
    res, box1Handle = vrep.simxGetObjectHandle(clientID, 'aruco_box_1',
                                               vrep.simx_opmode_oneshot_wait)
    eulerStepRange = (0.05, 1.0)
    postionThrehold = 0.1
    faceEulers = {}
    facePostions = {}
    # 初始化
    for name in faceNames:
        faceEulers[name] = np.zeros((3, 3))
        facePostions[name] = np.zeros(3)
    while vrep.simxGetConnectionId(clientID) != -1:
        status, img = cap.read()
        if status:
            img, rots, trans, ids, _ = getMarkerPoseImage(
                img, maker_dict, camInternalMat, camDistortMat)
            ids = ids.flatten()
            if len(ids):
                R, T, ID_ = filterMatrix(rots, trans, ids)
                if R is None:
                    continue
                data = []
                for i in range(3):
                    for j in range(4):
                        if j == 3:
                            data.append(T[i])
                        else:
                            data.append(R[i, j])

                vrep.simxCallScriptFunction(
                    clientID, 'remoteApiCommandServer',
                    vrep.sim_scripttype_childscript, 'setPosition', [], data,
                    ['c%s' % ID_], emptyBuff, vrep.simx_opmode_oneshot)
            # for m in range(len(rots)):
            #     rot = rots[m]
            # ori_w = tf.transformations.euler_from_matrix(rot, 'rxyz')
            #     pos_w = trans[m]
            cv2.imshow("Video", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Image Capture Error")
            break
else:
    #配置RGB和深度图数据流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, resolution[0], resolution[1],
                         rs.format.z16, 30)
    config.enable_stream(rs.stream.color, resolution[0], resolution[1],
                         rs.format.bgr8, 30)

    #开始传输
    pipeline.start(config)
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            #获取深度图和RGB图
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 转换为numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            #检测marker
            color_image, rots, trans, ids, params = getMarkerPoseImage(
                color_image, maker_dict, camInternalMat, camDistortMat)
            rvecs, tvecs, corners=params
            if SHOW_MARKER:
                color_image = cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            if SHOW_AXIS:
                for x in range(len(ids.flatten())):
                    color_image=cv2.aruco.drawAxis(color_image, camInternalMat, camDistortMat, rvecs[x],
                                    tvecs[x], 0.1)
            # 深度图转换为RGB热力图
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.01), cv2.COLORMAP_JET)

            # 水平拼接
            images = np.hstack((color_image, depth_colormap))
            cv2.imshow('RealSense', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
