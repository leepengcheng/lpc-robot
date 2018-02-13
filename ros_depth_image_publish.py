# coding:utf-8
'''
发布tf转换
'''
import vrep
import time
import cv2
import numpy as np
from math import *
# ros package
import rospy
import tf
from sensor_msgs.msg import CameraInfo, Image
import time



def getCameraMatrix(angle,resolution):
    "angle为视场角的弧度"
    res =resolution
    cx = cy = res[0] * 0.5  
    #方法1:间接求
    # f = 0.008  # 相机焦距:单位mm(虚拟焦距)
    # dx = dy = tan(angle / 2) * f / (cx * 0.5)  # 单个像素大小
    # fx = f / dx #焦距的像素长度
    # fy = f / dy
    #方法2:直接求
    fx=fy=(cx * 0.5)/tan(angle / 2) 
    Tx = Ty = 1
    K = np.array([[fx, 0, cx],
                [0, fy, cy],
                [0, 0,   1]])  # 相机内参
    P = np.array([[fx, 0, cx, 0],
                [0, fy, cy, 0],
                [0, 0,   1, 0]]) # 投影矩阵
    return map(lambda x:x.flatten().tolist(),(K,P))


def pubTransformTimeNow(cam_trans,cam_euler,time_now):
    for plink,slink in UR5_LINKS:
        trans,rot=tf_sub.lookupTransform(plink,slink,rospy.Time(0))
        tf_pub.sendTransform(trans,rot,time_now,slink,plink)
    cam_euler.append("rxyz")
    tf_pub.sendTransform(cam_trans,tf.transformations.quaternion_from_euler(*cam_euler),time_now,"camera","world")

def genRosCameraInfo(K,P,resolution, time_now):
    '''
    默认参数
    header:
        seq: 0
        stamp:
            secs: 0
            nsecs:         0
    frame_id: ''
    height: 0
    width: 0
    distortion_model: ''
    D: []
    K: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    P: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    binning_x: 0
    binning_y: 0
    roi:
    x_offset: 0
    y_offset: 0
    height: 0
    width: 0
    do_rectify: False
    '''
    caminfo = CameraInfo()
    caminfo.width = resolution[0]
    caminfo.height = resolution[1]
    caminfo.distortion_model = "plumb_bob"
    caminfo.header.frame_id = "camera"
    caminfo.header.stamp = time_now
    caminfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # 畸变系数，无初始值需要指定
    caminfo.K = K
    caminfo.P = P
    return caminfo


def genRosDepthImage(np_img, resolution, time_now):
    rosImg = Image()
    rosImg.data = np_img.flatten().tobytes()
    rosImg.header.frame_id = "camera"
    rosImg.header.stamp = time_now
    rosImg.width = resolution[0]  # 宽
    rosImg.height = resolution[1]  # 高
    rosImg.step = resolution[0] * 4
    rosImg.encoding = "32FC1"
    rosImg.is_bigendian = 0
    return rosImg





UR5_LINKS=[("upper_arm_link","forearm_link"),
("shoulder_link","upper_arm_link"),
("base_link","shoulder_link"),
("forearm_link","wrist_1_link"),
("wrist_1_link","wrist_2_link"),
("wrist_2_link","wrist_3_link")]
VISION_SENSOR = 'Vision_sensor#'  # 传感器名称
DISP_DEPTHIMG = True # 是否显示图片


vrep.simxFinish(-1)  # 关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 开启连接
# ros
rospy.init_node("rgbd_camera")
tf_pub = tf.TransformBroadcaster(queue_size=10)
tf_sub = tf.TransformListener()
depthimg_pub = rospy.Publisher(
    "/rgbd_camera/depth/image_raw", Image, queue_size=5)
caminfo_pub = rospy.Publisher(
    "/rgbd_camera/depth/camera_info", CameraInfo, queue_size=1, latch=True)


if clientID != -1:
    print 'Connected to remote API server'
    # 获得相机句柄
    res, sensorHandle = vrep.simxGetObjectHandle(clientID, VISION_SENSOR, vrep.simx_opmode_oneshot_wait)
    # 获得深度相机的近景/远景距离/视场角/分辨率
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
    resolution=(res_x,res_y)
    cam_K,cam_P=getCameraMatrix(angle,resolution) #相机内参
    # 相机的位置固定，只变化转角
    err, sensorPos = vrep.simxGetObjectPosition(clientID, sensorHandle, -1, vrep.simx_opmode_oneshot_wait)
    print("farclip:%s  nearclip:%s" % (farClip, nearClip))
    print("resolution:%s x %s" % (res_x, res_y))
    print("perspective_radius:%s"%angle)
    # 发送流传送命令
    err, resolution_, image = vrep.simxGetVisionSensorDepthBuffer(
        clientID, sensorHandle, vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1 and not rospy.is_shutdown()):
        # 深度图
        err1, resolution_, img = vrep.simxGetVisionSensorDepthBuffer(
            clientID, sensorHandle, vrep.simx_opmode_buffer)
        # 相机欧拉角
        err2, sensorEuler = vrep.simxGetObjectOrientation(
            clientID, sensorHandle, -1, vrep.simx_opmode_oneshot_wait)
        if err1 == vrep.simx_return_ok:
            time_now = rospy.Time.now()  # 时间
            np_img = np.array(img, dtype=np.float32)
            np_img.resize([resolution[1], resolution[0]])
            np_img = np_img[:,::-1]# 垂直反转
            #发送深度图
            depthimg_pub.publish(genRosDepthImage(np_img, resolution, time_now))
            #发送相机参数
            caminfo_pub.publish(genRosCameraInfo(cam_K,cam_P,resolution,time_now))
            # 发送tf
            pubTransformTimeNow(sensorPos,sensorEuler,time_now)
            if DISP_DEPTHIMG:
                # img=cv2.flip(img,1) #水平反转图片
                cv2.imshow('Depth image', np_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        elif err == vrep.simx_return_novalue_flag:
            # print "No image yet"
            pass
        else:
            print "Error code: %d" % err
            break
else:
    print "Failed to connect to remote API Server"
    vrep.simxFinish(clientID)
cv2.destroyAllWindows()
