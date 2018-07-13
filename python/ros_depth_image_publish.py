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



# 转发TF
def pubTransformTimeNow(cam_trans, cam_euler, time_now):
    for plink, slink in ROBOT_LINKS:
        trans, rot = tf_sub.lookupTransform(plink, slink, rospy.Time(0))
        tf_pub.sendTransform(trans, rot, time_now, slink, plink)
    tf_pub.sendTransform(cam_trans, tf.transformations.quaternion_from_euler(
        *cam_euler), time_now, "rgbd_camera_link", "world")


def genRosCameraInfo(K, P, resolution):
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
    caminfo.header.frame_id = "rgbd_camera_link"
    # caminfo.header.stamp = time_now
    caminfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # 畸变系数，无初始值需要指定
    caminfo.K = K
    caminfo.P = P
    return caminfo


def genRosDepthImage(resolution):
    rosImg = Image()
    # rosImg.data = np_img.flatten().tobytes()
    rosImg.header.frame_id = "rgbd_camera_link"
    # rosImg.header.stamp = time_now
    rosImg.width = resolution[0]  # 宽
    rosImg.height = resolution[1]  # 高
    rosImg.step = resolution[0] * 4
    rosImg.encoding = "32FC1"
    rosImg.is_bigendian = 0
    return rosImg


#UR5 本体LINK
UR5_LINKS = [("base_link", "shoulder_link"),
             ("shoulder_link", "upper_arm_link"),
             ("upper_arm_link", "forearm_link"),
             ("forearm_link", "wrist_1_link"),
             ("wrist_1_link", "wrist_2_link"),
             ("wrist_2_link", "wrist_3_link"),
             ("wrist_3_link","ee_link")]
#SEVEN_DOF 本体+爪子LINK
SEVEN_DOF_LINKS = [
    ('base_link', 'shoulder_pan_link'),
    ('shoulder_pan_link', 'shoulder_pitch_link'),
    ('shoulder_pitch_link', 'elbow_roll_link'),
    ('elbow_roll_link', 'elbow_pitch_link'),
    ('elbow_pitch_link', 'wrist_roll_link'),
    ('wrist_roll_link', 'wrist_pitch_link'),
    ('wrist_pitch_link','gripper_roll_link')]
#bhand机械手LINK
BHAND_LINKS = [
    ('bh_base_link', 'bh_finger_31_link'),
    ('bh_finger_31_link', 'bh_finger_32_link'),
    ('bh_finger_32_link', 'bh_finger_33_link'),
    ('bh_base_link', 'bh_finger_21_link'),
    ('bh_finger_21_link', 'bh_finger_22_link'),
    ('bh_finger_22_link', 'bh_finger_23_link'),
    ('bh_base_link', 'bh_finger_11_link'),
    ('bh_finger_11_link', 'bh_finger_12_link'),
    ('bh_finger_12_link', 'bh_finger_13_link')]

# ROBOT_LINKS = UR5_LINKS   #UR5
ROBOT_LINKS = UR5_LINKS + BHAND_LINKS #UR5+HAND
# ROBOT_LINKS =SEVEN_DOF_LINKS #SEVEN_DOF+HAND


VISION_SENSOR = 'kinect_depth#'  # 传感器名称
DISP_DEPTHIMG = False  # 是否显示图片


vrep.simxFinish(-1)  # 关闭所有连接
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 开启连接
# ros
rospy.init_node("rgbd_camera")
tf_pub = tf.TransformBroadcaster(queue_size=5)
tf_sub = tf.TransformListener()
depthimg_pub = rospy.Publisher(
    "/rgbd_camera/depth/image_raw", Image, queue_size=5)
caminfo_pub = rospy.Publisher(
    "/rgbd_camera/depth/camera_info", CameraInfo, queue_size=1, latch=True)


if clientID != -1:
    print 'Connected to remote API server'
    # 获得相机句柄
    res, sensorHandle = vrep.simxGetObjectHandle(
        clientID, VISION_SENSOR, vrep.simx_opmode_oneshot_wait)
    # 获得深度相机的近景/远景距离/视场角/分辨率
    nearClip, farClip, angle, resolution = getVirtualCamAdditionalMatrix(
        clientID, sensorHandle)
    # 计算相机内参
    cam_K, cam_P = getVirtualCamInternalMatrix(angle, resolution)
    # 相机的位置固定，只变化转角
    err, sensorPos = vrep.simxGetObjectPosition(
        clientID, sensorHandle, -1, vrep.simx_opmode_oneshot_wait)
    # 相机欧拉角(默认不动)
    err2, sensorEuler = vrep.simxGetObjectOrientation(
        clientID, sensorHandle, -1, vrep.simx_opmode_oneshot_wait)
    sensorEuler.append("rxyz")  # 欧拉角顺序xyz

    print("farclip:%s  nearclip:%s" % (farClip, nearClip))
    print("resolution:%s x %s" % (resolution[0], resolution[1]))
    print("perspective_radius:%s" % angle)
    # 发送流传送命令
    err, resolution_, image = vrep.simxGetVisionSensorDepthBuffer(
        clientID, sensorHandle, vrep.simx_opmode_streaming)
    caminfo_msg = genRosCameraInfo(cam_K, cam_P, resolution)
    rosimg_msg = genRosDepthImage(resolution)

    while (vrep.simxGetConnectionId(clientID) != -1 and not rospy.is_shutdown()):
        # 深度图
        err1, resolution_, img = vrep.simxGetVisionSensorDepthBuffer(
            clientID, sensorHandle, vrep.simx_opmode_buffer)
        if err1 == vrep.simx_return_ok:
            time_now = rospy.Time.now()  # 时间
            np_img = np.array(img, dtype=np.float32)
            np_img.resize([resolution[1], resolution[0]])
            np_img = np_img[:, ::-1]  # 垂直反转
            ros_img = nearClip + np_img * \
                (farClip - nearClip)  # 发送给ROS需要转换为实际距离
            # 发送深度图
            rosimg_msg.data = ros_img.flatten().tobytes()
            rosimg_msg.header.stamp = time_now
            depthimg_pub.publish(rosimg_msg)
            # 发送相机参数
            caminfo_msg.header.stamp = time_now
            caminfo_pub.publish(caminfo_msg)
            # 发送tf
            pubTransformTimeNow(sensorPos, sensorEuler, time_now)
            if DISP_DEPTHIMG:
                # img=cv2.flip(img,1) #水平反转图片
                np_img = cv2.pyrDown(np_img)  # 降采样
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
