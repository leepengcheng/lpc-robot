#coding:utf-8
import rospy
import tf
from sensor_msgs.msg import Image

def callback(data):
    print(data.is_bigendian)
    # trans,rot=listen.lookupTransform("camera", "odom_combined",rospy.Time.now())

rospy.init_node("image_sub")
listen=tf.TransformListener()
sub=rospy.Subscriber("/rgbd_camera/depth/image_raw",Image,callback)
rospy.spin()
# while not rospy.is_shutdown():
#       trans,rot=listen.lookupTransform("camera", "odom_combined",rospy.Time(0))
#         rospy.spinonce()          
