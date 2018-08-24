#coding:utf-8
import b0
import numpy as np
from sensor_msgs.point_cloud2 import PointCloud2,PointField
import rospy
import tf


msg=PointCloud2()
msg.header.frame_id="cam"
msg.fields=[
    PointField('x',0,PointField.FLOAT32,1),
    PointField('y',4,PointField.FLOAT32,1),
    PointField('z',8,PointField.FLOAT32,1),
]
msg.height=1
msg.is_bigendian=False
msg.point_step=12
msg.is_dense=True

def callback(ptsbuffer):
    time_now=rospy.Time(0)
    buff_size=len(ptsbuffer)
    # pc=np.frombuffer(ptsbuffer,dtype=np.dtype("f4,f4,f4"))
    # msg.data=pc.view(dtype=np.float32).tolist()
    msg.header.stamp=time_now
    msg.width=buff_size/12
    msg.row_step=buff_size
    msg.data=ptsbuffer
    ros_tf_pub.sendTransform((0,0,1), (0,0,0,1), time_now, "cam", "world")
    ros_pub.publish(msg)
    
    


node_b0 = b0.Node('pointcloud_sub')
sub_b0 = b0.Subscriber(node_b0, 'pc', callback)
node_b0.init()
rospy.init_node("pointcloud_ros")
print('Subscribed to topic "%s"...' % sub_b0.get_topic_name())

ros_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=10)
ros_tf_pub = tf.TransformBroadcaster(queue_size=1)
try:
    while not node_b0.shutdown_requested() and not rospy.is_shutdown():
        # node_b0.spin()
        node_b0.spin_once()
finally:
    node_b0.cleanup()