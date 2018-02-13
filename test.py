#coding:utf-8
import rospy
import tf


links=[("upper_arm_link","forearm_link"),
("shoulder_link","upper_arm_link"),
("base_link","shoulder_link"),
("forearm_link","wrist_1_link"),
("wrist_1_link","wrist_2_link"),
("wrist_2_link","wrist_3_link")]

# ros
rospy.init_node("rgbd_camera")
tf_sub = tf.TransformListener()
tf_pub=tf.TransformBroadcaster()
rospy.sleep(rospy.Duration(1))
for plink,slink in links:
    trans,rot=tf_sub.lookupTransform(plink,slink,rospy.Time(0))
    tf_pub.sendTransform(trans,rot,rospy.Time.now(),slink,plink)




