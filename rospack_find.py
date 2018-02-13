#coding:utf-8
from yaml import load
from subprocess import Popen,PIPE
PACKAGE_NAME="pr2_moveit_config"

#find the ros package path
PACKAGE_PATH=Popen("rospack find %s"%PACKAGE_NAME,shell=True,stdin=PIPE,stdout=PIPE).stdout.read().strip()
if "Error" in PACKAGE_PATH:
    raise NameError(PACKAGE_PATH)

sensor_dict=load(open("%s/config/sensors_kinect_depthmap.yaml"%PACKAGE_PATH).read())
