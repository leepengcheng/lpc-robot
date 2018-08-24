#coding:utf-8
import b0
import pyrealsense2 as rs
import numpy as np
import time


pc = rs.pointcloud()
points = rs.points()
pipe = rs.pipeline()
pipe.start()

node = b0.Node('pointcloud-pub')
pub = b0.Publisher(node, 'pc')
node.init()
print('Publishing to topic "%s"...' % pub.get_topic_name())

try:
    while not node.shutdown_requested():
        frames = pipe.wait_for_frames()

        depth = frames.get_depth_frame()
        # color = frames.get_color_frame()
        # pc.map_to(color)

        points = pc.calculate(depth)

        pts=[pt for pt in np.asanyarray(points.get_vertices()) if any(pt)]
        pub.publish(np.array(pts).tobytes())
        # time.sleep(0.1)
        # print("sending point cloud")

finally:
    pipe.stop()
    node.cleanup()

