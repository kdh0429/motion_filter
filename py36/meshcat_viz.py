import numpy as np
import os
import time

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import rospy
from VR.msg import matrix_3_4

tfs = dict()

prefix = "F"
# prefix = ""

def tracker_callback(msg):
    topic_name = msg._connection_header["topic"][1:]
    T_raw = np.vstack([np.array(msg.firstRow),
                        np.array(msg.secondRow),
                        np.array(msg.thirdRow),
                        np.array([0 , 0, 0, 1])])
    tfs[topic_name] = T_raw

rospy.init_node('raw_tracker_node')

for i in range(6):
    rospy.Subscriber(f"/{prefix}TRACKER{i}", matrix_3_4, tracker_callback)

rospy.Subscriber(f"/{prefix}HMD", matrix_3_4, tracker_callback)

asset_dir = os.path.dirname(os.path.abspath(__file__))
# Create a new visualizer
vis = meshcat.Visualizer()
vis.open()
vis.url()

time.sleep(1) #wait for server start

# setting viz assets
vis[f"{prefix}HMD"].set_object(g.Box([0.2, 0.1, 0.1]))
tfs[f"{prefix}HMD"] = np.eye(4)
for i in range(6):
    vis[f"{prefix}TRACKER{i}"].set_object(g.ObjMeshGeometry.from_file(os.path.join(asset_dir, "assets/Vive_Tracker_meter.obj")))
    # vis[f"TRACKER{i}"].set_transform(tf.scale_matrix(0.002))
    tfs[f"{prefix}TRACKER{i}"] = np.eye(4)

flag = rospy.get_param("/mp/viz_flag")
while flag :
    vis[f"{prefix}HMD"].set_transform(tfs[f"{prefix}HMD"])
    for i in range(6):
        vis[f"{prefix}TRACKER{i}"].set_transform(tfs[f"{prefix}TRACKER{i}"])
    flag = rospy.get_param("/mp/viz_flag")