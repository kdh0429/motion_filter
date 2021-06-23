import numpy as np
import os
import time
import sys
if sys.platform=="win32":
    sys.path.append('C:\\opt\\ros\\melodic\\x64\\lib\\site-packages')
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import rospy
# from VR.msg import matrix_3_4
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
print("imported")
tfs = dict()

# prefix = "F"
prefix = "R"

def tracker_callback(msg):
    topic_name = msg._connection_header["topic"][1:]
    posquat = np.array(msg.data, dtype=np.float64)
    # print(posquat)
    # T_raw = np.vstack([np.array(msg.firstRow),
    #                     np.array(msg.secondRow),
    #                     np.array(msg.thirdRow),
    #                     np.array([0 , 0, 0, 1])])
    # tfs[topic_name] = T_raw
    T = np.eye(4)
    T[:3, 3] = posquat[:3]
    T[:3, :3] = R.from_quat(posquat[3:]).as_matrix()
    # print(topic_name, T)
    tfs[topic_name] = T

rospy.init_node('raw_tracker_node')

for i in range(7):
    rospy.Subscriber(f"/{prefix}posquat{i}", Float64MultiArray, tracker_callback)

asset_dir = os.path.dirname(os.path.abspath(__file__))
# Create a new visualizer
# port = np.random.randint(6000, 7000)
vis = meshcat.Visualizer()
vis.open()
vis.url()
print("NEW")
time.sleep(1) #wait for server start

# setting viz assets
vis[f"{prefix}posquat6"].set_object(g.Box([0.2, 0.1, 0.1]))
tfs[f"{prefix}posquat6"] = np.eye(4)
for i in range(6):
    vis[f"{prefix}posquat{i}"].set_object(g.ObjMeshGeometry.from_file(os.path.join(asset_dir, "assets/Vive_Tracker_meter.obj")))
    tfs[f"{prefix}posquat{i}"] = np.eye(4)

flag = rospy.get_param("/mp/viz_flag")
while flag :
# max_time = int(1e+4)
# for i in range(max_time):
    vis[f"{prefix}posquat6"].set_transform(tfs[f"{prefix}posquat6"])
    for i in range(6):
        vis[f"{prefix}posquat{i}"].set_transform(tfs[f"{prefix}posquat{i}"])
    flag = rospy.get_param("/mp/viz_flag")
    time.sleep(0.01) 