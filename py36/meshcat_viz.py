import argparse
import os
import sys
import time

import numpy as np

if sys.platform == "win32":
    sys.path.append("C:\\opt\\ros\\melodic\\x64\\lib\\site-packages")
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import rospy
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

# from VR.msg import matrix_3_4
from std_msgs.msg import Float64MultiArray

print("imported")
tfs = dict()

parser = argparse.ArgumentParser()
parser.add_argument("type", type=int, help="type 0: raw only, type 1: filter only, type 2: both")
parser.add_argument("--bias", type=float, help="if type 2, bias will move filters to y direction.", default=0.0)
args = parser.parse_args()


prefixes = ["R", "F"]
colors = ["0xfc1403", "0x034efc"]  # Red, Blue
bias_y = args.bias

flag = True


def on_press(key):
    try:
        print("alphanumeric key {0} pressed".format(key.char))
    except AttributeError:
        print("special key {0} pressed".format(key))


def on_release(key):
    global flag
    print("{0} released".format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        flag = False


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()


def tracker_callback(msg):
    topic_name = msg._connection_header["topic"][1:]
    posquat = np.array(msg.data, dtype=np.float64)
    T = np.eye(4)
    T[:3, 3] = posquat[:3]

    if topic_name[0] == "F":
        T[1, 3] += bias_y
    T[:3, :3] = R.from_quat(posquat[3:]).as_matrix()

    tfs[topic_name] = T


def init_tracker_viz(vis, tfs, type):
    vis[f"{prefixes[type]}posquat6"].set_object(
        g.Box([0.2, 0.1, 0.1]), g.MeshLambertMaterial(color=colors[type], reflectivity=0.8)
    )
    tfs[f"{prefixes[type]}posquat6"] = np.eye(4)
    for i in range(6):
        vis[f"{prefixes[type]}posquat{i}"].set_object(
            g.ObjMeshGeometry.from_file(os.path.join(asset_dir, "assets/Vive_Tracker_meter.obj")),
            g.MeshLambertMaterial(color=colors[type], reflectivity=0.8),
        )
        tfs[f"{prefixes[type]}posquat{i}"] = np.eye(4)


def set_transform(vis, tfs, type):
    vis[f"{prefixes[type]}posquat6"].set_transform(tfs[f"{prefixes[type]}posquat6"])
    for i in range(6):
        vis[f"{prefixes[type]}posquat{i}"].set_transform(tfs[f"{prefixes[type]}posquat{i}"])


if __name__ == "__main__":
    start = time.time()
    rospy.init_node("raw_tracker_node")
    # Subscribe Setting
    for i in range(7):
        if args.type == 0 or args.type == 1:
            rospy.Subscriber(f"/{prefixes[args.type]}posquat{i}", Float64MultiArray, tracker_callback)
        elif args.type == 2:
            for j in range(args.type):
                rospy.Subscriber(f"/{prefixes[j]}posquat{i}", Float64MultiArray, tracker_callback)
        else:
            raise NotImplementedError("Unknown type")

    asset_dir = os.path.dirname(os.path.abspath(__file__))

    # Create a new visualizer
    vis = meshcat.Visualizer()
    vis.open()
    vis.url()
    print(f"Setting Time: {time.time() - start:0.4f}s")
    time.sleep(0.1)  # wait for server start
    start = time.time()

    # setting viz assets
    if args.type == 0 or args.type == 1:
        init_tracker_viz(vis, tfs, args.type)
    elif args.type == 2:
        for i in range(args.type):
            init_tracker_viz(vis, tfs, i)

    print(f"Asset Setting Time: {time.time() - start:0.4f}s")

    while True:
        # for i in range(int(1e+3)):
        start = time.time()
        if args.type == 0 or args.type == 1:
            set_transform(vis, tfs, args.type)
        elif args.type == 2:
            for i in range(args.type):
                set_transform(vis, tfs, i)

        if not flag:
            sys.exit()

        time.sleep(0.01)
        print(f"Process Time: {1/(time.time() - start):0.0f}hz")
