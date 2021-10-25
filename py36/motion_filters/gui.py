import os
import sys
from collections import deque

if sys.platform == "win32":
    sys.path.append("C:\\opt\\ros\\melodic\\x64\\lib\\site-packages")
from collections import deque

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np
import pyqtgraph as pg
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose

NUM_TRACKER = 6

class MeshCatViz:
    def __init__(self, viz_type, skel_type):
        self.skel_type = skel_type
        self.viz_type = viz_type
        self.vis = meshcat.Visualizer()
        self.url = self.vis.url()
        self.tfs = dict()
        self.colors = ["0xfc1403", "0x034efc"]  # Red, Blue
        self.bias_y = 1.5  # TODO change to widget
        self.vertices = dict()

        # setting subscriber
        self.set_subscribe()

        # init viz
        if self.viz_type == 0 or self.viz_type == 1:
            self.init_viz(self.viz_type)

    def set_subscribe(self):
        rospy.Subscriber("/HMDViz", Pose, self.hmd_callback)
        for i in range(NUM_TRACKER):
            rospy.Subscriber("/TRACKERViz"+str(i), Pose, self.tracker_callback)

    def hmd_callback(self, msg):
        topic_name = msg._connection_header["topic"][1:]
        T = np.eye(4)
        T[:3, 3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        T[:3, :3] = R.from_quat(np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])).as_matrix()
        self.tfs[topic_name] = T


    def tracker_callback(self, msg):
        topic_name = msg._connection_header["topic"][1:]
        if self.viz_type==0:
            T = np.eye(4)
            T[:3, 3] = np.array([msg.position.x, msg.position.y, msg.position.z])
            T[:3, :3] = R.from_quat(np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])).as_matrix()
            self.tfs[topic_name] = T


    def init_viz(self, viz_type):
        asset_dir = os.path.dirname(os.path.abspath(__file__))
        self.vis["HMDViz"].set_object(
            g.Box([0.2, 0.1, 0.1]), g.MeshLambertMaterial(color=self.colors[viz_type], reflectivity=0.8)
        )
        self.tfs["HMDViz"] = np.eye(4)
        for i in range(NUM_TRACKER):
            self.vis["TRACKERViz"+str(i)].set_object(
                g.ObjMeshGeometry.from_file(os.path.join(asset_dir, "../assets/Vive_Tracker_meter.obj")),
                g.MeshLambertMaterial(color=self.colors[viz_type], reflectivity=0.8, opacity=0.4),
            )
            T = np.eye(4)
            T[:3, 3] = np.random.rand(3)
            self.tfs["TRACKERViz"+str(i)] = T

    def update(self):
        # print("update")
        self.set_transform(self.viz_type)
 

    def set_transform(self, viz_type):
        size = 0.02
        linewidth = 20
        self.vis["HMDViz"].set_transform(self.tfs["HMDViz"])
        for i in range(NUM_TRACKER):
            self.vis["TRACKERViz"+str(i)].set_transform(self.tfs["TRACKERViz"+str(i)])

            

class MainGUI(QMainWindow):
    def __init__(self, meshcat_viz_type, vis_hz, plot_buffer, plot_hz, meshcat_skel_type):
        QMainWindow.__init__(self)
        self.meshcat_viz_type = meshcat_viz_type
        self.meshcat_skel_type = meshcat_skel_type
        self.vis_hz = vis_hz
        self.plot_buffer = plot_buffer
        self.plot_hz = plot_hz

        self.init_ui()
        self.set_timer()


    def init_ui(self):
        # layout = QVBoxLayout()
        layout = QGridLayout()

        # web browser widget
        self.meshcat_viz = MeshCatViz(self.meshcat_viz_type, self.meshcat_skel_type)
        url = self.meshcat_viz.url
        web = QWebEngineView()  # browser in qtgui
        web.load(QUrl(url))

        # slider widget for graph index
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 6)
        slider.setTickInterval(1)
        slider.setTickPosition(QSlider.TicksAbove)
        layout.addWidget(slider, 0, 1)

        slider1 = QSlider(Qt.Horizontal)
        slider1.setRange(0, 1)
        slider1.setTickInterval(1)
        slider1.setTickPosition(QSlider.TicksAbove)
        slider1.valueChanged.connect(self.viz_changed)
        layout.addWidget(slider1, 0, 0)

        main_widget = QTabWidget()
        main_widget.addTab(web, "MeshCatViz")
        layout.addWidget(main_widget, 1, 0, 2, 2)

        cwg = QWidget()
        cwg.setLayout(layout)

        self.setCentralWidget(cwg)

    def viz_changed(self, value):
        if value != self.meshcat_viz.viz_type:
            print(f"viz id changes to {value}")
            self.meshcat_viz.init_tracker_viz(value)
            self.meshcat_viz.viz_type = value

            # self.plot_widgets.reset_poses()
            # self.plot_widgets.id = value

    def set_timer(self):
        meshcat_timer = QTimer(self)
        meshcat_timer.setInterval(int(1 / self.vis_hz * 1000.0))
        meshcat_timer.timeout.connect(self.meshcat_viz.update)
        meshcat_timer.start()
