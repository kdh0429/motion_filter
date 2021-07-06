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
from std_msgs.msg import Float64MultiArray

NUM_TRACKER = 6


class MeshCatViz:
    def __init__(self, viz_type):
        self.viz_type = viz_type
        self.vis = meshcat.Visualizer()
        self.url = self.vis.url()
        self.tfs = dict()
        self.prefixes = ["R", "F"]
        self.colors = ["0xfc1403", "0x034efc"]  # Red, Blue
        self.bias_y = 1.5  # TODO change to widget
        self.vertices = dict()
        # setting subscriber
        self.set_subscribe()

        # init viz
        if self.viz_type == 0 or self.viz_type == 1:
            self.init_tracker_viz(self.viz_type)

    def set_subscribe(self):
        rospy.Subscriber("/skel", Float64MultiArray, self.skel_callback)
        for i in range(NUM_TRACKER + 1):
            for j in range(2):
                rospy.Subscriber(f"/{self.prefixes[j]}posquat{i}", Float64MultiArray, self.tracker_callback)

    def skel_callback(self, msg):
        data = np.array(msg.data, dtype=np.float64).reshape(-1, 3).T
        # print(data.shape)
        self.vertices["spines"] = data[:, :3]
        self.vertices["larm"] = data[:, 3:8]
        self.vertices["rarm"] = data[:, 8:13]         

    def tracker_callback(self, msg):
        topic_name = msg._connection_header["topic"][1:]
        posquat = np.array(msg.data, dtype=np.float64)

        if topic_name[0] == "R" and self.viz_type==0:
            T = np.eye(4)
            T[:3, 3] = posquat[:3]
            T[:3, :3] = R.from_quat(posquat[3:]).as_matrix()
            self.tfs[topic_name] = T

        if topic_name[0] == "F" and self.viz_type==1:
            T = np.eye(4)
            T[:3, 3] = posquat[:3]
            T[1, 3] += self.bias_y
            T[:3, :3] = R.from_quat(posquat[3:]).as_matrix()
            self.tfs[topic_name] = T


    def init_tracker_viz(self, viz_type):
        asset_dir = os.path.dirname(os.path.abspath(__file__))
        self.vis[f"{self.prefixes[viz_type]}posquat6"].set_object(
            g.Box([0.2, 0.1, 0.1]), g.MeshLambertMaterial(color=self.colors[viz_type], reflectivity=0.8)
        )
        self.tfs[f"{self.prefixes[viz_type]}posquat6"] = np.eye(4)
        for i in range(NUM_TRACKER):
            self.vis[f"{self.prefixes[viz_type]}posquat{i}"].set_object(
                g.ObjMeshGeometry.from_file(os.path.join(asset_dir, "../assets/Vive_Tracker_meter.obj")),
                g.MeshLambertMaterial(color=self.colors[viz_type], reflectivity=0.8),
            )
            T = np.eye(4)
            T[:3, 3] = np.random.rand(3)
            self.tfs[f"{self.prefixes[viz_type]}posquat{i}"] = T

    def update(self):
        # print("update")
        self.set_transform(self.viz_type)
 

    def set_transform(self, viz_type):
        size = 0.02
        for i in range(NUM_TRACKER + 1):
            self.vis[f"{self.prefixes[viz_type]}posquat{i}"].set_transform(self.tfs[f"{self.prefixes[viz_type]}posquat{i}"])
        if len(self.vertices.keys()) >= 3:
            self.vis["spines"].set_object(g.Line(g.PointsGeometry(self.vertices["spines"]), g.LineBasicMaterial(color="0x034efc", linewidth=10 )))
            self.vis["spines_point"].set_object(g.PointsGeometry(self.vertices["spines"]), g.PointsMaterial(size=size))

            self.vis["larm"].set_object(g.Line(g.PointsGeometry(self.vertices["larm"]), g.LineBasicMaterial(color="0x034efc", linewidth=10 )))
            self.vis["larm_point"].set_object(g.PointsGeometry(self.vertices["larm"]), g.PointsMaterial(size=size))

            self.vis["rarm"].set_object(g.Line(g.PointsGeometry(self.vertices["rarm"]), g.LineBasicMaterial(color="0x034efc", linewidth=10)))
            self.vis["rarm_point"].set_object(g.PointsGeometry(self.vertices["rarm"]), g.PointsMaterial(size=size))

class PlotGUI:
    def __init__(self, max_buffer, plot_hz):
        # do something
        self.keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        self.id = 0
        self.pws = [pg.PlotWidget(name=f"self.keys[i]") for i in range(7)]  # 6dof pose into 7 variable
        self.max_buffer = max_buffer
        self.plot_hz = plot_hz
        self.rrecent = np.zeros(7)
        self.frecent = np.zeros(7)
        self.set_plot_widget()
        self.reset_poses()
        self.set_subscribe()

    def update(self):
        self.rposes.append(self.rrecent)
        self.fposes.append(self.frecent)
        
        rtmp = np.vstack(self.rposes).T  # (7, num_data)
        ftmp = np.vstack(self.fposes).T  # (7, num_data)

        rx = np.arange(rtmp.shape[1])/float(self.plot_hz)
        fx = np.arange(ftmp.shape[1])/float(self.plot_hz)

        for idx in range(7):
            self.rcurves[idx].setData(x=rx, y=rtmp[idx])
            self.fcurves[idx].setData(x=fx, y=ftmp[idx])

    def reset_poses(self):
        self.rposes = deque([np.zeros(7)], maxlen=self.max_buffer)
        self.fposes = deque([np.zeros(7)], maxlen=self.max_buffer)

    def set_plot_widget(self):
        self.rcurves = []  # raw
        self.fcurves = []  # filter

        for pw in self.pws:
            # pw.enableAutoRange()
            pw.setXRange(0, 2)
            pw.setYRange(-2, 2)
            pw.setLabel("bottom", "time", "s")
            pw.setLabel("left", "pose", "m")
            pw.setBackground("w")
            self.rcurves.append(pw.plot(pen="r"))
            self.fcurves.append(pw.plot(pen="b"))

    def set_subscribe(self):
        for i in range(NUM_TRACKER + 1):
            rospy.Subscriber(f"/Rposquat{i}", Float64MultiArray, self.tracker_callback)
            rospy.Subscriber(f"/Fposquat{i}", Float64MultiArray, self.tracker_callback)

    def tracker_callback(self, msg):
        topic_name = msg._connection_header["topic"][1:]
        posquat = np.array(msg.data, dtype=np.float64)

        if topic_name[0] == "R" and topic_name[-1] == str(self.id):
            self.rrecent = posquat

        if topic_name[0] == "F" and topic_name[-1] == str(self.id):
            self.frecent = posquat


class MainGUI(QMainWindow):
    def __init__(self, meshcat_viz_type, vis_hz, plot_buffer, plot_hz):
        QMainWindow.__init__(self)
        self.meshcat_viz_type = meshcat_viz_type
        self.vis_hz = vis_hz
        self.plot_buffer = plot_buffer
        self.plot_hz = plot_hz

        self.init_ui()
        self.set_timer()

    def init_ui(self):
        # layout = QVBoxLayout()
        layout = QGridLayout()

        # web browser widget
        self.meshcat_viz = MeshCatViz(self.meshcat_viz_type)
        url = self.meshcat_viz.url
        web = QWebEngineView()  # browser in qtgui
        web.load(QUrl(url))

        # slider widget for graph index
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 6)
        slider.setTickInterval(1)
        slider.setTickPosition(QSlider.TicksAbove)
        slider.valueChanged.connect(self.plotid_changed)
        layout.addWidget(slider, 0, 1)

        slider1 = QSlider(Qt.Horizontal)
        slider1.setRange(0, 1)
        slider1.setTickInterval(1)
        slider1.setTickPosition(QSlider.TicksAbove)
        slider1.valueChanged.connect(self.viz_changed)
        layout.addWidget(slider1, 0, 0)

        # plot widget
        self.plot_widgets = PlotGUI(self.plot_buffer, self.plot_hz)
        main_widget = QTabWidget()

        main_widget.addTab(web, "MeshCatViz")

        for idx, pw in enumerate(self.plot_widgets.pws):
            main_widget.addTab(pw, self.plot_widgets.keys[idx])

        layout.addWidget(main_widget, 1, 0, 2, 2)

        cwg = QWidget()
        cwg.setLayout(layout)

        self.setCentralWidget(cwg)

    def plotid_changed(self, value):
        if value != self.plot_widgets.id:
            print(f"plot id changes to {value}")
            self.plot_widgets.reset_poses()
            self.plot_widgets.id = value
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

        plot_timer = QTimer(self)
        plot_timer.setInterval(int(1 / self.plot_hz * 1000.0))
        plot_timer.timeout.connect(self.plot_widgets.update)
        plot_timer.start()