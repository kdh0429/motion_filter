#sudo apt install libxcb-xineramo0

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QMessageBox, QStatusBar, QMainWindow, QLabel
from PyQt5.QtCore import QCoreApplication
if sys.platform == "win32":
    sys.path.append("C:\\opt\\ros\\melodic\\x64\\lib\\site-packages")
import rospy
from functools import partial

from std_msgs.msg import Int8

    
class MyApp(QWidget):

    def __init__(self):
        rospy.init_node("retarget_gui", anonymous=True)
        self.pub = rospy.Publisher(f"/CALIBMODE", Int8, queue_size=100)

        super().__init__()
        self.initUI()

    def initUI(self):
        self.statusbar = QStatusBar()
        # self.setStatusBar(self.statusbar)
        grid = QGridLayout()
        self.setLayout(grid)
        self.label = QLabel('STATUS', self)
        self.label.resize(300, 30)

        btn0 = QPushButton('save experiment', self)
        btn0.resize(btn0.sizeHint())
        btn0.clicked.connect(partial(self.set_param, "/retarget/save_flag", True))
        grid.addWidget(btn0, 0, 0)

        btn1 = QPushButton('initpose_flag', self)
        btn1.resize(btn1.sizeHint())
        btn1.clicked.connect(partial(self.set_param, "/retarget/initpose_flag", True))
        grid.addWidget(btn1, 1, 0)

        btn2 = QPushButton('control start', self)
        btn2.resize(btn2.sizeHint())
        btn2.clicked.connect(partial(self.set_param, "/retarget/control_flag", True))
        grid.addWidget(btn2, 2, 0)


        btn3 = QPushButton('Attention(Still) Pose', self)
        btn3.resize(btn3.sizeHint())
        btn3.clicked.connect(partial(self.set_mode, 1))
        grid.addWidget(btn3, 0, 1)

        btn4 = QPushButton('T Pose', self)
        btn4.resize(btn4.sizeHint())
        btn4.clicked.connect(partial(self.set_mode, 2))
        grid.addWidget(btn4, 1, 1)

        btn5 = QPushButton('Forward Dress Pose', self)
        btn5.resize(btn5.sizeHint())
        btn5.clicked.connect(partial(self.set_mode, 3))
        grid.addWidget(btn5, 2, 1)

        btn6 = QPushButton('Reset Calib Mode', self)
        btn6.resize(btn6.sizeHint())
        btn6.clicked.connect(partial(self.set_mode, 4))
        grid.addWidget(btn6, 0, 2)

        btn7 = QPushButton('Set Waiting Mode', self)
        btn7.resize(btn7.sizeHint())
        btn7.clicked.connect(partial(self.set_mode, 5))
        grid.addWidget(btn7, 1, 2)

        btn8 = QPushButton('control stop', self)
        btn8.resize(btn8.sizeHint())
        btn8.clicked.connect(partial(self.set_param, "/retarget/control_flag", False))
        grid.addWidget(btn8, 2, 2)
        # btn1 = QPushButton('save experiment', self)
        # btn1.resize(btn.sizeHint())
        # btn1.clicked.connect(partial(self.set_param, "/retarget/still_pose_flag_", True))
        # grid.addWidget(btn1, 1, 0)



        self.setWindowTitle('Simple retarget gui')
        self.setGeometry(300, 300, 300, 200)
        self.show()

    def set_param(self, key, value):
        rospy.set_param(key, value)
        self.label.setText(f'set {key} to {value}')

    def set_mode(self, int_mode):
        msg = Int8()
        msg.data = int_mode
        self.pub.publish(msg)
        self.label.setText(f'set Pose Calib Mode to {int_mode}')

if __name__ == '__main__':
    
   app = QApplication(sys.argv)
   ex = MyApp()
   sys.exit(app.exec_())