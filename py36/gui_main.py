from motion_filters.gui import MainGUI
from PyQt5.QtWidgets import QApplication
import sys
if sys.platform == "win32":
    sys.path.append("C:\\opt\\ros\\melodic\\x64\\lib\\site-packages")
import rospy
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--type", type=int, default=0, help="type 0: raw only, type 1: filter only")
parser.add_argument("--viz_hz", type=int, default=10, help="meshcat viz update hz")
parser.add_argument("--max_buffer", type=int, default=200, help="pyqtgraph max buffer size")
parser.add_argument("--plot_hz", type=int, default=100, help="pyqtgraph update hz")


args = parser.parse_args()

if __name__ == "__main__":
    rospy.init_node("motion_filter_gui_node")
    app = QApplication(sys.argv)
    form = MainGUI(args.type, args.viz_hz, args.max_buffer, args.plot_hz)
    form.show()
    app.exec_()
