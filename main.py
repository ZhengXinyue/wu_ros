#!/usr/bin/python3
# coding=utf-8
import os
import sys

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import QVBoxLayout, QMessageBox, QHBoxLayout, QApplication, QWidget, QLabel, QPlainTextEdit, \
    QTextEdit, QMainWindow, QPushButton, QDialog
from PyQt5 import QtGui, QtWidgets, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, qDebug, QSettings, QVariant, Qt, QObject, QPoint
from PyQt5.QtGui import QIcon, QPixmap

from ui_example import Ui_MainWindow
from ros_node import UAVNode


class MyMainWindow(QMainWindow):
    def __init__(self):
        super(MyMainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.setWindowTitle('UAV Software')
        self.setWindowIcon(QIcon('icon.png'))
        self.read_settings()
        self.uav_node = UAVNode()
        self.uav_node.start()

        self.figure = plt.figure(facecolor='lightgrey')
        self.canvas = FigureCanvas(self.figure)
        self.ui.canvas_layout.addWidget(self.canvas)
        self.uav_figure = self.figure.add_subplot(111)
        self.initialize_canvas()
        plt.connect('button_press_event', self.canvas_mouse_event)

        self.disable_others()
        self.ui.start_btn.clicked.connect(self.enable_others)
        self.ui.stop_btn.clicked.connect(self.disable_others)
        self.ui.unlock_btn.clicked.connect(self.uav_node.idle)
        self.ui.takeoff_btn.clicked.connect(self.uav_node.takeoff)
        self.ui.landing_btn.clicked.connect(self.uav_node.land)
        self.ui.hold_btn.clicked.connect(self.uav_node.hold)
        self.ui.send_pos_btn.clicked.connect(self.uav_node.publish_pos)
        self.ui.change_formation_btn.clicked.connect(lambda: self.uav_node.publish_formation(self.ui.formation.currentIndex(), self.ui.control_mode.currentIndex()))
        self.ui.formation_size.textEdited.connect(lambda: self.uav_node.change_formation_size(self.ui.formation_size.text()))
        self.marker_count = 1

        self.uav_node.formation_size = int(self.ui.formation_size.text())

    def initialize_canvas(self):
        self.uav_figure.set_title('Position')
        self.uav_figure.grid()
        self.uav_figure.set_xlim(-1000, 1000)
        self.uav_figure.set_ylim(-1000, 1000)

    def canvas_mouse_event(self, event):
        x_data, y_data = event.xdata, event.ydata
        if x_data and y_data:
            self.uav_figure.scatter(x_data, y_data,  color='r', marker='*', s=50)
            self.uav_figure.text(x_data-10, y_data+20, '%d' % self.marker_count, fontsize=12)
            self.canvas.draw()
            self.marker_count += 1

            self.uav_node.virtual_leader_pos[0] = x_data
            self.uav_node.virtual_leader_pos[1] = y_data
            self.ui.x_pos.setText('x_pos: %.2f' % x_data)
            self.ui.y_pos.setText('y_pos: %.2f' % y_data)

    def clear_figure(self):
        self.uav_figure.cla()
        self.initialize_canvas()

    def disable_others(self):
        self.ui.unlock_btn.setEnabled(False)
        self.ui.takeoff_btn.setEnabled(False)
        self.ui.landing_btn.setEnabled(False)
        self.ui.hold_btn.setEnabled(False)
        self.ui.change_formation_btn.setEnabled(False)
        self.ui.formation.setEnabled(False)
        self.ui.stop_btn.setEnabled(False)
        self.ui.control_mode.setEnabled(False)
        self.ui.send_pos_btn.setEnabled(False)
        self.ui.formation_size.setEnabled(False)

        self.ui.start_btn.setEnabled(True)

    def enable_others(self):
        self.ui.unlock_btn.setEnabled(True)
        self.ui.takeoff_btn.setEnabled(True)
        self.ui.landing_btn.setEnabled(True)
        self.ui.hold_btn.setEnabled(True)
        self.ui.change_formation_btn.setEnabled(True)
        self.ui.formation.setEnabled(True)
        self.ui.stop_btn.setEnabled(True)
        self.ui.control_mode.setEnabled(True)
        self.ui.send_pos_btn.setEnabled(True)
        self.ui.formation_size.setEnabled(True)

        self.ui.start_btn.setEnabled(False)

    def closeEvent(self, a0):
        self.uav_node.stop_flag = True
        # finish the ros thread
        self.uav_node.quit()
        self.uav_node.wait()
        self.write_settings()

    def write_settings(self):
        settings = QSettings('config', 'uav_software')
        # window
        settings.setValue('geometry', self.saveGeometry())
        settings.setValue('windowState', self.saveState())

    def read_settings(self):
        settings = QSettings('config', 'uav_software')
        # window
        if settings.value('windowState') is not None:
            self.restoreState(settings.value('windowState'))
        if settings.value('geometry') is not None:
            self.restoreGeometry(settings.value('geometry'))


if __name__ == '__main__':
    # 工作目录改为catkin_wu/src/talker/src
    os.chdir('src/talker/src')
    app = QApplication(sys.argv)

    w = MyMainWindow()
    w.show()
    sys.exit(app.exec_())



