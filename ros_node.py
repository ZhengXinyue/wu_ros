import os

import numpy as np
import rospy

from PyQt5.QtCore import QThread, pyqtSignal

# from talker.msg import SwarmCommand
from my_swarm_command import SwarmCommand


class UAVNode(QThread):

    def __init__(self):
        super().__init__()
        node_name = 'uav_node'
        rospy.init_node(node_name, anonymous=True)
        self.stop_flag = False
        self.rate = rospy.Rate(1)

        self.swarm_num_uav = 1
        self.publishers = []
        self.messages = []

        self.control_mode = 0
        self.formation_size = 1
        self.formation_idx = 0
        self.virtual_leader_pos = [0, 0, 1]
        self.virtual_leader_vel = [0, 0, 0]
        self.virtual_leader_yaw = 0

    def stop_uav(self):
        self.publishers.clear()
        self.messages.clear()

    def start_uav(self):
        self.publishers.clear()
        self.messages.clear()
        for i in range(self.swarm_num_uav):
            publisher = rospy.Publisher('/uav%d' % (i + 1) + '/prometheus/swarm_command', SwarmCommand, queue_size=1)
            self.publishers.append(publisher)
            self.messages.append(SwarmCommand())

    def publish_once(self):
        for message, publisher in zip(self.messages, self.publishers):
            publisher.publish(message)

    def change_uav_num(self, uav_num):
        try:
            self.swarm_num_uav = int(uav_num)
        except ValueError:
            print('uav num should be a number')

    def change_z_value(self, z_value):
        try:
            self.virtual_leader_pos[2] = float(z_value)
        except ValueError:
            print('z value should be a number')

    def change_formation_size(self, formation_size):
        try:
            self.formation_size = int(formation_size)
        except ValueError:
            print('formation size should be a number')

    def change_formation(self, formation_idx):
        self.formation_idx = formation_idx

    def change_control_mode(self, control_mode):
        self.control_mode = control_mode

    def idle(self):
        """
        解锁
        :return:
        """
        for message in self.messages:
            message.Mode = SwarmCommand.Idle
            message.yaw_ref = 999
        self.publish_once()

    def takeoff(self):
        """
        起飞
        :return:
        """
        for message in self.messages:
            message.Mode = SwarmCommand.Takeoff
            message.yaw_ref = 0
        self.publish_once()

    def hold(self):
        for message in self.messages:
            message.Mode = SwarmCommand.Hold
        self.publish_once()

    def land(self):
        """
        降落
        :return:
        """
        for message in self.messages:
            message.Mode = SwarmCommand.Land
        self.publish_once()

    def disarm(self):
        """
        上锁
        :return:
        """
        for message in self.messages:
            message.Mode = SwarmCommand.Disarm
        self.publish_once()

    def publish_pos(self):
        for message in self.messages:
            message.swarm_size = self.formation_size
            message.position_ref[0] = self.virtual_leader_pos[0]
            message.position_ref[1] = self.virtual_leader_pos[1]
            message.position_ref[2] = self.virtual_leader_pos[2]
            message.velocity_ref[0] = self.virtual_leader_vel[0]
            message.velocity_ref[1] = self.virtual_leader_vel[1]
            message.velocity_ref[2] = self.virtual_leader_vel[2]
            message.yaw_ref = self.virtual_leader_yaw
        self.publish_once()

    def publish_formation(self):
        if self.formation_idx == 0:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.One_column
        elif self.formation_idx == 1:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Triangle
        elif self.formation_idx == 2:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Square
        elif self.formation_idx == 3:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Circular

        if self.control_mode == 0:
            for message in self.messages:
                message.Mode = SwarmCommand.Position_Control
        if self.control_mode == 1:
            for message in self.messages:
                message.Mode = SwarmCommand.Velocity_Control
        if self.control_mode == 2:
            for message in self.messages:
                message.Mode = SwarmCommand.Accel_Control

        for message in self.messages:
            message.swarm_size = self.formation_size
            message.position_ref[0] = self.virtual_leader_pos[0]
            message.position_ref[1] = self.virtual_leader_pos[1]
            message.position_ref[2] = self.virtual_leader_pos[2]
            message.velocity_ref[0] = self.virtual_leader_vel[0]
            message.velocity_ref[1] = self.virtual_leader_vel[1]
            message.velocity_ref[2] = self.virtual_leader_vel[2]
            message.yaw_ref = self.virtual_leader_yaw

        self.publish_once()


