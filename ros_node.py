import os

import numpy as np
import rospy

from PyQt5.QtCore import QThread, pyqtSignal

from talker.msg import SwarmCommand


class UAVNode(QThread):
    data_arrive = pyqtSignal(name='data_signal')

    def __init__(self):
        super().__init__()
        node_name = 'uav_node'
        rospy.init_node(node_name, anonymous=True)
        self.stop_flag = False
        self.rate = rospy.Rate(1)

        self.swarm_num_uav = 1
        self.publishers = []
        for i in range(self.swarm_num_uav):
            publisher = rospy.Publisher('/uav%d' % i + '/prometheus/swarm_command', SwarmCommand, queue_size=1)
            self.publishers.append(publisher)

        self.controller_num = 0
        self.formation_size = 1
        self.virtual_leader_pos = [0, 0, 1]
        self.virtual_leader_vel = [0, 0, 0]
        self.virtual_leader_yaw = 0

        self.messages = [SwarmCommand() for i in range(self.swarm_num_uav)]

    def publish_once(self):
        for message, publisher in zip(self.messages, self.publishers):
            publisher.publish(message)

    def idle(self):
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
        解锁
        :return:
        """
        for message in self.messages:
            message.Mode = SwarmCommand.Disarm
        self.publish_once()

    def publish_formation(self, formation_idx):
        if formation_idx == 0:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.One_column
        elif formation_idx == 1:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Triangle
        elif formation_idx == 2:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Square
        elif formation_idx == 3:
            for message in self.messages:
                message.swarm_shape = SwarmCommand.Circular

        if self.controller_num == 0:
            for message in self.messages:
                message.Mode = SwarmCommand.Position_Control
        if self.controller_num == 1:
            for message in self.messages:
                message.Mode = SwarmCommand.Velocity_Control
        if self.controller_num == 2:
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


