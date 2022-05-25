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
        self.messages = []
        for i in range(self.swarm_num_uav):
            publisher = rospy.Publisher('/uav%d' % (i + 1) + '/prometheus/swarm_command', SwarmCommand, queue_size=1)
            self.publishers.append(publisher)
            self.messages.append(SwarmCommand())

        self.control_mode = 0
        self.formation_size = 1
        self.virtual_leader_pos = [0, 0, 1]
        self.virtual_leader_vel = [0, 0, 0]
        self.virtual_leader_yaw = 0

    def publish_once(self):
        for message, publisher in zip(self.messages, self.publishers):
            publisher.publish(message)

    def change_formation_size(self, formation_size):
        try:
            self.formation_size = int(formation_size)
        except ValueError:
            print('formation size should be a number')

    def change_mode(self):
        for message in self.messages:
            message.Mode = SwarmCommand.Idle
        # self.publish_once()

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

    def publish_formation(self, formation_idx, control_mode):
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

        self.control_mode = control_mode
        if control_mode == 0:
            for message in self.messages:
                message.Mode = SwarmCommand.Position_Control
        if control_mode == 1:
            for message in self.messages:
                message.Mode = SwarmCommand.Velocity_Control
        if control_mode == 2:
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


