#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   MecArm.py
@Time   :   2022/09/23 19:48:10
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   MecArm Forward Kinematics & Inverse Kinematics
'''
# from turtle import forward
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
# from std_msgs.msg import Header
import numpy as np


class MeArm:
    """MecArm class
    
      t2/
       /\ 
    l1/  \l2 
     /    \____
    /_t1    l3
    
    """

    def __init__(self) -> None:
        # Constants
        self.l1 = 0.12
        self.l2 = 0.1225
        self.l3 = 0.14

        # States
        self.joint_states = [0, 0]  # in radius
        self.cube_pose = [0, 0, 0]

        # ROS topics
        self.pub_end = rospy.Publisher('arm_position', Pose, queue_size=10)

        def on_joint_states(msg: JointState):
            """callback function for joint_states

            :param JointState msg: subscriber message
            """
            # self.joint_states[0] = msg.position[0]
            # self.joint_states[1] = msg.position[1]

            end_point = self.forward_kinematics([msg.position[0], msg.position[1]])
            msg = Pose()
            msg.position.x = end_point[0]
            msg.position.y = end_point[1]
            msg.position.z = 0
            self.pub_end.publish(msg)

        self.sub_joint = rospy.Subscriber('joint_states_hw1', JointState, on_joint_states)

    def forward_kinematics(self, joint_states):
        """forward kinematics
        """

        def _SE2_transform_from_angle(angle, length):
            """return a SE2 transform matrix from angle and length

            :param num angle: joint angle in rad
            :param num length: the length of the link
            :return np.ndarray: the transform matrix
            """
            return np.array([
                [np.cos(angle), -np.sin(angle), length * np.cos(angle)],
                [np.sin(angle), np.cos(angle), length * np.sin(angle)],
                [0, 0, 1],
            ])

        T1 = _SE2_transform_from_angle(joint_states[0], self.l1)
        T2 = _SE2_transform_from_angle(joint_states[1], self.l2)

        end_point = T1 @ T2 @ np.array([0, 0, 1]) + np.array([self.l3, 0, 0])

        return end_point


if __name__ == '__main__':
    rospy.init_node('me_arm')
    rate = rospy.Rate(100)

    me_arm = MeArm()

    rospy.spin()
