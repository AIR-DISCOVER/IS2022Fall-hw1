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
        self.l3 = 0.13
        # self.l3 = 0

        # States
        self.cube_pose = [0, 0, 0]
        self.arm_pose = [0, 0, 0]

        self.pub_joint = rospy.Publisher('joint_states', JointState, queue_size=10)

        def on_update():
            msg = JointState()
            msg.position = self.inverse_kinematics(self.cube_pose, self.arm_pose)
            self.pub_joint.publish(msg)

        def on_cube_pose(msg: Pose):
            self.cube_pose[0] = msg.position.x
            self.cube_pose[1] = msg.position.y
            self.cube_pose[2] = msg.position.z
            on_update()

        def on_arm_pose(msg: Pose):
            self.arm_pose[0] = msg.position.x
            self.arm_pose[1] = msg.position.y + 0.11
            self.arm_pose[2] = msg.position.z
            on_update()

        # ROS topics
        self.sub_cube = rospy.Subscriber('/pose/cube_5', Pose, on_cube_pose)
        self.sub_arm = rospy.Subscriber('/pose/ep_world', Pose, on_arm_pose)

    def inverse_kinematics(self, cube_pose, ep_pose):
        joint_pos = [0, 0]
        
        # [TODO] implement reverse kinematics

        return joint_pos



if __name__ == '__main__':
    rospy.init_node('me_arm')
    me_arm = MeArm()
    rospy.spin()