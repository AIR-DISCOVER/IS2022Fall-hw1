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
        '''constants'''
        # TODO: change the constants to your own
        self.l1 = 0.12
        self.l2 = 0.1225
        self.l3 = 0
        
        '''variables'''
        # for forward kinematics
        self.joint_pos = [0, 0] # in rad
        # for inverse kinematics
        self.end_point = [0, 0] 
        # self.header = Header()
        # self.header.frame_id = 'base_link'  # TODO: change the frame_id to your own
        
        '''ros communication'''
        # forward kinematics
        self.sub_joint = rospy.Subscriber('joint_states', JointState, self.joint_callback)
        self.pub_end = rospy.Publisher('arm_position', Pose, queue_size=10)
        # inverse kinematics
        self.sub_end = rospy.Subscriber('arm_position', Pose, self.end_callback)
        self.pub_joint = rospy.Publisher('joint_states', JointState, queue_size=10)
        
    def forward_kinematics(self):
        """forward kinematics
        """
        def _SE2_transform_from_angle(angle, length):
            """return a SE2 transform matrix from angle and length

            :param num angle: joint angle in rad
            :param num length: the length of the link
            :return np.ndarray: the transform matrix
            """
            return np.array([[np.cos(angle), -np.sin(angle), length * np.cos(angle)],
                             [np.sin(angle), np.cos(angle),  length * np.sin(angle)],
                             [0,             0,              1                     ]])
            
        T1 = _SE2_transform_from_angle(self.joint_pos[0], self.l1)
        T2 = _SE2_transform_from_angle(self.joint_pos[1], self.l2)
        
        end_point = T1 @ T2 @ np.array([0, 0, 1]) + np.array([self.l3, 0, 0])
        
        return end_point

    def inverse_kinematics(self):
        # TODO：implement inverse kinematics
        pass
    
    def joint_callback(self, msg: JointState):
        """callback function for joint_states

        :param JointState msg: subscriber message
        """
        self.joint_pos[0] = msg.position[0]
        self.joint_pos[1] = msg.position[1]
        
    def end_callback(self, msg: Pose):
        """callback function for end_point

        :param Pose msg: subscriber message
        """
        self.end_point[0] = msg.position.x
        self.end_point[1] = msg.position.y
        
    def publish_forward(self):
        """publish the end_point from the joint_states by forward kinematics
        """
        end_point = self.forward_kinematics()
        msg = Pose()
        # header
        # self.header.stamp = rospy.Time.now()
        # self.header.seq += 1
        # msg.header = self.header
        # end_point
        msg.position.x = end_point[0]
        msg.position.y = end_point[1]
        msg.position.z = 0
        
        self.pub_end.publish(msg)
        
    def publish_inverse(self):
        """publish the joint_states from the end_point by inverse kinematics
        """
        joint_pos = self.inverse_kinematics()
        msg = JointState()
        # header
        # self.header.stamp = rospy.Time.now()
        # self.header.seq += 1
        # msg.header = self.header
        # joint_pos
        msg.position = joint_pos
        
        self.pub_joint.publish(msg)
        

if __name__ == '__main__':
    rospy.init_node('me_arm')
    rate = rospy.Rate(100)
    
    me_arm = MeArm()
    
    while not rospy.is_shutdown():
        me_arm.publish_forward()
        rospy.loginfo_once('publish forward...')
        rate.sleep()
