#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 27 17:16:21 2017

@author: cschuwerk
"""

PKG = 'roboclaw_driver'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

#import sys
import unittest

import rospy
from kinematics_python_wrapper.kinematics_py import Kinematics;


# Kinematics class
class ArmKinematics(Kinematics):

    def __init__(self, kinematics_plugin, position_only_ik, solve_type, start_link_urdf, end_link_urdf):
        try:
            self.initResult = False
            if kinematics_plugin == 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin':
                rospy.set_param('/ik_node/trac_ik/position_only_ik', position_only_ik)
                rospy.set_param('/ik_node/trac_ik/solve_type', solve_type)
            super(ArmKinematics, self).__init__("ik_node", kinematics_plugin)
            self.initResult = self.initialize('/robot_description', "arm", start_link_urdf, end_link_urdf, 0.05)
            rospy.loginfo('IK plugin initialized!')
        except Exception as e:
            rospy.logerr('Could not initialize IK plugin: ' + e.message)
            return
    
    def getInitResult(self):
        return self.initResult


# Test Case for the Kinematics plugin
class TestArmKinematics(unittest.TestCase):
    
    # Called automatically before every test below
    # see: https://docs.python.org/3/library/unittest.html
    def setUp(self):
        self.kinematics_plugin = rospy.get_param('~kinematics_plugin')
        self.position_only_ik = rospy.get_param('~position_only_ik')
        self.solve_type = rospy.get_param('~solve_type')
        self.start_link_urdf = rospy.get_param('~start_link_urdf')
        self.end_link_urdf = rospy.get_param('~end_link_urdf')
        self.kinematics = ArmKinematics(self.kinematics_plugin, self.position_only_ik, self.solve_type, self.start_link_urdf, self.end_link_urdf)
        self.assertTrue(self.kinematics.getInitResult())
    
    def test_arm_kinematics_init(self):
        self.assertTrue(self.kinematics.getInitResult())

    # Some forward kinematic tests
    def test_fk_all_joints_zero(self):
        pose = self.kinematics.getPositionFK(self.end_link_urdf, [0.0, 0.0, 0.0, 0.0, 0.0] )
        self.assertIsNotNone(pose)
        self.assertEqual(len(pose['position']), 3)
        self.assertEqual(len(pose['orientation']), 4)
        
    def test_fk_few_joints(self):
        pose = self.kinematics.getPositionFK(self.end_link_urdf, [0.0] )
        self.assertIsNone(pose)
        
    def test_fk_too_many_joints(self):
        pose = self.kinematics.getPositionFK(self.end_link_urdf, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        self.assertIsNone(pose)
        
    def test_fk_joint_limit(self):
        pose = self.kinematics.getPositionFK(self.end_link_urdf, [-99.0, 0.0, 0.0, 0.0, 0.0] )
        self.assertIsNotNone(pose)
        ik_solution = self.kinematics.searchPositionIK(pose['position'], pose['orientation'], [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)
        ik_solution_approx = self.kinematics.searchApproximatePositionIK(pose['position'], pose['orientation'], [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)        
        self.assertIsNotNone(ik_solution)
        self.assertIsNotNone(ik_solution_approx)
        self.assertNotEqual(ik_solution[0], -99.0)
        self.assertNotEqual(ik_solution_approx[0], -99.0)
        
    # Some inverse kinematic tests
    def test_ik_for_zero_pos(self):
        pose = self.kinematics.getPositionFK(self.end_link_urdf, [0.0, 0.0, 0.0, 0.0, 0.0] )
        self.assertIsNotNone(pose)
        ik_solution = self.kinematics.searchPositionIK(pose['position'], pose['orientation'], [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)
        #ik_solution_approx = self.kinematics.searchApproximatePositionIK(pose['position'], pose['orientation'], [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)        
        self.assertIsNotNone(ik_solution)
        #self.assertIsNotNone(ik_solution_approx)
    
    def test_ik_unreachable(self):
        position = [10.0, 10.0, 10.0]
        orientation = [0.0, 0.0, 0.0, 1.0]
        ik_solution = self.kinematics.searchPositionIK(position, orientation, [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)
        #ik_solution_approx = self.kinematics.searchApproximatePositionIK(position, orientation, [0.0, 0.0, 0.0, 0.0, 0.0], 0.1)
        self.assertIsNone(ik_solution)
        #self.assertIsNone(ik_solution_approx)
    
    def test_ik_approximate_solution(self):
        position = [0.3, -0.1, 0.3]
        orientation = [0.925, -0.371, 0.082, 0.000]
        #seed_state = [-0.38, -0.47, 0.11, -1.04, -2.76]
        seed_state = [0.0, 0.0, 0.0, 0.0, 0.0]
        ik_solution = self.kinematics.searchPositionIK(position, orientation, seed_state, 0.5)
        ik_solution_approx = self.kinematics.searchApproximatePositionIK(position, orientation, seed_state, 0.5)
        self.assertIsNone(ik_solution)
        self.assertIsNotNone(ik_solution_approx)

if __name__ == '__main__':
    
    # Required to have ROS output streams displayed for debugging
    rospy.init_node('kinematics_python_wrapper_test_kdl', anonymous=False)
    
    
    import rostest
    rostest.rosrun(PKG, 'test_kinematics_python_wrapper', TestArmKinematics)
    

    #rospy.on_shutdown(shutdown_hook)
    #rospy.spin()


