"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import time
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.2)

arm.close_gripper()

q = arm.neutral_position()
arm.safe_move_to_position(q)
arm.open_gripper()

q = np.array([0,-1 ,0,-2,-0.3,1,1]) # TODO: try changing this!
arm.safe_move_to_position(q)
time.sleep(5) #time for screenshots
q = np.array([1,-1.5 ,0,-2,0,1,1]) # TODO: try changing this!
arm.safe_move_to_position(q)
time.sleep(5) #time for screenshots
q = np.array([0,0 ,0,-2,0,2,0]) # TODO: try changing this!
arm.safe_move_to_position(q)
time.sleep(5) #time for screenshots
q = np.array([1.5,0.5 ,0,-1,0,2,0]) # TODO: try changing this!
arm.safe_move_to_position(q)
time.sleep(5) #time for screenshots
q = np.array([1.5,0 ,0.5,-0.1,1,1.5,0]) # TODO: try changing this!
arm.safe_move_to_position(q)
time.sleep(5) #time for screenshots
arm.close_gripper()
