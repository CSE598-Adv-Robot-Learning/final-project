import time
import random
import numpy as np
import rospy
from baxter_pykdl import baxter_kinematics
import baxter_interface
import pickle

rospy.init_node('baxter_kinematics')
limb_l = baxter_interface.Limb('left')
limb_r = baxter_interface.Limb('right')
arr_right = pickle.load(open("arr_right.pkl"))

for i in arr_right:
    limb_r.set_joint_positions(i)
    time.sleep(0.2)
