"""
Created on Fri Feb  7, 2020

@author: Evan, Michael, Frankie
"""
import sys
import time
import pickle
import pdb


class BaxterSim:
    def __init__(self):
        try:
            import sim
        except Exception:
            print('sim.py could not be imported')
        self.sim = sim
        self.clientID = self.connect_to_sim()

    def connect_to_sim(self):
        ''' Connect to CoppeliaSim'''
        print('Program started')
        self.sim.simxFinish(-1)  # just in case, close all opened connections

        clientID = self.sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if clientID != -1:
            print('Connected to remote API server')
        else:
            print('Failed connecting to remote API server')
            sys.exit("Could not connect")
        return clientID

    def set_right_arm(self, joint_vals):
        """ Set Right arm to a specific position

        Parameters:
        joint_vals (array): array containing joint positions for each DOF of right arm

        """
        joint_nums = []
        for i in range(1, 8):
            err_code, joint = self.sim.simxGetObjectHandle(self.clientID, "Baxter_rightArm_joint" + str(i),
                                                           self.sim.simx_opmode_blocking)
            joint_nums.append(joint)
        for i in range(7):
            self.sim.simxSetJointTargetPosition(self.clientID, joint_nums[i], joint_vals[i],
                                                self.sim.simx_opmode_oneshot)


def main():
    baxter_sim = BaxterSim()
    baxter_sim.connect_to_sim()

    trajectory_right = pickle.load(open("arr_right.pkl", "rb"))
    joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    for frame in trajectory_right:
        joint_values = [frame[name] for name in joint_names]
        baxter_sim.set_right_arm(joint_values)
    time.sleep(1)
    print('Program ended')


if __name__ == "__main__":
    main()
