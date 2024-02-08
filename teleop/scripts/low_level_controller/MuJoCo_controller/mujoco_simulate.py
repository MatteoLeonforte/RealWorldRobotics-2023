from mujoco_controller_framework import *
import trajectories

if __name__ == '__main__':

    gc = GripperControllerMujocoSim()
    hand = Hand(gc)
    trajectories.finger_rotation_x_forward(hand)

