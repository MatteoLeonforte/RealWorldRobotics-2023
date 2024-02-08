from mujoco_controller_framework import *
import trajectories
from motor_control.example_GroupA import *
from motor_control.gripper_controller_GroupA import *

if __name__ == '__main__':

    gc = GripperController(port="/dev/ttyUSB0",calibration=False)
    hand = Hand(gc)
    trajectories.palm_x_rotation(hand)

