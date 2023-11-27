from gripper_controller import GripperController
import time


"""
Example script to control the finger joint angles
"""

homepos = [0, 0]
goalpos = [0,  1]


def main():
    global gc
    gc = GripperController(port="/dev/ttyusb0",calibration=True)
    

    gc.write_desired_joint_angles(goalpos)

    gc.wait_for_motion()

    time.sleep(1)

    gc.write_desired_joint_angles(homepos)

    #gc.wait_for_motion()

    time.sleep(1)

    gc.terminate()


if __name__ == "__main__":
    main()