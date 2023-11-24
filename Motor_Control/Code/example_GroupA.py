from gripper_controller_GroupA import GripperController
import time
import numpy as np
import finger_kinematics_GroupA as fk


"""
Example script to control the finger joint angles
"""


def desired_joint_angles_test(gc: GripperController):
    #repeat desired joint angles
    loop = 0
    loop = int(input("How many times do you wish to select the position:"))
    i = 0
    while(i < loop):
        written = 1
        while(written):
            desire = {}
            desire = input("Choose whether specific joint-angles (ja) or abort (break) is desired:")
            
            # if desire == "mp":
            #     goalpos_motor = [float(x) for x in input("Input desired motor-pos:").split()]
            #     print("Desired motor-positions are: ", goalpos_motor)
            #     written = 0
                
            if desire == "ja":
                goalpos = [float(x) for x in input("Input desired joint angles in degrees:").split()]
                written = 0
                print("Desired joint-angles are:", goalpos)
            elif desire == "break":
                print("Process is being aborted")
                written = 1
            else:
                print("Input is not acceptable! Try again:")

        if desire == "ja":
            print("Desired joint angles are being achieved...")
            gc.write_desired_joint_angles(goalpos)
            gc.wait_for_motion()
            time.sleep(1)
            print("Desired joint angles achieved!")
        
        # if desire == "mp":
        #     print("Desired motor positions are being achieved...")
        #     gc.write_desired_motor_pos(np.array(goalpos_motor))
        #     gc.wait_for_motion()
        #     time.sleep(1)
        #     print("Desired motor positions achieved!")
        i = i+1

    print("Process terminated!")

    pass
    
def trajectory_input(gc: GripperController):
    #NOT COMPLETE
    ja_first = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ja_second = [15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ja_third = [60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    print("First joint angles:", ja_first)
    print("First joint angles are being achieved...")
    gc.write_desired_joint_angles(ja_first)
    gc.wait_for_motion()
    time.sleep(1)
    
    print("Second joint angles", ja_second)
    print("Second joint angles are being achieved...")
    gc.write_desired_joint_angles(ja_second)
    gc.wait_for_motion()
    time.sleep(1)
    
    print("Third joint angles:", ja_third)
    print("Third joint angles are being achieved...")
    gc.write_desired_joint_angles(ja_third)
    gc.wait_for_motion()
    time.sleep(1)
    
    print("First joint angles:", ja_first)
    print("First joint angles are being achieved...")
    gc.write_desired_joint_angles(ja_first)
    gc.wait_for_motion()
    time.sleep(1)

    pass

def test_tendons(gc: GripperController):
    
    loop = 0
    loop = int(input("How many times do you wish to tes the tendons:"))
    i = 0
    
    ja_start_pos = np.zeros(11)
    print("First ground-truth joint angles: ", ja_start_pos)
    motor_pos_start = gc.test_desired_joint_angles(ja_start_pos)
    print("The motors position at start: ", motor_pos_start)
    
    
    while(i < loop):
        
        joint_angles = [float(x) for x in input("Input desired joint angles in degrees:").split()]
        print("Desired joint angles: ", joint_angles)
        
        tendon_lengths_finger1 = fk.pose2tendon_finger1(joint_angles[0], joint_angles[1], joint_angles[2])
        print("Tendon lengths Finger1: ", tendon_lengths_finger1)
        
        
        tendon_lengths_finger2 = fk.pose2tendon_finger2(joint_angles[3], joint_angles[4])
        print("Tendon lengths Finger2: ", tendon_lengths_finger2)
        
        tendon_lengths_finger3 = fk.pose2tendon_finger3(joint_angles[5], joint_angles[6])
        print("Tendon lengths Finger3: ", tendon_lengths_finger3)
        
        tendon_lengths_finger4 = fk.pose2tendon_finger4(joint_angles[7], joint_angles[8])
        print("Tendon lengths Finger4: ", tendon_lengths_finger4)
        
        tendon_lengths_finger5 = fk.pose2tendon_finger5(joint_angles[9], joint_angles[10])
        print("Tendon lengths Finger5: ", tendon_lengths_finger5)
        
        motor_pos_des = gc.test_desired_joint_angles(joint_angles)
        
        
        print("New Motor position would be: ", motor_pos_des)
        
        motor_pos_diff = motor_pos_des - motor_pos_start
        actual_motor_angles_diff = motor_pos_diff * 180/np.pi
        
        print("The motors actually turned by this amount in deg: ", actual_motor_angles_diff)
    
        i = i+1
    pass

def main():
    
    homepos = []
    goalpos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    #curr_joint_angles = []
    #curr_joint_angles = gc.
    #print("Current joint_angles: {}")
    global gc
    gc = GripperController(port="/dev/ttyUSB0",calibration=True)
    curr_motor_pos = []
    curr_motor_pos = gc.get_motor_pos()
    print("Current Motor-positions: ", curr_motor_pos)
    
    task = int(input("Select Task: (1) test_tendons, (2) test_desired_joint_angles, (3) trajectory:"))
    if task == 1:
        print("Task 1: Test tendon lengths selected!")
        test_tendons(gc)
        
    elif task == 2 or task == 3:
        if task == 2:
            print("Task 2: Follow desired joint angles selected!")
            desired_joint_angles_test(gc)
        elif task == 3:
            print("Task 3: Follow predefined trajectory selected!")
            trajectory_input(gc)
    else:
        print("No valid task selected. Process will terminate!")
    
    
    gc.terminate()


if __name__ == "__main__":
    main()