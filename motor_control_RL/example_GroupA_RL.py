from gripper_controller_GroupA_RL import GripperController
import time
import numpy as np
import finger_kinematics_GroupA_RL as fk
import os


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
            """
            limits = gc.compare_joint_angles_to_limits(goalpos)
            for i in range(np.size(limits, 1)):
                if limits[0,i] is False:
                    print("Angle exceeds upper limit for joint angle ", i)
                elif limits[1,i] is False:
                    print("Angle exceeds lower limit for joint angle ", i)
                else:
                    print("Desired joint angles are being achieved...")
                    gc.write_desired_joint_angles(goalpos)
                    gc.wait_for_motion()
                    time.sleep(1)
                    print("Desired joint angles achieved!")
            """
            print("Desired joint angles are being achieved...")
            gc.write_desired_joint_angles(goalpos)
            gc.wait_for_motion()
            time.sleep(0.5)
            print("Desired joint angles achieved!")
             
        i = i+1

    print("Process terminated!")

    pass
    
def trajectory_input(gc: GripperController):
    #Task1
    ja1 =      [0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0]
    
    #Task2
    ja2 =   [0, 0, 0, -45, 0, -45, 0, -45, 0, -45, 0]
    ja3 =   [0, 0, 0, 0, 0, -45, 0, -45, 0, -45, 0]
    ja4 =   [0, 0, 0, 0, 0, 0, 0, -45, 0, -45, 0]
    ja5 =   [0, 0, 0, 0, 0, 0, 0, 0, 0, -45, 0]
    ja6 =   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    #Task3
    ja7 =   [0, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ja8 =   [0, 45, 0, 45, 0, 0, 0, 0, 0, 0, 0]
    ja9 =   [0, 45, 0, 45, 0, 45, 0, 0, 0, 0, 0]
    ja10 =  [0, 45, 0, 45, 0, 45, 0, 45, 0, 0, 0]
    ja11 =  [0, 45, 0, 45, 0, 45, 0, 45, 0, 45, 0]
    
    #Task3.5
    ja35 =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #Task4
    ja12 =  [0, 0, 45, 0, 0, 0, 0, 0, 0, 0, 0]
    ja13 =  [0, 0, 45, 0, 45, 0, 0, 0, 0, 0, 0]
    ja14 =  [0, 0, 45, 0, 45, 0, 45, 0, 0, 0, 0]
    ja15 =  [0, 0, 45, 0, 45, 0, 45, 0, 45, 0, 0]
    ja16 =  [0, 0, 45, 0, 45, 0, 45, 0, 45, 0, 45]
    
    #Task5
    ja17 =  [0, 0, 90, 0, 45, 0, 45, 0, 45, 0, 45]
    ja18 =  [0, 0, 90, 0, 90, 0, 45, 0, 45, 0, 45]
    ja19 =  [0, 0, 90, 0, 90, 0, 90, 0, 45, 0, 45]
    ja20 =  [0, 0, 90, 0, 90, 0, 90, 0, 90, 0, 45]
    ja21 =  [0, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90]

    #Task6 go back to calibration position
    
    task1 = [ja1]
    task2 = [ja2, ja3, ja4, ja5, ja6]
    task3 = [ja7, ja8, ja9, ja10, ja11]
    task35 = [ja35]
    task4 = [ja12, ja13, ja14, ja15, ja16]
    task5 = [ja17, ja18, ja19, ja20, ja21]
    task6 = [ja1]
    
    tasks = [task1, task2, task3, task35, task4, task5, task6]
    
    for task in tasks:
        print("It's time for task", task.index)
        for pose in task:
            print("Next joint angles: ", pose)
            input("Agree to these joint angles with ENTER.")
            gc.write_desired_joint_angles(np.array(pose))
            print("Waiting for motion")
            #gc.wait_for_motion()
        
    
    """
    gc.write_desired_joint_angles(ja_first)
    gc.wait_for_motion()
    time.sleep(1)
    """
    pass

def test_tendons(gc: GripperController):
    
    loop = 0
    loop = int(input("How many times do you wish to tes the tendons:"))
    i = 0
    
    #ja_start_pos = np.zeros(11)
    ja_start_pos = np.array([0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0])
    print("Motor position according calibration: ", gc.motor_id2init_pos)
    
    motor_pos_start = gc.test_desired_joint_angles(ja_start_pos)
    print("The motors position at start: ", motor_pos_start)
    
    
    while(i < loop):
        
        joint_angles = [float(x) for x in input("Input desired joint angles in degrees:").split()]
        print("Desired joint angles: ", joint_angles)
        
        motor_pos_des = gc.test_desired_joint_angles(joint_angles)
        
        joint_angles = np.deg2rad(joint_angles)
        
        tendon_lengths_finger1 = fk.pose2tendon_thumb(joint_angles[0], joint_angles[1], joint_angles[2])
        print("Tendon lengths Finger1: ", tendon_lengths_finger1)
        
        
        tendon_lengths_finger2 = fk.pose2tendon_finger(joint_angles[3], joint_angles[4])
        print("Tendon lengths Finger2: ", tendon_lengths_finger2)
        
        tendon_lengths_finger3 = fk.pose2tendon_finger(joint_angles[5], joint_angles[6])
        print("Tendon lengths Finger3: ", tendon_lengths_finger3)
        
        tendon_lengths_finger4 = fk.pose2tendon_finger(joint_angles[7], joint_angles[8])
        print("Tendon lengths Finger4: ", tendon_lengths_finger4)
        
        tendon_lengths_finger5 = fk.pose2tendon_finger(joint_angles[9], joint_angles[10])
        print("Tendon lengths Finger5: ", tendon_lengths_finger5)
        
        
        
        
        print("New Motor position would be: ", motor_pos_des)
        
        motor_pos_diff = motor_pos_des - motor_pos_start
        actual_motor_angles_diff = motor_pos_diff * 180/np.pi
        
        print("The motors actually turned by this amount in deg: ", actual_motor_angles_diff)
    
        i = i+1
    pass

def manipulate(gc:GripperController):
    curr_motor_pos = []
    curr_motor_pos = gc.get_motor_pos()
    print("Current Motor-positions: ", curr_motor_pos)
    
    task = int(input("Select Task: (1) test_tendons, (2) test_desired_joint_angles, (3) trajectory, (4) grabbing:"))
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
    
    elif task == 4:
            print("Task 4: Grabbing on Robotic arm selected!")
            test_hand_with_arm(gc)
    else:
        print("No valid task selected. Process will terminate!")

def test_hand_with_arm(gc: GripperController):
    max_angles = np.array([0, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]) #if only lower joints are used
    #max_angles = np.array([0, 90, 90, 90, 90, 90, 90, 90, 90, 90]) #if both lower and upper joints are used 
    
    change_angles = np.array([0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0])
    changing_fingers = [int(x) for x in input("Input which fingers to be moved (1, 2, 3, 4 or 5): ").split()]

        
    scaling = 1.0
    while(scaling != 2.0):
        scaling = float(input("Scaling factor [0,1] (enter 2 to break loop): "))
        scaling_thumb = float(input("Scaling factor thumb [0,1]: "))
        scaling_rot = float(input("Scaling factor thumb_rotation [0,1]: "))
        
        if scaling > 1.0 and scaling < 0.0 and scaling_thumb > 1.0 and scaling_thumb < 0.0 and scaling_rot > 1.0 and scaling_rot < 0.0:
            print("Scaling factor too small or too large!")
        else:
            if scaling == 2 or scaling_thumb == 2 or scaling_rot == 2:
                break

            for finger in changing_fingers:
                change_angles[2*finger-1] = scaling*90.0
                change_angles[2*finger] = scaling*90.0
            
            change_angles[1] = scaling_thumb*90
            change_angles[2] = scaling_thumb*90
            change_angles[0] = scaling_rot*50
            #goalpos = scaling * max_angles
            print("Following joint angles are being achieved: ", change_angles)
            assert(scaling <= 1.0)
            assert(scaling >=0.0)
            gc.write_desired_joint_angles(change_angles)
            print("Motor currents: ", gc.get_motor_cur())
            time.sleep(1)
    
    gc.write_desired_joint_angles([0, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0])
    

def set_RL_policy(gc: GripperController):
     # Policy loading
    base_path = os.path.dirname(os.path.realpath(__file__))
    policy_path = base_path + "/recorded_policies/Sphere_UP_XNEG_15_scaled_500epochs.npy"
    data = np.load(policy_path)
    data = data[0, :, :]
    data_len = data.shape[0]
    print(data.shape) #should be (1000,11)

    max_angles = np.array([60, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90])
    max_angles_rad = np.deg2rad(max_angles)
    min_angles = np.array([-60, -45, 0, -45, 0, -45, 0, -45, 0, -45, 0])
    min_angles_rad = np.deg2rad(min_angles)

    cmd_idx = 0
    while True:
        if cmd_idx >=data_len:
            cmd_idx=0
            print("RESTARTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        angles = data[cmd_idx, :]
        
        np.clip(angles, min_angles_rad, max_angles_rad)
        print("Following joint angles commanded to robot: ", np.rad2deg(angles))
        print(len(angles))

        gc.command_joint_angles(angles)
        #gc.wait_for_motion()
        cmd_idx += 1 # ADDED
        time.sleep(0.1)

def main():
    
    #Gripper intialization & calibration
    global gc
    gc = GripperController(port="/dev/ttyUSB0",calibration=True)
    
   

    
    #manipulate(gc)
    #test_hand_with_arm(gc)
    set_RL_policy(gc)
    
    #ball 0.6 0.7 0
    #mug 0.45 0.45 0
    #pen 0.7 0.8 0.5
    
    
    
    gc.terminate()
    pass

if __name__ == "__main__":
    main()