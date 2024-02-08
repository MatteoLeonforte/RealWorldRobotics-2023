from mujoco_controller_framework import *

def palm_x_rotation2(hand):

    while True:

        # POSE 0
        hand.plate_right()                     # Plate right, open hand
        hand.move_finger('index', [-40, 90])
        hand.move_finger('middle', [-40, 90]) 
        hand.move_finger('ring', [-40, 90]) 
        hand.move_finger('pinky', [70, 50])  
        hand._update()

        # POSE 1  
        hand.move_finger('index', [0, 10])
        hand.move_finger('middle', [-40, 90]) 
        hand.move_finger('ring', [-40, 90])     
        hand._update()

        # POSE 1  
        hand.plate_center() 
        
        hand.move_finger('index', [0, 10])
        hand.move_finger('middle', [0, 10]) 
        hand.move_finger('ring', [-40, 90])     
        hand._update()

        # POSE 1  
        hand.move_finger('thumb', [0, 90])
        hand.move_finger('index', [0, 10])
        hand.move_finger('middle', [0, 10]) 
        hand.move_finger('ring', [0, 10])     
        hand._update()
        
        # POSE 4
        hand.move_finger('thumb', [10, 10])
        hand.move_finger('index', [10, 10])
        hand.move_finger('middle', [40, 0])
        hand.move_finger('ring', [-40, 90]) 
        hand._update()


def palm_x_rotation(hand):

    while True:

        # POSE 0
        hand.plate_right()                     # Plate right, open hand
        hand.move_finger('index', [-40, 90])
        hand.move_finger('middle', [10, 10]) 
        hand.move_finger('ring', [40, 0]) 
        hand.move_finger('pinky', [70, 50])  
        hand._update()

        # POSE 1  
        hand.move_finger('thumb', [50, 20])
        hand.move_finger('index', [-40, 90]) 
        hand.move_finger('middle', [-40, 90])  
        hand.move_finger('ring', [0, 10])     
        hand._update()
        
        # POSE 1    
        hand.plate_center()                     # Plate center, open hand
        hand.move_finger('index', [40, 0]) 
        hand.move_finger('middle', [-40, 90])  
        hand.move_finger('ring', [0, 10])     
        hand._update()


        # POSE 3
        hand.move_finger('thumb', [0, 90])
        hand.move_finger('index', [10, 10])
        hand.move_finger('middle', [40, 0])
        hand.move_finger('ring', [-40, 90]) 
        hand._update()

        # POSE 4
        hand.move_finger('thumb', [10, 10])
        hand.move_finger('index', [10, 10])
        hand.move_finger('middle', [40, 0])
        hand.move_finger('ring', [-40, 90]) 
        hand._update()


def palm_z_rotation(hand):

    while True:

        # POSE 0
        hand.plate_center()                     # Plate center, open hand   
        hand.open_all_fingers(20)
        hand._update()
        
        # POSE 1    
        hand.plate_left()                       # Rotate plate to the left, open hand
        hand._update()

        # POSE 3
        hand.move_finger('thumb', [20, 50])     # Thumb and index

        hand.move_finger('index', [20, 70])
        hand.move_finger('middle', [20, 55])
        hand.move_finger('ring', [20, 40])
        hand._update()


        # POSE 5
        hand.plate_right()                   # Thumb and pinky    
        hand.move_finger('index', [20, 20])
        hand.move_finger('middle', [20, 40])
        hand.move_finger('ring', [20, 55])
        hand.move_finger('pinky', [20, 70])
        hand._update()

        # POSE 6
        hand.open_all_fingers(20)                

def finger_rotation_x_forward(hand):

    while True:
        
        # POSE 1
        hand.move_finger('thumb', [10, 20])
        hand.move_finger('middle', [10, 90])
        hand.move_finger('ring', [10, 90])
        hand.move_finger('index', [70, 10])
        hand.move_finger('pinky', [70, 10])
        hand._update()

        # POSE 2
        hand.move_finger('thumb', [60, 20])
        hand.move_finger('middle', [50, 80])
        hand.move_finger('ring', [50, 80])
        hand.move_finger('index', [70, 10])
        hand.move_finger('pinky', [70, 10])
        hand._update()


        # POSE 2.5
        hand.move_finger('thumb', [60, 20])
        hand.move_finger('middle', [50, 80])
        hand.move_finger('ring', [50, 80])
        hand.move_finger('index', [30, 10])
        hand.move_finger('pinky', [30, 10])
        hand._update()

        # POSE 3
        hand.move_finger('thumb', [20, 90])
        hand.move_finger('middle', [50, 20])
        hand.move_finger('ring', [50, 20])
        hand.move_finger('index', [30, 10])
        hand.move_finger('pinky', [30, 10])
        hand._update()

        # POSE 3.5
        hand.move_finger('thumb', [20, 90])
        hand.move_finger('middle', [50, 20])
        hand.move_finger('ring', [50, 20])
        hand.move_finger('index', [70, 10])
        hand.move_finger('pinky', [70, 10])
        hand._update()

        # POSE 4
        hand.move_finger('thumb', [10, 90])
        hand.move_finger('middle', [10, 20])
        hand.move_finger('ring', [10, 20])
        hand.move_finger('index', [70, 10])
        hand.move_finger('pinky', [70, 10])
        hand._update()