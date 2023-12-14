from math import cos, sin
import numpy as np


# ------------------- Calculations of Tendon Lengths at single joint ------------------- #
# TODO: Add your own functions here to calculate the tendon lengths for each joint
global radius
radius = 1 #in cm

#Thumb:
def tendonlength_left_joint0(theta_joint0):
   '''Input: joint angle of joint0 (rotary thumb) in rad
      Output: total normal lenght of left tendon'''
   return -tendonlength_right_joint0(theta_joint0)

def tendonlength_right_joint0(theta_joint0):
   '''Input: joint angle of joint0 (rotary thumb) in rad
      Output: total normal lenght of right tendon'''
   
   if theta_joint0 < 0:
      return -tendonlength_right_joint0(-theta_joint0)
   else:
      #dimensions
      delta_y_plate = 2.54803 #cm
      delta_x_plate = 0.89305 #cm
      theta_null = np.arctan(delta_x_plate/ delta_y_plate) #rad
      
      
      r_plate = np.sqrt(delta_x_plate**2+delta_y_plate**2)
      r_palm = 2.7 #cm
      delta_z = 0.1 #cm
      
      #calculate the ground length
      ground_etha = np.pi/2 - theta_null #rad
      ground_xy = np.sqrt(r_plate**2+r_palm**2-2*r_plate*r_palm*cos(ground_etha))
      ground_length = np.sqrt(ground_xy**2+delta_z**2)   #length if theta_joint0 = 0

      #calculate th new length
      etha = np.pi/2 - (theta_null + theta_joint0) #rad
      new_xy = np.sqrt(r_plate**2+r_palm**2-2*r_plate*r_palm*cos(etha))
      new_length = np.sqrt(new_xy**2+delta_z**2)
      
      #calculate difference in tendon length
      delta_length = new_length - ground_length 
      
      return delta_length


def tendonlength_flexor_joint1(theta_joint0, theta_joint1):
   '''Input: joint angle of joint1 (base-pp Finger1) in rad
      Output: total normal lengths of flexor tendon through joint1'''
      #TODO add correction terms for tendon through CoR
   return -tendonlength_extensor_joint1(theta_joint0, theta_joint1)

def tendonlength_extensor_joint1(theta_joint0, theta_joint1):
   '''Input: joint angle of joint1 (base-pp Finger 1) in rad
      Output: total normal lengths of extensor tendon through joint1'''
   #TODO add correction terms for tendon through CoR
   theta_norm = -np.pi/4
   """
   y_new = radius*sin(theta_joint1)+2*radius*cos(theta_joint1*0.5)
   y_norm = radius*sin(theta_norm)+2*radius*cos(theta_norm*0.5)
   
   z_new = -radius*(cos(theta_joint1)-1) + 2*radius*sin(theta_joint1*0.5)
   z_norm = -radius*(cos(-np.pi/4)-1) + 2*radius*sin(-np.pi/4*0.5)
   
   delta_x = 0
   delta_y = y_new - y_norm
   delta_z = z_new - z_norm
   length_delta = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
   """
   y_new = radius*sin(theta_joint1) + 2*radius*cos(theta_joint1*0.5)
   y_norm = radius*sin(theta_norm) + 2*radius*cos(theta_norm*0.5)
   
   z_new = radius*(1-cos(theta_joint1)) + 2*radius*sin(theta_joint1*0.5)
   z_norm = radius*(1-cos(theta_norm)) + 2*radius*sin(theta_norm*0.5)
   
   length_new = np.sqrt(y_new**2 + z_new**2)
   length_norm = np.sqrt(y_norm**2 + z_norm**2)
   
   length_delta = length_new - length_norm
   
   return length_delta

def tendonlength_flexor_joint2(theta_joint0, theta_joint2):
   '''Input: joint angle of joint2 (pp-mp Finger1) in rad
      Output: total normal lengths of flexor tendon through joint2'''
   #TODO add correction terms for tendon through CoR with theta0
   return -1/2*tendonlength_extensor_joint2(theta_joint0, theta_joint2)

def tendonlength_extensor_joint2(theta_joint0, theta_joint2):
   '''Input: joint angle of joint2 (pp-mp Finger1) in rad
      Output: total normal lengths of extensor tendon through joint2'''
   #TODO add correction terms for tendon through CoR with theta0
   theta_norm = 0.0
   """
   delta_x = 0
   delta_y = radius*sin(theta_joint2)+2*radius*cos(theta_joint2*0.5)-2*radius #norm is 2r because theta_norm = 0
   delta_z = -radius*(cos(theta_joint2)-1) + 2*radius*sin(theta_joint2*0.5)

   return 2*np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
   """
   theta_norm = 0.0
   
   y_new = radius*sin(theta_joint2) + 2*radius*cos(theta_joint2*0.5)
   y_norm = radius*sin(theta_norm) + 2*radius*cos(theta_norm*0.5)
   
   z_new = radius*(1-cos(theta_joint2)) + 2*radius*sin(theta_joint2*0.5)
   z_norm = radius*(1-cos(theta_norm)) + 2*radius*sin(theta_norm*0.5)
   
   length_new = np.sqrt(y_new**2 + z_new**2)
   length_norm = np.sqrt(y_norm**2 + z_norm**2)
   
   length_delta = length_new - length_norm
   
   return 2*length_delta



#Finger:
def tendonlength_flexor_joint3(theta_joint3):
   '''Input: joint angle of joint3 (base-pp Finger2) in rad
      Output: total normal lengths of flexor tendon through joint3'''
   return -tendonlength_extensor_joint3(theta_joint3)

def tendonlength_extensor_joint3(theta_joint3):
   '''Input: joint angle of joint3 (base-pp Finger2) in rad
      Output: total normal lengths of extensor tendon through joint3'''
   theta_norm = -np.pi/4
   """
   OLD VERSION
   y_new = radius*sin(theta_joint3)+2*radius*cos(theta_joint3*0.5)
   y_norm = radius*sin(theta_norm)+2*radius*cos(theta_norm*0.5)

   z_new = -radius*(cos(theta_joint3)-1) + 2*radius*sin(theta_joint3*0.5)
   z_norm = -radius*(cos(theta_norm)-1) + 2*radius*sin(theta_norm*0.5)
   
   delta_x = 0
   delta_y = radius*sin(theta_joint3)+2*radius*cos(theta_joint3*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint3)-1) + 2*radius*sin(theta_joint3*0.5)
   length_delta = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
   
   return np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
   """
   
   y_new = radius*sin(theta_joint3) + 2*radius*cos(theta_joint3*0.5)
   y_norm = radius*sin(theta_norm) + 2*radius*cos(theta_norm*0.5)
   
   z_new = radius*(1-cos(theta_joint3)) + 2*radius*sin(theta_joint3*0.5)
   z_norm = radius*(1-cos(theta_norm)) + 2*radius*sin(theta_norm*0.5)
   
   length_new = np.sqrt(y_new**2 + z_new**2)
   length_norm = np.sqrt(y_norm**2 + z_norm**2)
   
   length_delta = length_new - length_norm
   
   return length_delta

def tendonlength_flexor_joint4(theta_joint4):
   '''Input: joint angle of joint4 (pp-mp Finger2) in rad
      Output: total normal lengths of flexor tendon through joint4'''
   return -1/2*tendonlength_extensor_joint4(theta_joint4)

def tendonlength_extensor_joint4(theta_joint4):
   '''Input: joint angle of joint2 (pp-mp Finger2) in rad
      Output: total normal lengths of extensor tendon through joint4'''
   """
   delta_x = 0
   delta_y = radius*sin(theta_joint4)+2*radius*cos(theta_joint4*0.5)-2*radius #norm is 2R because theta_norm = 0
   delta_z = -radius*(cos(theta_joint4)-1) + 2*radius*sin(theta_joint4*0.5)

   return 2*np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
   """
   theta_norm = 0.0
   
   y_new = radius*sin(theta_joint4) + 2*radius*cos(theta_joint4*0.5)
   y_norm = radius*sin(theta_norm) + 2*radius*cos(theta_norm*0.5)
   
   z_new = radius*(1-cos(theta_joint4)) + 2*radius*sin(theta_joint4*0.5)
   z_norm = radius*(1-cos(theta_norm)) + 2*radius*sin(theta_norm*0.5)
   
   length_new = np.sqrt(y_new**2 + z_new**2)
   length_norm = np.sqrt(y_norm**2 + z_norm**2)
   
   length_delta = length_new - length_norm
   
   return 2*length_delta

"""
#Finger 3:
def tendonlength_flexor_joint5(theta_joint5):
   '''Input: joint angle of joint5 (base-pp Finger3) in rad
      Output: total normal lengths of flexor tendon through joint5'''
   return -tendonlength_extensor_joint5(theta_joint5)

def tendonlength_extensor_joint5(theta_joint5):
   '''Input: joint angle of joint5 (base-pp Finger3) in rad
      Output: total normal lengths of extensor tendon through joint5'''
   delta_x = 0
   delta_y = radius*sin(theta_joint5)+2*radius*cos(theta_joint5*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint5)-1) + 2*radius*sin(theta_joint5*0.5)
   
   return np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

def tendonlength_flexor_joint6(theta_joint6):
   '''Input: joint angle of joint6 (pp-mp Finger3) in rad
      Output: total normal lengths of flexor tendon through joint6'''
   return -1/2*tendonlength_extensor_joint6(theta_joint6)

def tendonlength_extensor_joint6(theta_joint6):
   '''Input: joint angle of joint6 (pp-mp Finger3) in rad
      Output: total normal lengths of extensor tendon through joint6'''
   delta_x = 0
   delta_y = radius*sin(theta_joint6)+2*radius*cos(theta_joint6*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint6)-1) + 2*radius*sin(theta_joint6*0.5)

   return 2*np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

#Finger 4:
def tendonlength_flexor_joint7(theta_joint7):
   '''Input: joint angle of joint7 (base-pp Finger4) in rad
      Output: total normal lengths of flexor tendon through joint7'''
   return -tendonlength_extensor_joint7(theta_joint7)

def tendonlength_extensor_joint7(theta_joint7):
   '''Input: joint angle of joint7 (base-pp Finger4) in rad
      Output: total normal lengths of extensor tendon through joint7'''
   delta_x = 0
   delta_y = radius*sin(theta_joint7)+2*radius*cos(theta_joint7*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint7)-1) + 2*radius*sin(theta_joint7*0.5)
   
   return np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

def tendonlength_flexor_joint8(theta_joint8):
   '''Input: joint angle of joint8 (pp-mp Finger4) in rad
      Output: total normal lengths of flexor tendon through joint8'''
   return -1/2*tendonlength_extensor_joint8(theta_joint8)

def tendonlength_extensor_joint8(theta_joint8):
   '''Input: joint angle of joint8 (pp-mp Finger4) in rad
      Output: total normal lengths of extensor tendon through joint8'''
   delta_x = 0
   delta_y = radius*sin(theta_joint8)+2*radius*cos(theta_joint8*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint8)-1) + 2*radius*sin(theta_joint8*0.5)

   return 2*np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

#Finger 5:
def tendonlength_flexor_joint9(theta_joint9):
   '''Input: joint angle of joint9 (base-pp Finger5) in rad
      Output: total normal lengths of flexor tendon through joint9'''
   return -tendonlength_extensor_joint9(theta_joint9)

def tendonlength_extensor_joint9(theta_joint9):
   '''Input: joint angle of joint9 (base-pp Finger5) in rad
      Output: total normal lengths of extensor tendon through joint9'''
   delta_x = 0
   delta_y = radius*sin(theta_joint9)+2*radius*cos(theta_joint9*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint9)-1) + 2*radius*sin(theta_joint9*0.5)
   
   return np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

def tendonlength_flexor_joint10(theta_joint10):
   '''Input: joint angle of joint10 (pp-mp Finger5) in rad
      Output: total normal lengths of flexor tendon through joint10'''
   return -1/2*tendonlength_extensor_joint10(theta_joint10)

def tendonlength_extensor_joint10(theta_joint10):
   '''Input: joint angle of joint10 (pp-mp Finger5) in rad
      Output: total normal lengths of extensor tendon through joint10'''
   delta_x = 0
   delta_y = radius*sin(theta_joint10)+2*radius*cos(theta_joint10*0.5)-2*radius
   delta_z = -radius*(cos(theta_joint10)-1) + 2*radius*sin(theta_joint10*0.5)

   return 2*np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
"""


# ------------------- Calculations of Tendon Lengths for all joints ------------------- #
# TODO: Add your own functions here to calculate the tendon lengths for all joints and for each finger (if needed)

def pose2tendon_thumb(theta_joint0, theta_joint1, theta_joint2):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_right_joint0(theta_joint0),
            tendonlength_left_joint0(theta_joint0),
            tendonlength_flexor_joint1(theta_joint0, theta_joint1),
            tendonlength_extensor_joint1(theta_joint0, theta_joint1),
            tendonlength_flexor_joint2(theta_joint0, theta_joint2),
            tendonlength_extensor_joint2(theta_joint0, theta_joint2)]

def pose2tendon_finger(theta_joint3, theta_joint4):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_flexor_joint3(theta_joint3),
            tendonlength_extensor_joint3(theta_joint3),
            tendonlength_flexor_joint4(theta_joint4),
            tendonlength_extensor_joint4(theta_joint4)]
"""
def pose2tendon_finger3(theta_joint5, theta_joint6):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_flexor_joint5(theta_joint5),
            tendonlength_extensor_joint5(theta_joint5),
            tendonlength_flexor_joint6(theta_joint6),
            tendonlength_extensor_joint6(theta_joint6)]

def pose2tendon_finger4(theta_joint7, theta_joint8):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_flexor_joint7(theta_joint7),
            tendonlength_extensor_joint7(theta_joint7),
            tendonlength_flexor_joint8(theta_joint8),
            tendonlength_extensor_joint8(theta_joint8)]

def pose2tendon_finger5(theta_joint9, theta_joint10):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_flexor_joint9(theta_joint9),
           tendonlength_extensor_joint9(theta_joint9),
           tendonlength_flexor_joint10(theta_joint10),
           tendonlength_extensor_joint10(theta_joint10)]
"""