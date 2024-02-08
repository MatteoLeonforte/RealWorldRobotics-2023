import time

import mujoco
import mujoco.viewer
import math
import threading
import numpy as np


class GripperControllerMujocoSim:
  """A class that runs a MuJoCo simulation, sending joint commands to the gripper"""

  def __init__(self)->None:
    self.m = mujoco.MjModel.from_xml_path('/home/matteo/isaac_ws_RWR/Teleoperation/RealWorldRobotics-2023/design/hand_design.xml')
    self.d = mujoco.MjData(self.m)
    self.simulation_thread = threading.Thread(target=self.run_viewer)
    self.lock = threading.Lock()
    self.sim_running = False
    self.start_simulation()


  def run_viewer(self):

    with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
      while viewer.is_running() and self.sim_running:
        with self.lock:
          mujoco.mj_step(self.m, self.d)
          viewer.sync()
          time.sleep(0.01)
      viewer.close()

  def start_simulation(self):
    self.sim_running = True
    self.simulation_thread.start()

  def stop_simulation(self):
    self.sim_running = False
    self.simulation_thread.join()
  
  def init_joints(self, calibrate=False):
    pass

  def command_joint_angles(self, angles):
    with self.lock:
      self.d.ctrl[:] = angles


class Hand:
  def __init__(self, gc:GripperControllerMujocoSim) -> None:
    self.gc = gc
    self.plate = Joint(0, 0.,-60.0, 60.0)
    self.fingers = {
      'thumb': Finger('thumb', [1,2]),
      'index': Finger('index', [3,4]),
      'middle': Finger('middle', [5,6]),
      'ring': Finger('ring', [7,8]),
      'pinky': Finger('pinky', [9,10])
    }
    self.angles = [0]*11

  # Move the plate
  def move_plate(self, angle=0):
    self.plate.angle = angle

  def plate_center(self):
    self.move_plate(0)
  def plate_left(self):
    self.move_plate(-60)
  def plate_right(self):
    self.move_plate(60)

  

  # Move the fingers
  def move_finger(self, finger_name, angle):

    assert finger_name in self.fingers, "Finger not found"
    self.fingers[finger_name].move(angle)

  def open_all_fingers(self, angle=0):
    for finger in self.fingers.values():
      finger.move([angle,angle])       
  def close_all_fingers(self):
    for finger in self.fingers.values():
      finger.close()
  
  def close_finger(self, finger_name):
    assert finger_name in self.fingers, "Finger not found"
    self.fingers[finger_name].close()
  def open_finger(self, finger_name):
    assert finger_name in self.fingers, "Finger not found"
    self.fingers[finger_name].open()

  #Update simulation
  def _update(self):
    self.angles[0] = self.plate.angle
    for finger in self.fingers.values():
      for joint in finger.joints:
        self.angles[joint.idx] = joint.angle

    # I want to clip the angles to the joint limits
    min_angles = [-60, -45,0,-45,0,-45,0,-45,0,-45,0]
    max_angles = [60, 90,90,90,90,90,90,90,90,90,90]

    angles = np.clip(self.angles, min_angles, max_angles)
    angles = np.deg2rad(angles)/2
    self.gc.command_joint_angles(angles)
    time.sleep(1)

class Joint:
  def __init__(self, idx:int, angle:float, min:float, max:float):
    self.idx = idx
    self.min = min
    self.max = max
    self.angle = angle

class Finger:
  def __init__(self, name: str, joints:[int]):

    assert len(joints) == 2, "Finger must have 2 joints"  
    self.name = name
    self.joints= []
    
    # Lower
    self.joints.append(Joint(joints[0], 0., -45.0, 90.0))  
    # Upper
    self.joints.append(Joint(joints[1], 0., 0.0, 90.0))
    

  def open(self):
    self.joints[0].angle = 0.
    self.joints[1].angle = 0.

  def close(self):
    self.joints[0].angle = 90.
    self.joints[1].angle = 90.

  def move(self, angles):
    
    assert len(angles) == 2, "Finger has 2 joints"
    assert angles[0] >= -45 and angles[0] <= 90, "Angle must be between -45 and 90"
    assert angles[1] >= 0 and angles[1] <= 90, "Angle must be between 0 and 90"

    self.joints[0].angle = angles[0]
    self.joints[1].angle = angles[1]

