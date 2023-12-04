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

  def run_viewer(self):

    with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
      while viewer.is_running():
        with self.lock:
          mujoco.mj_step(gc.m, gc.d)
          viewer.sync()
          #print("Running simulation")
          time.sleep(0.01)

  def start_simulation(self):
    self.simulation_thread.start()

  def stop_simulation(self):
    self.viewer.close()
    self.simulation_thread.join()
  
  def init_joints(self, calibrate=False):
    pass

  def command_joint_angles(self, angles):
    with self.lock:
      print(f"Received joint angles: {angles}")
      self.d.ctrl[:] = angles

    pass



if __name__ == "__main__":
  gc = GripperControllerMujocoSim()
  print("Initialized GripperControllerMujocoSim")
  gc.start_simulation()
  print("Started simulation")
  
  # Send a sequence of commands to the gripper
  num_actuators = 11
  steps = np.linspace(0,math.pi/2,100)
  s = 0
  
  while True:
    # Send a command to the gripper to move all actuators from 0 to 90 degrees and back to 0 degrees with a 0.5 second pause in between
    while s<100:
      angles = [steps[s]]*num_actuators
      s = s+1
      gc.command_joint_angles(angles)
      mujoco.mj_step(gc.m, gc.d)
      time.sleep(0.1)
      
    while s>0:
      angles = [steps[s]]*num_actuators
      s = s-1
      gc.command_joint_angles(angles)
      mujoco.mj_step(gc.m, gc.d)
      time.sleep(0.1)
