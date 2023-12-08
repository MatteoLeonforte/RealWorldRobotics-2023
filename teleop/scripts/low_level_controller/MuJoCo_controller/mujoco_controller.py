import time

import mujoco
import mujoco.viewer
import math
import threading
import numpy as np
import os


class GripperControllerMujocoSim:
  """A class that runs a MuJoCo simulation, sending joint commands to the gripper"""

  def __init__(self)->None:
    self.base_path = os.path.dirname(os.path.realpath(__file__))
    self.xml_path = self.base_path + "/../../../../design/hand_design.xml"

    self.m = mujoco.MjModel.from_xml_path(self.xml_path) # ADAPT HERE
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
      print("commanding angles", angles)
      self.d.ctrl[:] = angles