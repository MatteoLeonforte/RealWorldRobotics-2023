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
    gc.start_simulation()

  def run_viewer(self):

    with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
      while viewer.is_running():
        with self.lock:
          mujoco.mj_step(self.m, self.d)
          viewer.sync()
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
      self.d.ctrl[:] = angles