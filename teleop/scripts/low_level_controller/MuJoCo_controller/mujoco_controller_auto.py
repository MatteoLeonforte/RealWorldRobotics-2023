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




# Main function

if __name__ == '__main__':

  gc = GripperControllerMujocoSim()
  target = math.pi/2
  angles = [0]*11

  while True:
    
    angles[0] = target
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[0] = -target
    gc.command_joint_angles(angles)
    time.sleep(1)


    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[1] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles[2] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[3] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles[4] = target
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[5] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles[6] = target
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[7] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles[8] = target
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles[9] = target
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)
    angles[10] = target
    gc.command_joint_angles(angles)
    time.sleep(1)

    angles = [0]*11
    gc.command_joint_angles(angles)
    time.sleep(1)