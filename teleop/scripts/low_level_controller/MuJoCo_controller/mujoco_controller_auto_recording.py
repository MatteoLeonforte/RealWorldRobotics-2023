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



def flex_finger(finger: int, gc: GripperControllerMujocoSim, angle: float, finger_dict: dict):
  pass




# Main function

if __name__ == '__main__':

  gc = GripperControllerMujocoSim()

  # Read file
  file_path = '/home/matteo/isaac_ws_RWR/Teleoperation/RealWorldRobotics-2023/motor_control_RL/recorded_policies/Sphere_UP_XNEG_15_scaled.npy'
  data = np.load(file_path)
  # (num_envs, num_steps, num_actions) -> (2, # , 11)
  cmd_sequence = data[0, :, :]
  cmd_sequence_len = cmd_sequence.shape[0]

  finger_dict = {
    'thumb': [0,1,2],
    'index': [3,4],
    'middle': [5,6],
    'ring': [7,8],
    'pinky': [9,10]
  }

  target = math.pi/2
  angles = [0]*11

  # Scale the cmd_sequence to the correct range

  for i in range(cmd_sequence_len):
    gc.command_joint_angles(cmd_sequence[i,:])
    time.sleep(0.5)
