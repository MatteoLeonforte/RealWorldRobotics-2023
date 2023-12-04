import time

import mujoco
import mujoco.viewer
import math

m = mujoco.MjModel.from_xml_path('/home/matteo/isaac_ws_RWR/faive_gym_oss/assets/rot_thumb_hand/hand_rotatingThumb.xml')
d = mujoco.MjData(m)



finger_tested = 1
direction = 1

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # Testing pattern
    curr_act = math.degrees(d.ctrl[finger_tested])
    if curr_act>=90:
        d.ctrl[finger_tested]=0
        finger_tested = finger_tested+1 if finger_tested<10 else 1
        
    
    else:
      d.ctrl[finger_tested]+=math.radians(2)

    
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)