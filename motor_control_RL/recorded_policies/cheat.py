import numpy as np

file_path = "/home/matteo/isaac_ws_RWR/Teleoperation/RealWorldRobotics-2023/motor_control_RL/recorded_policies/Sphere_UP_XPOS_15_scaled.npy"

# Load the npy file
data = np.load(file_path)
data = data[0,:,:]
data[:,1] += np.deg2rad(3)
data[:,2] += np.deg2rad(3)

# Print the contents of the npy file
#print(data)

# Save the modified data in a new file
new_file_path = "/home/matteo/isaac_ws_RWR/Teleoperation/RealWorldRobotics-2023/motor_control_RL/recorded_policies/Sphere_UP_XPOS_15_cheat3.npy"
np.save(new_file_path, data)