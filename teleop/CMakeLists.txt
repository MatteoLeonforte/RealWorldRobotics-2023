# Required CMake Version
cmake_minimum_required(VERSION 3.9)

# Package Name
project(teleop)

# Find Catkin
find_package(catkin REQUIRED)
catkin_package()

# Install Python Scripts
catkin_install_python(PROGRAMS 
  src/gripper_controller_node.py 
  src/mano_faive_retargeter.py 
  src/oakd_hand_tracker_ingress.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)