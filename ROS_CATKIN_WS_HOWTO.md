

1. Create a project folder e.g. RWR

```bash
mkdir RWR
cd RWR
```

2. In the RWR folder: 
   1. source ros 
   2. then create the catkin workspace (we need sudo apt-get install python3-catkin-tools for this installed)

```bash
source /opt/ros/noetic/setup.bash 
mkdir -p catkin_ws_rwr/src
cd catkin_ws_rwr/
catkin_make
```

3.  symlink the teleoperation folder into catkin_ws_rwr/src
4.  It is structured like this: `ln -s /path/to/source /path/to/destination`

```bash
ln -s ~/RWR/RealWorldRobotics-2023/teleop ~/RWR/catkin_ws_rwr/src/teleop
```





Now when you work with ros you wan to source it **every time** before using ros

```bash
cd ~/RWR/catkin_ws_rwr
source devel/setup.bash

```
