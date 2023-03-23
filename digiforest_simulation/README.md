# Digiforest Simulation
This repository contains the robot models, worlds, and entities that shall be used for the DIGIFOREST Project.

## NOTE: Extra world and model support is in development and is being added to the repository. 


## Instructions for installation and use:

1. Install Ignition Gazebo from [here](https://gazebosim.org/docs/garden/install).
2. Make a catkin workspace for ROS 1.

```sh
mkdir -p ~/digiforest_ws/src && cd ~/digiforest_ws/src
git clone git@github.com:ntnu-arl/digiforest_simulation.git
cd ~/digiforest_ws/
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

3. Download the mesh files for simulation with the link, https://syncandshare.lrz.de/getlink/fiKze8tooFHm9pNDaEAfqR/rubicon.dae. And put this under /Rubicon/mesh

4. To run an empty world with the robot

```sh
source ~/digiforest_ws/devel/setup.bash
roslaunch digiforest_simulation spawn_world.launch
```

5. In another terminal, spawn the robot. This spawns the robot in the simulator and also spawns the ROS1 bridge to convert topics from native a gazebo message format to ROS1 message formats. The default robot is RMF_OWL with an RGB camera, a depth camera and a 3D lidar.
```sh
source ~/digiforest_ws/devel/setup.bash
roslaunch digiforest_simulation spawn_robot.launch
```


6. Velocity commands to the robot can be given using rostopics.
```sh
rostopic pub /rmf_owl/cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.5
  z: 0.5
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```


6. Teleop in the GUI of Ignition Gazebo can be used to control the robot
![teleop](https://user-images.githubusercontent.com/44289986/209318521-1e77c7ae-185c-43e4-850b-338ed52fc0ca.png)
