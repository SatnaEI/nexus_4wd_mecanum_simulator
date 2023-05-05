# robotSAR
Repository for the Search And Rescue robot for the Laboratorio course

How to set up the raspberry properly from newly flashed SD card with Debian Buster

1: follow this tutorial to install ROS on the raspberry 

https://varhowto.com/install-ros-noetic-raspberry-pi-4/

From there all packages must be installed from source because apt cannot locate the correct repositories, even when expanded with contrib and non-free

$cd <ws>/src


2: Install the project repository

$ git clone https://github.com/SatnaEI/nexus_4wd_mecanum_simulator/tree/onRaspi

Make sure to switch to the onRaspi branch

$ catkin_make


3: Install gmapping for SLAM algorithm

$ git clone https://github.com/ros-perception/slam_gmapping.git

$ git clone https://github.com/ros-perception/openslam_gmapping

$ catkin_make


4: Install the navegation stack

move or delete <ws>/src/geomtetry2 in order to install an up-to-date version

Add CATKIN_IGNORE in <ws>/src/geometry

$ git clone https://github.com/ros/geometry2

$ sudo apt-get install libbullet-dev

$ sudo apt-get install libsdl-image1.2-dev 

$ sudo apt-get install libsdl-dev

$ https://github.com/ros-planning/navigation_msgs

$ git clone https://github.com/ros-planning/navigation





Topic for laser scan: /scan


Dependencies: 
- $sudo apt-get install ros-noetic-gmapping
- https://github.com/hrnr/m-explore
- git clone https://github.com/ros-drivers/rosserial.git

$catkin_make
if any package does not appears with roslaunch xx_package_xx ... or rosrun xx_package_xx ..., run
$rospack find xx_package_xx


To launch the simulation: 

1: $roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_house_world.launch 

2: $roslaunch nexus_4wd_mecanum_gazebo navegation_test.launch

3: $rosrun rviz rviz

4: load show.rviz into rviz

5: $roslaunch nexus_4wd_mecanum_gazebo move_base_launch.launch 

