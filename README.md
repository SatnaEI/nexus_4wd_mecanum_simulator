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

move or delete <ws>/src/navigation_msgs in order to install an up-to-date version

$ https://github.com/ros-planning/navigation_msgs

$ git clone https://github.com/ros-planning/navigation



5: Install explore-lite

move or delete <ws>/src/vision_opencv in order to install an up-to-date version

$ git clone --branch noetic https://github.com/ros-perception/vision_opencv



6: Install RPlidar

$ git clone https://github.com/Slamtec/rplidar_ros



7: Install Rosserial

$git clone https://github.com/ros-drivers/rosserial



8: Configure camera

$pip3 install pypng




Topic for laser scan: /scan


After each git clone .... : 
$ catkin_make
if any package does not appears with roslaunch xx_package_xx ... or rosrun xx_package_xx ..., run
$rospack find xx_package_xx

