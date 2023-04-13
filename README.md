# robotSAR
Repository for the Search And Rescue robot for the Laboratorio course

Dependencies: 
- $sudo apt-get install ros-noetic-gmapping
- https://github.com/hrnr/m-explore


To launch the simulation: 

1: $roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_house_world.launch 

2: $roslaunch nexus_4wd_mecanum_gazebo navegation_test.launch

3: $rosrun rviz rviz

4: load show.rviz into rviz

5: $roslaunch nexus_4wd_mecanum_gazebo move_base_launch.launch 

