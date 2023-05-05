cmake_minimum_required(VERSION 3.0.2)
project(nexus_4wd_mecanum_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #gazebo-dev
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  message_generation
)




catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_library(nexus_ros_force_based_move src/nexus_ros_force_based_move.cpp)
#target_link_libraries(nexus_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(tf_broadcaster src/tf_broadcaster.py)
#add_library(nexus_old src/nexus_old.cpp)
#target_link_libraries(nexus_old ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(talker src/talker.cpp)

target_link_libraries(talker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#install(PROGRAMS
#    src/tf_broadcaster.py
##   scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

catkin_install_python(PROGRAMS
  src/tf_broadcaster.py
  src/cameraRead.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

