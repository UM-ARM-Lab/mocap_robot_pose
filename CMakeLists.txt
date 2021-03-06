cmake_minimum_required(VERSION 2.8.3)
project(mocap_robot_pose)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf lightweight_vicon_bridge)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf lightweight_vicon_bridge DEPENDS Eigen3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} SYSTEM ${EIGEN3_INCLUDE_DIR})

## Set build flags for working with OpenMP
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

## Declare a cpp library
# add_library(mocap_robot_pose
#   src/${PROJECT_NAME}/mocap_robot_pose.cpp
# )

## Declare a cpp executable
add_executable(mocap_robot_pose_node src/mocap_robot_pose_node.cpp)
add_dependencies(mocap_robot_pose_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mocap_robot_pose_node ${catkin_LIBRARIES})

## Simple mocap to odom_combined publisher
add_executable(mocap_robot_base_pub src/mocap_robot_base_pub.cpp)
add_dependencies(mocap_robot_base_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(mocap_robot_base_pub ${catkin_LIBRARIES})

## Simple mocap to arm pose publisher
add_executable(mocap_robot_arm_pub src/mocap_robot_arm_pub.cpp)
add_dependencies(mocap_robot_arm_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(mocap_robot_arm_pub ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mocap_robot_pose mocap_robot_pose_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mocap_robot_pose.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
