cmake_minimum_required(VERSION 3.5)

# Set the project name
project("trajectories-totg")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  trajectory_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES 
#   CATKIN_DEPENDS message_runtime
#  DEPENDS 
)

# Add source files
set(SOURCES
    src/Path.cpp
    src/Trajectory.cpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


# Create Example executable
add_executable(example src/examples/Example.cpp ${SOURCES})
target_link_libraries(example Eigen3::Eigen)
target_include_directories(example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


add_executable(example2 src/examples/Example2.cpp ${SOURCES})
target_link_libraries(example2 Eigen3::Eigen)
target_include_directories(example2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/plot.py  
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch #config urdf worlds trees
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS example2
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)