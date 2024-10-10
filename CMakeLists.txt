cmake_minimum_required(VERSION 3.5)

# Set the project name
project("TEST")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add source files
set(SOURCES
    Path.cpp
    Trajectory.cpp
)

# Create Example executable
add_executable(example Example.cpp ${SOURCES})
target_link_libraries(example Eigen3::Eigen)
target_include_directories(example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


add_executable(example2 Example2.cpp ${SOURCES})
target_link_libraries(example2 Eigen3::Eigen)
target_include_directories(example2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Copy plot.py to the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/plot.py
               ${CMAKE_CURRENT_BINARY_DIR}/plot.py
               COPYONLY)

# Copy waypoints.csv to the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/waypoints.csv
               ${CMAKE_CURRENT_BINARY_DIR}/waypoints.csv
               COPYONLY)

# Print a message to confirm the copy operations
message(STATUS "plot.py and waypoints.csv will be copied to the build directory")

# Create Test executable
#add_executable(Test Test.cpp ${SOURCES})
#target_link_libraries(Test Eigen3::Eigen)

# Include directories
#target_include_directories(Test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
