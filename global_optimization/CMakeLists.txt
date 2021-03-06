cmake_minimum_required(VERSION 2.8.3)
project(global_optimization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs xform_utils cwru_opencv_common tool_model_optimization cwru_davinci_interface cwru_davinci_kinematics )

find_package(OpenCV REQUIRED)

#include_directories(include ${catkin_INCLUDE_DIRS} ${tool_model_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs xform_utils cwru_opencv_common tool_model_optimization cwru_davinci_interface cwru_davinci_kinematics)
# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pg -Q" )
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg -Q")

# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg -Q")

# Libraries: uncomment the following and edit arguments to create a new library
# add_library(my_lib src/my_lib.cpp)   

add_library(optimization_calibration
          src/optimization_calibration.cpp
)

add_executable(optimization_calibration_main src/optimization_calibration_main.cpp)
add_executable(generate_dataset src/generate_dataset.cpp)
add_executable(check_video src/check_video.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)
target_link_libraries(optimization_calibration tool_model davinci_kinematics )
target_link_libraries(optimization_calibration_main optimization_calibration ${catkin_LIBRARIES} )
target_link_libraries(generate_dataset optimization_calibration ${catkin_LIBRARIES} )
target_link_libraries(check_video optimization_calibration ${catkin_LIBRARIES} )