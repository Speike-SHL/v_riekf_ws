cmake_minimum_required(VERSION 3.0.2)
project(v_riekf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_BIND_GLOBAL_PLACEHOLDERS") #消除boost库的警告

#--------------------------- 加速程序运行 --------------------------
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(Eigen3 REQUIRED)

# find_package(CUDA REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v_riekf
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/InEKF
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${CUDA_INCLUDE_DIRS}
)

set(SRC_FILES
  src/InEKF/InEKF.cpp
  src/InEKF/LieGroup.cpp
  src/InEKF/NoiseParams.cpp
  src/InEKF/RobotState.cpp)

add_executable(v_riekf_node 
  src/v_riekf_node.cpp
  ${SRC_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(v_riekf_node
  ${catkin_LIBRARIES}
)

