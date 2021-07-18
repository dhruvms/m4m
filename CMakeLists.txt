cmake_minimum_required(VERSION 3.5.1)
project(pushplan)

if(NOT CMAKE_BUILD_TYPE)
    # set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Release)
endif()
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++14 -fmax-errors=5")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fmax-errors=5")


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  leatherman
  message_generation
  roscpp
  rospy
  sbpl_collision_checking
  sbpl_kdl_robot_model
  smpl_ompl_interface
  tf
  visualization_msgs
)
find_package(smpl REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_library(
    PUSHPLAN
    src/planner.cpp
    src/collision_checker.cpp
    src/agent.cpp
    src/wastar.cpp
    src/constants.cpp)
target_link_libraries(PUSHPLAN ${catkin_LIBRARIES} smpl::smpl)

add_executable(coop_plan test/coop_plan.cpp)
target_link_libraries(coop_plan ${catkin_LIBRARIES} smpl::smpl PUSHPLAN)
