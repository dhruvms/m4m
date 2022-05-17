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
	roscpp
	rospy
	leatherman
	moveit_msgs
	sbpl_collision_checking
	sbpl_kdl_robot_model
	smpl_ompl_interface
	visualization_msgs
	tf
	std_msgs
	sensor_msgs
	trajectory_msgs
	comms
)
find_package(smpl REQUIRED)
find_package(comms REQUIRED)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
	comms
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_library(
	PUSHPLAN
	src/search/planner.cpp
	src/utils/bullet_sim.cpp
	src/search/cbs_nodes.cpp
	src/search/cbs.cpp
	src/search/cbswp.cpp
	src/search/pbs.cpp
	src/search/wastar.cpp
	src/search/focal.cpp
	src/sampling/node.cpp
	src/sampling/sampling_planner.cpp
	src/sampling/rrt.cpp
	src/sampling/rrtstar.cpp
	src/utils/collision_checker.cpp
	src/agents/object.cpp
	src/agents/agent_lattice.cpp
	src/agents/agent.cpp
	src/agents/robot.cpp
	src/utils/constants.cpp)
target_link_libraries(PUSHPLAN ${catkin_LIBRARIES} smpl::smpl) # -lprofiler

add_executable(coop_plan test/coop_plan.cpp)
target_link_libraries(coop_plan ${catkin_LIBRARIES} smpl::smpl PUSHPLAN)

add_executable(sampling test/sampling.cpp)
target_link_libraries(sampling ${catkin_LIBRARIES} smpl::smpl PUSHPLAN)
