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
	message_generation
)
find_package(smpl REQUIRED)

add_service_files(
	FILES
	AddObject.srv
	AddYCBObject.srv
	AddRobot.srv
	ResetSimulation.srv
	CheckScene.srv
	ResetScene.srv
	ResetArm.srv
	SetRobotState.srv
	SetColours.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_library(
	PUSHPLAN
	src/planner.cpp
	src/collision_checker.cpp
	src/movable.cpp
	src/bullet_sim.cpp
	src/agent.cpp
	src/robot.cpp
	src/wastar.cpp
	src/constants.cpp)
target_link_libraries(PUSHPLAN ${catkin_LIBRARIES} smpl::smpl) # -lprofiler

add_executable(coop_plan test/coop_plan.cpp)
target_link_libraries(coop_plan ${catkin_LIBRARIES} smpl::smpl PUSHPLAN)
