#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <pushplan/types.hpp>

#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>

#include <string>
#include <memory>

namespace clutter
{

class Robot
{
public:
	Robot() : m_ph("~") {};

	bool Init();

private:
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::KDLRobotModel> m_rm;

	moveit_msgs::RobotState m_start_state;

	bool readRobotModelConfig(const ros::NodeHandle &nh);
	bool setupRobotModel();
	bool readStartState();
	bool setReferenceStartState();
};

} // namespace clutter


#endif // ROBOT_HPP
