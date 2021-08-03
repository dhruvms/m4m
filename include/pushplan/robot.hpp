#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <pushplan/types.hpp>
#include <pushplan/movable.hpp>

#include <ros/ros.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <moveit_msgs/RobotState.h>

#include <string>
#include <memory>
#include <random>

namespace clutter
{

class Robot : public Movable
{
public:
	Robot() : m_ph("~"), m_rng(m_dev()) {};

	bool Init() override;

	bool AtGoal(const LatticeState& s, bool verbose=false) override;
	void Step(int k) override;

	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs) override;

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetGoalHeuristic(const LatticeState& s) override;

	const std::vector<Object>* GetObject(const LatticeState& s) override;
	using Movable::GetObject;

private:
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::KDLRobotModel> m_rm;
	moveit_msgs::RobotState m_start_state;

	double m_mass, m_b, m_z;
	std::string m_shoulder, m_elbow, m_wrist, m_tip;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	Trajectory m_retrieve; // use TBD

	bool setIKState(
		const Coord& loc, const State& seed, State& state);

	int generateSuccessor(
		const LatticeState* parent,
		int jidx, int delta,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2) override;
	bool convertPath(
		const std::vector<int>& idpath) override;

	// Robot Model functions
	bool readRobotModelConfig(const ros::NodeHandle &nh);
	bool setupRobotModel();
	bool readStartState();
	bool setReferenceStartState();

	void initObjects();
	void reinitObjects(const State& s);
};

} // namespace clutter


#endif // ROBOT_HPP
