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

	bool Setup() override;
	bool Init() override;
	bool RandomiseStart();

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

	Coord GetEECoord();
	const moveit_msgs::RobotState* GetStartState() { return &m_start_state; };

private:
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::KDLRobotModel> m_rm;
	moveit_msgs::RobotState m_start_state;

	// cached from robot model
	std::vector<double> m_min_limits;
	std::vector<double> m_max_limits;
	std::vector<bool> m_continuous;
	std::vector<bool> m_bounded;

	std::vector<int> m_coord_vals;
	std::vector<double> m_coord_deltas;

	double m_mass, m_b, m_table_z;
	std::string m_shoulder, m_elbow, m_wrist, m_tip;
	const smpl::urdf::Link* m_link_s = nullptr;
	const smpl::urdf::Link* m_link_e = nullptr;
	const smpl::urdf::Link* m_link_w = nullptr;
	const smpl::urdf::Link* m_link_t = nullptr;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	void getRandomState(smpl::RobotState& s);
	bool reinitStartState();

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

	void coordToState(const Coord& coord, State& state) const;
	void stateToCoord(const State& state, Coord& coord) const;

	// Robot Model functions
	bool readRobotModelConfig(const ros::NodeHandle &nh);
	bool setupRobotModel();
	bool readStartState();
	bool setReferenceStartState();
	bool readResolutions(std::vector<double>& resolutions);

	void initObjects();
	void reinitObjects(const State& s);
};

} // namespace clutter


#endif // ROBOT_HPP
