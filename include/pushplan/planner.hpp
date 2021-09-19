#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/types.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/bullet_sim.hpp>
#include <pushplan/ObjectsPoses.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class Planner
{
public:
	Planner() : m_num_agents(-1), m_ooi_idx(-1),
				m_t(0), m_phase(0), m_ph("~") {};
	bool Init(const std::string& scene_file, int scene_id);

	bool Plan();
	bool SaveData();
	bool Alive();
	bool Rearrange();
	std::uint32_t RunSim();
	bool TryExtract();
	void AnimateSolution();

	const std::vector<Object>* GetObject(const LatticeState& s, int priority);
	const std::vector<Object>* GetGraspObjs() {
		return m_robot->GetGraspObjs();
	}

	const std::vector<Object>* GetOOIObject() { return m_ooi.GetObject(); };
	const LatticeState* GetOOIState() { return m_ooi.GetCurrentState(); };

private:
	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Robot> m_robot;
	std::shared_ptr<BulletSim> m_sim;

	int m_num_agents, m_ooi_idx, m_t, m_phase, m_scene_id;
	std::vector<Agent> m_agents;
	std::unordered_map<int, size_t> m_agent_map;
	Agent m_ooi, m_ee;
	Coord m_ooi_g;
	State m_ooi_gf;
	std::vector<double> m_goal;

	Trajectory m_exec;
	std::vector<trajectory_msgs::JointTrajectory> m_rearrangements;
	pushplan::ObjectsPoses m_rearranged;

	std::vector<size_t> m_priorities;

	ros::NodeHandle m_ph, m_nh;
	ros::ServiceServer m_simulate, m_animate, m_rearrange;
	std::uint32_t m_violation;

	std::map<std::string, double> m_stats;
	double m_plan_budget, m_sim_budget;

	bool whcastar();

	bool setupSim();
	bool runSim(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
	bool animateSolution(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
	bool rearrange(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

	void reinit();
	void prioritize();
	void step_agents(int k=1);

	bool setupProblem();
	void updateAgentPositions(
		const pushplan::ObjectsPoses& result,
		pushplan::ObjectsPoses& rearranged);
	int cleanupLogs();

	void parse_scene(std::vector<Object>& obstacles);
	void writePlanState(int iter);
	void setupGlobals();
	int armId();

	bool savePlanData();

};

} // namespace clutter


#endif // PLANNER_HPP
