#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/agents/agent.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/utils/bullet_sim.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/types.hpp>
#include <comms/ObjectsPoses.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class CBS;

class Planner
{
public:
	Planner() : m_num_objs(-1), m_scene_id(-1),
				m_ph("~") {};
	bool Init(const std::string& scene_file, int scene_id, bool ycb);
	bool Alive();
	bool SetupNGR();

	bool Plan();
	bool SaveData();
	bool Rearrange();
	std::uint32_t RunSim();
	bool TryExtract();
	void AnimateSolution();

	Agent* GetAgent(const int& id) {
		assert(id > 0); // 0 is robot
		return m_agents.at(m_agent_map[id]).get();
	}

	const std::vector<Object>* Get2DRobot(const LatticeState& s) {
		return m_robot->GetObject(s);
	}
	const Object* GetObject(int id) {
		return m_agents.at(m_agent_map[id])->GetObject();
	};

	bool CheckRobotCollision(Agent* a, const LatticeState& robot_state, int t, bool process)
	{
		if (!process) {
			return m_robot->CheckCollision(robot_state, t);
		}
		return m_robot->CheckCollisionWithObject(robot_state, a, t);
	}

	std::vector<Agent*> GetAllAgents()
	{
		std::vector<Agent*> all_agents;
		for(int i = 0; i < m_agents.size(); i++) {
			all_agents.push_back(m_agents[i].get());
		}
		return all_agents;
	}

private:
	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;
	std::shared_ptr<Robot> m_robot;
	std::shared_ptr<BulletSim> m_sim;
	std::shared_ptr<CBS> m_cbs;

	int m_num_objs, m_scene_id;
	std::vector<std::shared_ptr<Agent> > m_agents;
	std::shared_ptr<Agent> m_ooi;
	std::unordered_map<int, size_t> m_agent_map;
	Coord m_ooi_g;
	State m_ooi_gf;
	std::vector<double> m_goal;

	trajectory_msgs::JointTrajectory m_exec;
	std::vector<trajectory_msgs::JointTrajectory> m_rearrangements;
	comms::ObjectsPoses m_rearranged;

	std::vector<size_t> m_priorities;

	ros::NodeHandle m_ph, m_nh;
	std::uint32_t m_violation;

	std::map<std::string, double> m_stats;
	double m_plan_time, m_plan_budget, m_sim_budget, m_total_budget;

	bool createCBS();

	bool runSim();
	bool animateSolution();
	bool rearrange();

	bool setupProblem(bool backwards);
	void updateAgentPositions(
		const comms::ObjectsPoses& result,
		comms::ObjectsPoses& rearranged);
	int cleanupLogs();

	void init_agents(
		bool ycb, std::vector<Object>& obstacles);
	void parse_scene(std::vector<Object>& obstacles);
	void writePlanState(int iter);
	void setupGlobals();
	int armId();

	bool savePlanData();

};

} // namespace clutter


#endif // PLANNER_HPP
