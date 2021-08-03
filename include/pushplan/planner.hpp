#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/types.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/robot.hpp>

#include <ros/ros.h>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class Planner
{
public:
	Planner(const std::string& scene_file);

	void Plan();

	const std::vector<Object>* GetObject(const LatticeState& s, int priority);

	const std::vector<Object>* GetOOIObject() { return m_ooi.GetObject(); };
	const LatticeState* GetOOIState() { return m_ooi.GetCurrentState(); };

private:
	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Robot> m_robot;

	int m_num_agents, m_ooi_idx, m_t, m_phase;
	std::vector<Agent> m_agents;
	Agent m_ooi, m_ee;
	Coord m_ooi_g;
	State m_ooi_gf;

	std::vector<size_t> m_priorities;

	ros::NodeHandle m_ph;

	bool whcastar();

	void reinit();
	void prioritize();
	void step_agents(int k=1);

	void setupProblem(bool random=false);
	int cleanupLogs();

	void parse_scene(std::vector<Object>& obstacles);
	void writePlanState(int iter);
	void setupGlobals();
};

} // namespace clutter


#endif // PLANNER_HPP
