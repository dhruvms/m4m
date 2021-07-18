#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/types.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class Planner
{
public:
	Planner(const std::string& scene_file);

	void WHCAStar();

	const Object* GetObject(int priority);

private:
	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;

	int m_num_agents, m_ooi, m_t;
	std::vector<Agent> m_agents;
	Agent m_extract;
	Point m_extract_g;
	Pointf m_extract_gf;

	std::vector<size_t> m_priorities;

	void parse_scene(std::vector<Object>& obstacles);

	void reinit();
	void prioritize();
	void step_agents(int k=1);

	void writePlanState(int iter);
};

} // namespace clutter


#endif // PLANNER_HPP
