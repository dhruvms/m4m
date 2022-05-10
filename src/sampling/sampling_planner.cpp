#include <pushplan/sampling/sampling_planner.hpp>

namespace clutter {
namespace sampling {

bool SamplingPlanner::Solve()
{
	if (m_start == nullptr || !m_robot)
	{
		SMPL_ERROR("Start state and robot must be set!");
		return false;
	}

	return true;
}

void SamplingPlanner::SetStartState(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects)
{
	m_start = new Node(state, objects);
}

void SamplingPlanner::SetGoalState(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects)
{
	m_goal = new Node(state, objects);
}

void SamplingPlanner::addNode(Node* node)
{
	// add to container of all nodes
	m_nodes.push_back(node);

	// add to rtree for nearest neighbours
	point p;
	for (auto i = 0; i < node->robot_state().size(); ++i) {
		bg::set<i>(p, node->robot_state().at(i));
	}
	m_rtree.insert(std::make_pair(p, m_nodes.size()));

	// add to vertex in graph
	Vertex_t v = boost::add_vertex(m_G);
	m_G[v] = node;
}

} // namespace sampling
} // namespace clutter
