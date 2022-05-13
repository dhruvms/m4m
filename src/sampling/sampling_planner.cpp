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

void SamplingPlanner::addNode(Node* node, Vertex_t& node_v)
{
	// add to container of all nodes
	m_nodes.push_back(node);

	// add to vertex in graph
	node_v = boost::add_vertex(m_G);
	m_G[node_v] = node;

	// add to rtree for nearest neighbours
	point p;
	for (auto i = 0; i < node->robot_state().size(); ++i) {
		bg::set<i>(p, node->robot_state().at(i));
	}
	m_rtree.insert(std::make_pair(p, node_v));
}

void SamplingPlanner::addEdge(const Vertex_t& from, const Vertex_t& to)
{
	boost::add_edge(from, to, m_G);
}

double SamplingPlanner::configDistance(const smpl::RobotState& s1, const smpl::RobotState& s2)
{
	double dist = 0.0;
	auto rm = m_robot->RobotModel();
	for (int jidx = 0; jidx < rm->jointVariableCount(); jidx++) {
		if (rm->isContinuous(jidx)) {
			dist += smpl::angles::shortest_angle_dist(s1[jidx], s2[jidx]);
		}
		else {
			dist += std::fabs(s1[jidx] - s2[jidx]);
		}
	}

	return dist;
}

} // namespace sampling
} // namespace clutter
