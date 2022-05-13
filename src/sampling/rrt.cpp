#include <pushplan/sampling/rrt.hpp>
#include <pushplan/sampling/node.hpp>
#include <pushplan/utils/constants.hpp>

#include <comms/ObjectsPoses.h>
#include <smpl/console/console.h>

namespace clutter {
namespace sampling {

RRT::RRT()
:
SamplingPlanner(),
m_N(10000), m_steps(10),
m_gbias(0.05), m_gthresh(DEG5/5),
m_timeout(120.0)
{}

RRT::RRT(int samples, int steps, double gbias, double gthresh, double timeout)
:
SamplingPlanner(),
m_N(samples), m_steps(steps),
m_gbias(gbias), m_gthresh(gthresh),
m_timeout(timeout)
{}

bool RRT::Solve()
{
	if (!SamplingPlanner::Solve())
	{
		SMPL_ERROR("Planner not setup!");
		return false;
	}

	Vertex_t tree_v;
	addNode(m_start, tree_v);

	for (int i = 0; i < m_N; ++i)
	{
		// get random sample
		smpl::RobotState qrand;
		bool try_goal = false;
		if (m_distD(m_rng) < m_gbias)
		{
			try_goal = true;
			m_goal_fn(qrand);
		}
		else {
			m_robot->GetRandomState(qrand);
		}

		std::uint32_t result;
		if (!extend(qrand, tree_v, result))
		{
			if (result == 10000) {
				SMPL_DEBUG("Failed to selectVertex from tree!");
			}
			else
			{
				if (result == 10001) {
					SMPL_DEBUG("Failed to SetScene for xnear!");
				}
				else if (result == 0) {
					SMPL_ERROR("Error in SteerAction computation!");
				}
				else if (result == 100 || result == 101) {
					SMPL_DEBUG("SteerAction failed in simulation!");
				}
			}
		}
		else
		{
			if (try_goal)
			{
				Node* xnew = m_G[tree_v];
				if (result == 1000 || configDistance(qrand, xnew->robot_state()) < m_gthresh)
				{
					++m_goal_nodes;
					SMPL_INFO("Added a node close to the goal to the tree!");
				}
			}

			if (result == 5 || result == 6) {
				SMPL_DEBUG("SteerAction added to tree without simulation!");
			}
			else if (result == 9 || result == 10) {
				SMPL_DEBUG("SteerAction added to tree after simulation!");
			}
		}
	}
}

bool RRT::extend(const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result)
{
	Node* xnear = nullptr;
	Vertex_t near_v;
	if (selectVertex(sample, near_v)) {
		xnear = m_G[near_v];
	}
	else
	{
		result = 0x0002710; // 10000
		return false;
	}

	if (configDistance(sample, xnear->robot_state()) < m_gthresh) {
		result = 0x000003E8; // 1000 if sampled config too near to tree node
		return true;
	}

	smpl::RobotState qnew;
	comms::ObjectsPoses qnew_objs;
	if (steer(sample, xnear, qnew, qnew_objs, result))
	{
		// successful steer
		Node* xnew = new Node(qnew, qnew_objs, xnear);
		addNode(xnew, new_v);
		addEdge(near_v, new_v);

		return true;
	}

	return false;
}

bool RRT::selectVertex(const smpl::RobotState& qrand, Vertex_t& nearest)
{
	if (m_rtree.empty()) {
		return false;
	}

	std::vector<value> nn;
	// hax?
	point query_p;
	bg::set<0>(query_p, qrand.at(0));
	bg::set<1>(query_p, qrand.at(1));
	bg::set<2>(query_p, qrand.at(2));
	bg::set<3>(query_p, qrand.at(3));
	bg::set<4>(query_p, qrand.at(4));
	bg::set<5>(query_p, qrand.at(5));
	bg::set<6>(query_p, qrand.at(6));

	m_rtree.query(bgi::nearest(query_p, 1), std::back_inserter(nn));

	nearest = nn.front().second;
	return true;
}

bool RRT::steer(
	const smpl::RobotState& qrand,
	Node* xnear,
	smpl::RobotState& qnew,
	comms::ObjectsPoses& qnew_objs,
	std::uint32_t& result)
{
	if (!m_robot->SetScene(xnear->objects()))
	{
		result = 0x0002711;
		return false;
	}

	return m_robot->SteerAction(
			qrand, m_steps,
			xnear->robot_state(), xnear->objects(),
			qnew, qnew_objs, result);
}

} // namespace sampling
} // namespace clutter
