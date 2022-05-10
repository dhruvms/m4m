#include <pushplan/sampling/rrt.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/console/console.h>

namespace clutter {
namespace sampling {

RRT::RRT()
:
SamplingPlanner(),
m_N(10000),
m_gbias(0.05), m_gthresh(DEG5/5),
m_eps(DEG5/5), m_timeout(120.0)
{}

RRT::RRT(int samples, double gbias, double gthresh, double eps, double timeout)
:
SamplingPlanner(),
m_N(samples),
m_gbias(gbias), m_gthresh(gthresh),
m_eps(eps), m_timeout(timeout)
{}

bool RRT::Solve()
{
	if (!SamplingPlanner::Solve())
	{
		SMPL_ERROR("Planner not setup!");
		return false;
	}

	addNode(m_start);
	for (int i = 0; i < m_N; ++i)
	{
		// get random sample
		smpl::RobotState qrand;
		if (m_distD(m_rng) < m_gbias) {
			m_goal_fn(qrand);
		}
		else {
			m_robot->GetRandomState(qrand);
		}

		extend(qrand);
	}
}

std::uint32_t RRT::extend(const smpl::RobotState& qrand)
{
	Node* xnear = selectVertex(qrand);
	smpl::RobotState qnew;
	if (steer(qrand, xnear, qnew))
	{

	}
}

Node* RRT::selectVertex(const smpl::RobotState& qrand)
{
	std::vector<value> nn;
	rtree.query(bgi::nearest(qrand, 1), std::back_inserter(nn));

	return m_nodes.at(nn.front().second);
}

bool RRT::steer(
	const smpl::RobotState& qrand,
	Node* xnear,
	smpl::RobotState& qnew)
{
	m_robot->SetScene(xnear->objects());
}

} // namespace sampling
} // namespace clutter
