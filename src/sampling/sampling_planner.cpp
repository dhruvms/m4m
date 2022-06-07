#include <pushplan/sampling/sampling_planner.hpp>
#include <pushplan/sampling/node.hpp>

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
	// hax?
	point p;
	auto s = node->robot_state();
	bg::set<0>(p, s.at(0));
	bg::set<1>(p, s.at(1));
	bg::set<2>(p, s.at(2));
	bg::set<3>(p, s.at(3));
	bg::set<4>(p, s.at(4));
	bg::set<5>(p, s.at(5));
	bg::set<6>(p, s.at(6));
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

bool SamplingPlanner::poseWithinTolerance(const smpl::RobotState& s, double xyz_tolerance, double rpy_tolerance)
{
	Eigen::Affine3d s_pose;
	m_robot->ComputeFK(s, s_pose);

	auto dx = std::fabs(s_pose.translation()[0] - m_goal_pose.translation()[0]);
	auto dy = std::fabs(s_pose.translation()[1] - m_goal_pose.translation()[1]);
	auto dz = std::fabs(s_pose.translation()[2] - m_goal_pose.translation()[2]);

	Eigen::Quaterniond qg(m_goal_pose.rotation());
	Eigen::Quaterniond q(s_pose.rotation());
	if (q.dot(qg) < 0.0) {
		qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
	}

	auto theta = smpl::angles::normalize_angle(2.0 * std::acos(q.dot(qg)));

	return dx <= xyz_tolerance && dy <= xyz_tolerance && dz <= xyz_tolerance && theta < rpy_tolerance;
}

double SamplingPlanner::poseGoalDistance(const smpl::RobotState& s)
{
	Eigen::Affine3d s_pose;
	m_robot->ComputeFK(s, s_pose);

	Eigen::Vector4d dist;
	dist(0) = std::fabs(s_pose.translation()[0] - m_goal_pose.translation()[0]);
	dist(1) = std::fabs(s_pose.translation()[1] - m_goal_pose.translation()[1]);
	dist(2) = std::fabs(s_pose.translation()[2] - m_goal_pose.translation()[2]);

	Eigen::Quaterniond qg(m_goal_pose.rotation());
	Eigen::Quaterniond q(s_pose.rotation());
	if (q.dot(qg) < 0.0) {
		qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
	}

	dist(3) = smpl::angles::normalize_angle(2.0 * std::acos(q.dot(qg)));

	return dist.norm();
}

} // namespace sampling
} // namespace clutter
