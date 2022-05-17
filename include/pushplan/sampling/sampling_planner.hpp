#ifndef SAMPLING_PLANNER_HPP
#define SAMPLING_PLANNER_HPP

#include <pushplan/agents/robot.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <deque>
#include <random>
#include <functional>
#include <map>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace clutter {
namespace sampling {

class Node;

class SamplingPlanner
{
public:
	SamplingPlanner() :
	m_start(nullptr), m_goal(nullptr), m_rng(m_dev()),
	m_goal_nodes(0) {
		m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
	};

	virtual void SetStartState(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects);
	virtual void SetGoalState(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects);
	virtual bool Solve();
	virtual bool ExtractPath(std::vector<smpl::RobotState>& path) = 0;
	virtual bool ExtractTraj(trajectory_msgs::JointTrajectory& exec_traj) = 0;

	void SetRobot(const std::shared_ptr<Robot>& robot) {
		m_robot = robot;
	}
	void SetRobotGoalCallback(std::function<void(smpl::RobotState&)> callback) {
		m_goal_fn = callback;
	}
	auto GetStats() const -> const std::map<std::string, double>& {
		return m_stats;
	}

protected:
	typedef boost::adjacency_list<
		boost::listS, boost::vecS, boost::bidirectionalS, Node*> Graph_t;
	typedef Graph_t::vertex_descriptor Vertex_t;
	typedef Graph_t::edge_descriptor Edge_t;

	std::shared_ptr<Robot> m_robot;
	Graph_t m_G;
	std::deque<Node*> m_nodes;

	typedef bg::model::point<double, 7, bg::cs::cartesian> point;
	typedef std::pair<point, Vertex_t> value;
	bgi::rtree<value, bgi::quadratic<8> > m_rtree;

	Node *m_start, *m_goal;
	Vertex_t m_start_v;
	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	std::function<void(smpl::RobotState&)> m_goal_fn;
	int m_goal_nodes;
	std::map<std::string, double> m_stats;

	virtual bool extend(
		const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result) = 0;
	virtual bool selectVertex(const smpl::RobotState& qrand, Vertex_t& nearest) = 0;
	virtual bool steer(
		const smpl::RobotState& qrand,
		Node* xnear,
		smpl::RobotState& qnew,
		comms::ObjectsPoses& qnew_objs,
		std::uint32_t& result) = 0;

	virtual void addNode(Node* node, Vertex_t& node_v);
	virtual void addEdge(const Vertex_t& from, const Vertex_t& to);

	double configDistance(const smpl::RobotState& s1, const smpl::RobotState& s2);
};

} // namespace sampling
} // namespace clutter

#endif // SAMPLING_PLANNER_HPP
