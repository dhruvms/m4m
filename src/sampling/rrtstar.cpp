#include <pushplan/sampling/rrtstar.hpp>
#include <pushplan/sampling/node.hpp>
#include <pushplan/utils/constants.hpp>

#include <comms/ObjectsPoses.h>
#include <smpl/console/console.h>

#include <stdexcept>
#include <iostream>

namespace clutter {
namespace sampling {

RRTStar::RRTStar()
:
RRT()
{}

RRTStar::RRTStar(int samples, int steps, double gbias, double gthresh, double timeout)
:
RRT(samples, steps, gbias, gthresh, timeout)
{}

bool RRTStar::Solve()
{
	m_rewire_factor = 0.1;
	m_dofs = (double)m_robot->RobotModel()->jointVariableCount();
	m_radius = m_rewire_factor * (std::pow(2, m_dofs + 1) * M_E * (1.0 + 1.0 / m_dofs));

	m_start->set_cost(0.0);

	return RRT::Solve();
}

bool RRTStar::extend(const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result)
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
		Node* xnew = new Node(qnew, qnew_objs);
		addNode(xnew, new_v);
		xnew->set_cost(xnear->cost() + configDistance(xnear->robot_state(), qnew));

		Node* xmin = xnear;
		Vertex_t xmin_v = near_v;

		std::vector<Vertex_t> nns;
		nearestNeighbours(xnew->robot_state(), nns);
		if (!nns.empty())
		{
			for (const auto& v : nns)
			{
				xnear = m_G[v];
				std::uint32_t rewire_result;
				if (steer(xnew->robot_state(), xnear, qnew, qnew_objs, rewire_result))
				{
					if (rewire_result == 5 || rewire_result == 9)
					{
						// we actually reached xnew
						double new_c = xnear->cost() + configDistance(xnear->robot_state(), xnew->robot_state());
						if (new_c < xnew->cost())
						{
							xnew->set_cost(new_c);
							xnew->set_objects(qnew_objs);

							xmin = xnear;
							xmin_v = v;
						}
					}
				}
			}
		}

		xnew->set_parent(xmin);
		addEdge(xmin_v, new_v);

		if (!nns.empty())
		{
			for (const auto& v : nns)
			{
				if (v == xmin_v) {
					continue;
				}

				xnear = m_G[v];
				std::uint32_t rewire_result;
				if (steer(xnear->robot_state(), xnew, qnew, qnew_objs, rewire_result))
				{
					if (rewire_result == 5 || rewire_result == 9)
					{
						// we actually reached xnear
						double new_c = xnew->cost() + configDistance(xnew->robot_state(), xnear->robot_state());
						if (new_c < xnear->cost())
						{
							xnear->set_cost(new_c);
							xnear->set_objects(qnew_objs);

							if (boost::in_degree(v, m_G) != 1) {
								throw std::runtime_error("Found an RRT* node with more than one parent!");
							}

							Graph_t::in_edge_iterator eit_begin, eit_end;
							std::tie(eit_begin, eit_end) = boost::in_edges(v, m_G);
							Vertex_t parent_v = boost::source(*eit_begin, m_G);

							removeEdge(parent_v, v);
							addEdge(new_v, v);

							xnear->parent()->remove_child(xnear);
							xnear->set_parent(xnew);
						}
					}
				}
			}
		}

		return true;
	}

	return false;
}

bool RRTStar::nearestNeighbours(const smpl::RobotState& state, std::vector<Vertex_t>& nns)
{
	if (m_rtree.empty()) {
		return false;
	}

	std::vector<value> results;
	// hax?
	point query_p;
	bg::set<0>(query_p, state.at(0));
	bg::set<1>(query_p, state.at(1));
	bg::set<2>(query_p, state.at(2));
	bg::set<3>(query_p, state.at(3));
	bg::set<4>(query_p, state.at(4));
	bg::set<5>(query_p, state.at(5));
	bg::set<6>(query_p, state.at(6));

	double V = double(boost::num_vertices(m_G)) + 1.0;
	int k = std::ceil(m_radius * std::log(V));

	m_rtree.query(
		bgi::nearest(query_p, 10),
		std::back_inserter(results));

	nns.clear();
	for (const auto& nn : results) {
		nns.push_back(nn.second);
	}
	return true;
}

void RRTStar::removeEdge(const Vertex_t& source, const Vertex_t& dest)
{
	if (boost::num_vertices(m_G) == 0) {
		throw std::runtime_error("Remove edge from empty graph?!");
	}

	boost::remove_edge(source, dest, m_G);
}

} // namespace sampling
} // namespace clutter
