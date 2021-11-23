#include <pushplan/agent.hpp>
#include <pushplan/geometry.hpp>

#include <smpl/console/console.h>

#include <iostream>
#include <algorithm>

namespace clutter
{

bool Agent::Setup()
{
	m_orig_o = m_objs.back();
}

void Agent::ResetObject()
{
	m_objs.back().o_x = m_orig_o.o_x;
	m_objs.back().o_y = m_orig_o.o_y;
}

bool Agent::SetObjectPose(
	const std::vector<double>& xyz,
	const std::vector<double>& rpy)
{
	m_objs.back().o_x = xyz.at(0);
	m_objs.back().o_y = xyz.at(1);
	m_objs.back().o_z = xyz.at(2);

	m_objs.back().o_roll = rpy.at(0);
	m_objs.back().o_pitch = rpy.at(1);
	m_objs.back().o_yaw = rpy.at(2);
}

bool Agent::Init()
{
	m_solve.clear();
	m_retrieve.clear();
	m_move.clear();

	m_t = 0;

	m_objs.back().o_x = m_orig_o.o_x;
	m_objs.back().o_y = m_orig_o.o_y;

	m_init.t = m_t;
	m_init.state.clear();
	m_init.state.push_back(m_objs.back().o_x);
	m_init.state.push_back(m_objs.back().o_y);
	ContToDisc(m_init.state, m_init.coord);
	m_current = m_init;

	m_move.push_back(m_current);

	if (!m_wastar) {
		m_wastar = std::make_unique<WAStar>(this, 1.0); // make A* search object
	}

	return true;
}

bool Agent::AtGoal(const LatticeState& s, bool verbose)
{
	double dist = EuclideanDist(s.state, m_goalf);

	if (verbose) {
		SMPL_INFO("At: (%f, %f), Goal: (%f, %f), dist = %f (thresh = %f)", s.state.at(0), s.state.at(1), m_goalf.at(0), m_goalf.at(1), dist, GOAL_THRESH);
	}

	return dist < GOAL_THRESH;
}

void Agent::Step(int k)
{
	// TODO: account for k > 1
	m_t += k;
	for (const auto& s: m_solve)
	{
		if (s.t == m_t)
		{
			m_current.t = s.t;
			if (m_current.state != s.state)
			{
				if (m_t > 1 && m_priority > 1) {
					m_cc->UpdateConflicts(m_current, m_objs.back(), m_priority);
				}
				m_current = s;
				m_move.push_back(m_current);
			}
		}
	}
}

void Agent::GetSE2Push(std::vector<double>& push)
{
	push.clear();
	push.resize(3, 0.0); // (x, y, yaw)

	double move_dir = std::atan2(
					m_move.back().state.at(1) - m_move.front().state.at(1),
					m_move.back().state.at(0) - m_move.front().state.at(0));
	push.at(0) = m_orig_o.o_x + std::cos(move_dir + M_PI) * (m_objs.back().x_size + 0.05);
	push.at(1) = m_orig_o.o_y + std::sin(move_dir + M_PI) * (m_objs.back().x_size + 0.05);
	push.at(2) = move_dir;

	if (m_objs.back().shape == 0)
	{
		// get my object rectangle
		std::vector<State> rect;
		State o = {m_orig_o.o_x, m_orig_o.o_y};
		GetRectObjAtPt(o, m_objs.back(), rect);

		// find rectangle side away from push direction
		push.at(0) = m_orig_o.o_x + std::cos(move_dir + M_PI) * 0.5;
		push.at(1) = m_orig_o.o_y + std::sin(move_dir + M_PI) * 0.5;
		State p = {push.at(0), push.at(1)}, intersection;
		double op = EuclideanDist(o, p);
		int side = 0;
		for (; side <= 3; ++side)
		{
			LineLineIntersect(o, p, rect.at(side), rect.at((side + 1) % 4), intersection);
			if (PointInRectangle(intersection, rect))
			{
				if (EuclideanDist(intersection, o) + EuclideanDist(intersection, p) <= op + 1e-6) {
					break;
				}
			}
		}

		// compute push point on side
		intersection.at(0) += std::cos(move_dir + M_PI) * 0.08;
		intersection.at(1) += std::sin(move_dir + M_PI) * 0.08;

		// update push
		push.at(0) = intersection.at(0);
		push.at(1) = intersection.at(1);
	}
}

void Agent::GetSuccs(
	int state_id,
	std::vector<int>* succ_ids,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succ_ids->clear();
	costs->clear();

	LatticeState* parent = getHashEntry(state_id);
	assert(parent);
	m_closed.push_back(parent);

	if (IsGoal(state_id)) {
		SMPL_WARN("We are expanding the goal state (???)");
		return;
	}

	for (int dx = -1; dx <= 1; ++dx)
	{
		for (int dy = -1; dy <= 1; ++dy)
		{
			// ignore ordinal directions for 4-connected grid
			if (GRID == 4 && std::abs(dx * dy) == 1) {
				continue;
			}

			generateSuccessor(parent, dx, dy, succ_ids, costs);
		}
	}
}

unsigned int Agent::GetGoalHeuristic(int state_id)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	if (s->state.empty()) {
		DiscToCont(s->coord, s->state);
	}
	double dist = EuclideanDist(s->state, m_goalf);
	return (dist * COST_MULT);
}

unsigned int Agent::GetGoalHeuristic(const LatticeState& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	double dist = EuclideanDist(s.state, m_goalf);
	return (dist * COST_MULT);
}

const std::vector<Object>* Agent::GetObject(const LatticeState& s)
{
	m_objs.back().o_x = s.state.at(0);
	m_objs.back().o_y = s.state.at(1);
	return &m_objs;
}

int Agent::generateSuccessor(
	const LatticeState* parent,
	int dx, int dy,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	LatticeState child;
	child.t = parent->t + 1;
	child.coord = parent->coord;
	child.coord.at(0) += dx;
	child.coord.at(1) += dy;
	DiscToCont(child.coord, child.state);

	if (m_cc->ImmovableCollision(child, m_objs.back(), m_priority)) {
		return -1;
	}

	int succ_state_id;
	if (m_priority > 1 && !m_cc->IsStateValid(child, m_objs.back(), m_priority)) {
		return -1;
	}

	succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

unsigned int Agent::cost(
	const LatticeState* s1,
	const LatticeState* s2)
{
	if (s2->t <= m_t + WINDOW)
	{
		if (AtGoal(*s1) && AtGoal(*s2)){
			return 0;
		}

		double dist = std::max(1.0, EuclideanDist(s1->state, s2->state));
		return (dist * COST_MULT);

		// // Works okay for WINDOW = 20, but not for WINDOW <= 10
		// double bdist = m_cc->BoundaryDistance(s2f);
		// double bw = m_cc->GetBaseWidth(), bl = m_cc->GetBaseLength();
		// return ((dist + WINDOW*(1 - bdist/std::min(bw, bl))*(m_priority > 0)) * COST_MULT);
	}
	else if (s2->t == m_t + WINDOW + 1) {
		return GetGoalHeuristic(*s2);
	}
	else {
		SMPL_ERROR("Unknown edge cost condition! Return 0. (s1->t, s2->t, m_t) = (%d, %d, %d)", s1->t, s2->t, m_t);
		return 0;
	}
}

bool Agent::convertPath(
	const std::vector<int>& idpath)
{
	Trajectory opath; // vector of LatticeState

	if (idpath.empty()) {
		return true;
	}

	LatticeState state;

	// attempt to handle paths of length 1...do any of the sbpl planners still
	// return a single-point path in some cases?
	if (idpath.size() == 1)
	{
		auto state_id = idpath[0];

		if (state_id == m_goal_id)
		{
			auto* entry = getHashEntry(m_start_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", m_start_id);
				return false;
			}
			state = *entry;
			opath.push_back(state);
		}
		else
		{
			auto* entry = getHashEntry(state_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", state_id);
				return false;
			}
			state = *entry;
			opath.push_back(state);
		}
	}

	if (idpath[0] == m_goal_id)
	{
		SMPL_ERROR("Cannot extract a non-trivial path starting from the goal state");
		return false;
	}

	// grab the first point
	{
		auto* entry = getHashEntry(idpath[0]);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry for state %d", idpath[0]);
			return false;
		}
		state = *entry;
		opath.push_back(state);
	}

	// grab the rest of the points
	for (size_t i = 1; i < idpath.size(); ++i)
	{
		auto prev_id = idpath[i - 1];
		auto curr_id = idpath[i];

		if (prev_id == m_goal_id)
		{
			SMPL_ERROR("Cannot determine goal state predecessor state during path extraction");
			return false;
		}

		auto* entry = getHashEntry(curr_id);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry state %d", curr_id);
			return false;
		}
		state = *entry;
		opath.push_back(state);
	}
	m_solve = std::move(opath);
	return true;
}

} // namespace clutter
