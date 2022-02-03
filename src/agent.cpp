#include <pushplan/agent.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/cbs_h_node.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/conflicts.hpp>

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
	m_objs.back().o_x = m_orig_o.o_x;
	m_objs.back().o_y = m_orig_o.o_y;

	m_init.t = 0;
	m_init.state.clear();
	m_init.state.push_back(m_objs.back().o_x);
	m_init.state.push_back(m_objs.back().o_y);
	m_init.state.push_back(m_objs.back().o_z);
	m_init.state.push_back(m_objs.back().o_roll);
	m_init.state.push_back(m_objs.back().o_pitch);
	m_init.state.push_back(m_objs.back().o_yaw);
	ContToDisc(m_init.state, m_init.coord);

	if (!m_wastar) {
		m_wastar = std::make_unique<WAStar>(this, 1.0); // make A* search object
	}
	this->reset();
	this->SetStartState(m_init);
	this->SetGoalState(m_init.coord);

	return true;
}

bool Agent::SatisfyPath(HighLevelNode* ct_node, Robot* robot, Trajectory** sol_path)
{
	m_solve.clear();
	LatticeState s = m_init;
	// OOI stays in place during approach
	for (int i = 0; i < robot->GraspAt() + 2; ++i)
	{
		s.t = i;
		m_solve.push_back(s);
	}

	// OOI tracks robot EE during extraction
	auto* r_traj = robot->GetLastTraj();
	for (int i = robot->GraspAt() + 2; i < r_traj->size(); ++i)
	{
		++s.t;
		s.state = robot->GetEEState(r_traj->at(i).state);
		ContToDisc(s.state, s.coord);
		m_solve.push_back(s);
	}
	*sol_path = &(this->m_solve);

	return true;
}

bool Agent::SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path)
{
	m_solve.clear();
	// collect agent constraints
	m_constraints.clear();
	for (auto& constraint : ct_node->m_constraints)
	{
		if (constraint->m_me == ct_node->m_replanned) {
			m_constraints.push_back(constraint);
		}
	}
	m_cbs_solution = &(ct_node->m_solution);

	std::vector<int> solution;
	int solcost;
	bool result = m_wastar->replan(&solution, &solcost);

	if (result)
	{
		convertPath(solution);
		*sol_path = &(this->m_solve);
	}

	return result;
}

// As long as I am not allowed to be in this location at some later time,
// I have not reached a valid goal state
// Conversely, if I can remain in this location (per existing constraints),
// I am at a valid goal state (since states in collision with immovable obstacles
// or out of bounds will never enter OPEN)
bool Agent::IsGoal(int state_id)
{
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	bool constrained = false;
	for (const auto& constraint : m_constraints)
	{
		if (constraint->m_q.coord == s->coord) {
			if (constraint->m_time >= s->t) {
				constrained = true;
				break;
			}
		}
	}

	return !constrained;
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
	State sxy = {s->state.at(0), s->state.at(1)};
	double dist = EuclideanDist(sxy, m_goalf);
	return (dist * COST_MULT);
}

unsigned int Agent::GetGoalHeuristic(const LatticeState& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	State sxy = {s.state.at(0), s.state.at(1)};
	double dist = EuclideanDist(sxy, m_goalf);
	return (dist * COST_MULT);
}

const std::vector<Object>* Agent::GetObject(const LatticeState& s)
{
	m_objs.back().o_x = s.state.at(0);
	m_objs.back().o_y = s.state.at(1);
	return &m_objs;
}

void Agent::GetSE2Push(std::vector<double>& push)
{
	push.clear();
	push.resize(3, 0.0); // (x, y, yaw)

	double move_dir = std::atan2(
					m_solve.back().state.at(1) - m_solve.front().state.at(1),
					m_solve.back().state.at(0) - m_solve.front().state.at(0));
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
	child.state.insert(child.state.end(), parent->state.begin() + 2, parent->state.end());

	if (m_cc->OutOfBounds(child) || m_cc->ImmovableCollision(child, this->GetFCLObject())) {
		return -1;
	}

	UpdatePose(child);
	for (const auto& constraint : m_constraints)
	{
		if (constraint->m_me == constraint->m_other)
		{
 			if (m_cbs_solution->at(0).second.size() <= constraint->m_time)
			{
				// This should never happen - the constraint would only have existed
				// if this object and the robot had a conflict at that time
				SMPL_WARN("How did this robot-object conflict happen with a small robot traj?");
				continue;
			}

			// successor is invalid if that (position, time) configuration
			// is constrained
			if (m_cc->RobotObjectCollision(
						this, child,
						m_cbs_solution->at(0).second.at(constraint->m_time), constraint->m_time))
			{
				return -1;
			}
		}
		else if (child.t == constraint->m_time)
		{
			// CBS TODO: check FCL collision between constraint->m_me object
			// at location child.coord and constraint->m_other object
			// at location constraint->m_q at time constraint->m_time
			if (m_cc->ObjectObjectCollision(this, constraint->m_other, constraint->m_q)) {
				return -1;
			}
		}
	}

	int succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

unsigned int Agent::cost(
	const LatticeState* s1,
	const LatticeState* s2)
{
	double dist = EuclideanDist(s1->coord, s2->coord);
	dist = dist == 0.0 ? 1.0 : dist;
	return (dist * COST_MULT);
}

bool Agent::convertPath(
	const std::vector<int>& idpath)
{
	Trajectory opath; // vector of LatticeState

	if (idpath.empty()) {
		return true;
	}

	if (idpath[0] == m_goal_id)
	{
		SMPL_ERROR("Cannot extract a non-trivial path starting from the goal state");
		return false;
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
	else
	{
		// grab the first point
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
