#include <pushplan/agents/agent_lattice.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/utils/geometry.hpp>

auto std::hash<clutter::LatticeState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.begin() + 2));
	boost::hash_combine(seed, s.t);
	return seed;
}

namespace clutter
{

void AgentLattice::init(Agent* agent, bool backwards)
{
	m_agent = agent;
	m_backwards = backwards;
}

void AgentLattice::reset()
{
	// reset everything
	for (LatticeState* s : m_states) {
		if (s != nullptr) {
			delete s;
			s = nullptr;
		}
	}

	m_start_ids.clear();
	m_goal_ids.clear();
	m_state_to_id.clear();
	m_states.clear();
}

int AgentLattice::PushStart(const LatticeState& s)
{
	int start_id = getOrCreateState(s);
	m_start_ids.push_back(start_id);
	return start_id;
}

int AgentLattice::PushGoal(const Coord& p)
{
	int goal_id = getOrCreateState(p);
	m_goal_ids.push_back(goal_id);
	return goal_id;
}

void AgentLattice::SetCTNode(HighLevelNode* ct_node)
{
	m_constraints.clear();
	for (auto& constraint : ct_node->m_constraints)
	{
		if (constraint->m_me == ct_node->m_replanned) {
			m_constraints.push_back(constraint);
		}
	}

	m_cbs_solution = &(ct_node->m_solution);
	m_cbs_id = ct_node->m_replanned;
	m_max_time = ct_node->m_makespan;
}

void AgentLattice::AvoidAgents(const std::unordered_set<int>& to_avoid)
{
	m_to_avoid = to_avoid;
}

// As long as I am not allowed to be in this location at some later time,
// I have not reached a valid goal state
// Conversely, if I can remain in this location (per existing constraints),
// I am at a valid goal state (since states in collision with immovable obstacles
// or out of bounds will never enter OPEN)
bool AgentLattice::IsGoal(int state_id)
{
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	if (m_backwards) {
		return s->coord == m_agent->Goal();
	}

	bool constrained = false, conflict = false, ngr = false;

	ngr = m_agent->OutsideNGR(*s);
	if (ngr)
	{
		for (const auto& constraint : m_constraints)
		{
			if (constraint->m_q.coord == s->coord) {
				if (constraint->m_time >= s->t) {
					constrained = true;
					break;
				}
			}
		}

		if (!constrained)
		{
			bool conflict = goalConflict(*s);

			// for forward search goal must valid for all future time
			return !conflict;
		}

		// for forward search goal must not be constrained
		return false;
	}

	// for forward search goal must be outside NGR
	return false;
}

void AgentLattice::GetSuccs(
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

unsigned int AgentLattice::GetGoalHeuristic(int state_id)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	double dist = EuclideanDist(s->coord, m_agent->Goal());
	return (dist * COST_MULT);
}

unsigned int AgentLattice::GetConflictHeuristic(int state_id)
{
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	return (s->hc * COST_MULT);
}

unsigned int AgentLattice::GetGoalHeuristic(const LatticeState& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	double dist = EuclideanDist(s.coord, m_agent->Goal());
	return (dist * COST_MULT);
}

int AgentLattice::generateSuccessor(
	const LatticeState* parent,
	int dx, int dy,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	LatticeState child;
	child.t = parent->t + (m_backwards ? 0 : 1);
	child.hc = parent->hc;
	child.coord = parent->coord;
	child.coord.at(0) += dx;
	child.coord.at(1) += dy;
	DiscToCont(child.coord, child.state);
	child.state.insert(child.state.end(), parent->state.begin() + 2, parent->state.end());

	m_agent->UpdatePose(child);
	if (m_agent->OutOfBounds(child) || m_agent->ImmovableCollision()) {
		return -1;
	}

	for (const auto& constraint : m_constraints)
	{
		if (child.t == constraint->m_time)
		{
			// Conflict type 1: robot-object conflict
			if (constraint->m_me == constraint->m_other)
			{
	 		// 	if (m_cbs_solution->at(0).second.size() <= constraint->m_time)
				// {
				// 	// This should never happen - the constraint would only have existed
				// 	// if this object and the robot had a conflict at that time
				// 	SMPL_WARN("How did this robot-object conflict happen with a small robot traj?");
				// 	continue;
				// }

				// // successor is invalid if I collide in state 'child'
				// // with the robot configuration at the same time
				// if (m_cc->RobotObjectCollision(
				// 			this, child,
				// 			m_cbs_solution->at(0).second.at(constraint->m_time), constraint->m_time))
				// {
				// 	return -1;
				// }
			}
			// Conflict type 2: object-object conflict
			else
			{
				// successor is invalid if I collide in state 'child'
				// with the constraint->m_other object in state constraint->m_q
				if (m_agent->ObjectObjectCollision(constraint->m_other, constraint->m_q)) {
					return -1;
				}
			}
		}
	}

	if (!m_to_avoid.empty())
	{
		std::vector<LatticeState> other_poses;
		std::vector<int> other_ids;

		for (const auto& agent_traj: *m_cbs_solution)
		{
			if (agent_traj.first == m_cbs_id || agent_traj.first == 0) {
				continue;
			}
			if (m_to_avoid.find(agent_traj.first) == m_to_avoid.end()) {
				continue;
			}

			other_ids.push_back(agent_traj.first);
			if (agent_traj.second.size() <= child.t) {
				other_poses.push_back(agent_traj.second.back());
			}
			else {
				other_poses.push_back(agent_traj.second.at(child.t));
			}
		}
		if (m_agent->ObjectObjectsCollision(other_ids, other_poses)) {
			return -1;
		}
	}

	child.hc += conflictHeuristic(child);

	int succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	// For ECBS (the state of the object is collision checked with the state of all agents
	// and the resulting number of collisions is used as part of the cost function
	// therefore not a hard constraint (like constraints).)

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

unsigned int AgentLattice::cost(
	const LatticeState* s1,
	const LatticeState* s2)
{
	double dist = EuclideanDist(s1->coord, s2->coord);
	dist = dist == 0.0 ? 1.0 : dist;
	return (dist * COST_MULT);
}

int AgentLattice::conflictHeuristic(const LatticeState& state)
{
	int hc = 0;
	switch (LLHC)
	{
		case LowLevelConflictHeuristic::BINARY:
		{
			std::vector<LatticeState> other_poses;
			std::vector<int> other_ids;

			for (const auto& agent_traj: *m_cbs_solution)
			{
				if (agent_traj.first == m_cbs_id || agent_traj.first == 0) {
					continue;
				}

				other_ids.push_back(agent_traj.first);
				if (agent_traj.second.size() <= state.t) {
					other_poses.push_back(agent_traj.second.back());
				}
				else {
					other_poses.push_back(agent_traj.second.at(state.t));
				}
			}
			bool conflict = m_agent->ObjectObjectsCollision(other_ids, other_poses);

			// if (!conflict)
			// {
			// 	if (m_cbs_solution->at(0).second.size() <= state.t) {
			// 		conflict = conflict ||
			// 			m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.back(), state.t);
			// 	}
			// 	else {
			// 		conflict = conflict ||
			// 			m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.at(state.t), state.t);
			// 	}
			// }

			hc = (int)conflict;
			break;
		}
		case LowLevelConflictHeuristic::COUNT:
		{
			LatticeState other_pose;
			for (const auto& other_agent: *m_cbs_solution)
			{
				if (other_agent.first == m_cbs_id || other_agent.first == 0) {
					continue;
				}

				// other agent trajectory is shorter than current state's time
				// so we only collision check against the last state along the
				// other agent trajectory
				if (other_agent.second.size() <= state.t) {
					other_pose = other_agent.second.back();
				}
				else {
					// if the other agent has a trajectory longer than current state's time
					// we collision check against the current state's time
					other_pose = other_agent.second.at(state.t);
				}
				if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
					++hc;
				}
			}

			// // same logic for robot
			// if (m_cbs_solution->at(0).second.size() <= state.t) {
			// 	other_pose = m_cbs_solution->at(0).second.back();
			// }
			// else {
			// 	other_pose = m_cbs_solution->at(0).second.at(state.t);
			// }
			// if (m_cc->RobotObjectCollision(this, state, other_pose, state.t)) {
			// 	++hc;
			// }
			break;
		}
		default:
		{
			SMPL_ERROR("Unknown conflict heuristic type!");
		}
	}

	return hc;
}

bool AgentLattice::goalConflict(const LatticeState& state)
{
	m_agent->UpdatePose(state);

	LatticeState other_pose;
	for (const auto& other_agent: *m_cbs_solution)
	{
		if (other_agent.first == m_cbs_id || other_agent.first == 0) {
			continue;
		}

		// other agent trajectory is shorter than current state's time
		// so we only collision check against the last state along the
		// other agent trajectory
		if (other_agent.second.size() <= state.t)
		{
			other_pose = other_agent.second.back();
			if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
				return true;
			}
		}
		else
		{
			// if the other agent has a trajectory longer than current state's time
			// we collision check against all states in that trajectory beyond
			// the current state's time
			for (int t = state.t; t < (int)other_agent.second.size(); ++t)
			{
				other_pose = other_agent.second.at(t);
				if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
					return true;
				}
			}
		}
	}

	// // same logic for robot
	// if (m_cbs_solution->at(0).second.size() <= state.t)
	// {
	// 	if (m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.back(), state.t)) {
	// 		return true;
	// 	}
	// }
	// else
	// {
	// 	for (int t = state.t; t < (int)m_cbs_solution->at(0).second.size(); ++t)
	// 	{
	// 		other_pose = m_cbs_solution->at(0).second.at(t);
	// 		if (m_cc->RobotObjectCollision(this, state, other_pose, state.t)) {
	// 			return true;
	// 		}
	// 	}
	// }

	return false;
}

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
LatticeState* AgentLattice::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int AgentLattice::getHashEntry(
	const Coord& coord,
	const int& t)
{
	LatticeState s;
	s.coord = coord;
	s.t = t;

	auto sit = m_state_to_id.find(&s);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int AgentLattice::reserveHashEntry()
{
	LatticeState* entry = new LatticeState;
	int state_id = (int)m_states.size();

	// map state id -> state
	m_states.push_back(entry);

	// // map planner state -> graph state
	// int* pinds = new int[NUMOFINDICES_STATEID2IND];
	// std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
	// StateID2IndexMapping.push_back(pinds);

	return state_id;
}

int AgentLattice::createHashEntry(
	const Coord& coord,
	const State& state,
	const int& t,
	const int& hc)
{
	int state_id = reserveHashEntry();
	LatticeState* entry = getHashEntry(state_id);

	entry->coord = coord;
	entry->state = state;
	entry->t = t;
	entry->hc = hc;

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

int AgentLattice::getOrCreateState(
	const Coord& coord,
	const State& state,
	const int& t,
	const int& hc)
{
	int state_id = getHashEntry(coord, t);
	if (state_id < 0) {
		state_id = createHashEntry(coord, state, t, hc);
	}
	return state_id;
}

int AgentLattice::getOrCreateState(const LatticeState& s)
{
	return getOrCreateState(s.coord, s.state, s.t, s.hc);
}

int AgentLattice::getOrCreateState(const Coord& p)
{
	State temp;
	return getOrCreateState(p, temp, -1, -1);
}

bool AgentLattice::ConvertPath(
	const std::vector<int>& idpath)
{
	Trajectory* opath = m_agent->SolveTraj(); // vector of LatticeState
	opath->clear();

	if (idpath.empty()) {
		return true;
	}

	if (idpath[0] == m_goal_ids.back())
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

		if (state_id == m_goal_ids.back())
		{
			auto* entry = getHashEntry(state_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", state_id);
				return false;
			}
			state = *entry;
			opath->push_back(state);
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
			opath->push_back(state);
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
		opath->push_back(state);
	}

	// grab the rest of the points
	for (size_t i = 1; i < idpath.size(); ++i)
	{
		auto prev_id = idpath[i - 1];
		auto curr_id = idpath[i];

		if (prev_id == m_goal_ids.back())
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
		opath->push_back(state);
	}
	return true;
}

} // namespace clutter
