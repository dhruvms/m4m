#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/geometry.hpp>

#include <smpl/console/console.h>

#include <iostream>

auto std::hash<clutter::State>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, s.p.x);
	boost::hash_combine(seed, s.p.y);
	boost::hash_combine(seed, s.t);
	return seed;
}

namespace clutter
{

void Agent::Init()
{
	m_t = 0;

	m_current.t = m_t;
	Pointf obj(m_obj.o_x, m_obj.o_y);
	ContToDisc(obj, m_current.p);
	m_init = m_current;

	m_move.emplace_back(obj, m_t);

	m_wastar = std::make_unique<WAStar>(this, 1.0); // make A* search object
}

void Agent::SetStartState(const State& s)
{
	m_start = s;
	m_start_id = getOrCreateState(m_start);

	m_wastar->set_start(m_start_id);
}

void Agent::SetGoalState(const Point& p)
{
	m_goal = p;
	DiscToCont(m_goal, m_goalf);
	m_goal_id = getOrCreateState(m_goal);

	m_wastar->set_goal(m_goal_id);
}

void Agent::Search(int robin)
{
	m_priority = robin;

	std::vector<int> solution;
	int solcost;
	bool result = m_wastar->replan(&solution, &solcost);

	if (result)
	{
		convertPath(solution);
		m_cc->UpdateTraj(m_priority, m_solve);
	}
}

bool Agent::AtGoal(const State& s, bool verbose)
{
	if (m_phase == 0 && m_priority == 1)
	{
		return m_cc->OOICollision(s, m_obj);
	}

	Pointf sf;
	DiscToCont(s.p, sf);
	float dist = EuclideanDist(sf, m_goalf);

	if (verbose) {
		SMPL_INFO("At: (%f, %f), Goal: (%f, %f), dist = %f (thresh = %f)", sf.x, sf.y, m_goalf.x, m_goalf.y, dist, GOAL_THRESH);
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
			ContToDisc(s.p, m_current.p);
			m_move.emplace_back(s.p, m_t);
		}
	}
}

void Agent::reset(int phase)
{
	// reset everything
	for (State* s : m_states) {
		if (s != nullptr) {
			delete s;
			s = nullptr;
		}
	}
	m_state_to_id.clear();
	m_states.clear();

	m_wastar->reset();

	m_phase = phase;
}

void Agent::GetSuccs(
	int state_id,
	std::vector<int>* succ_ids,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succ_ids->clear();
	costs->clear();

	State* parent = getHashEntry(state_id);
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

bool Agent::IsGoal(int state_id)
{
	assert(state_id >= 0);
	State* s = getHashEntry(state_id);
	assert(s);

	return s->t == m_t + WINDOW + 1;
}

unsigned int Agent::GetGoalHeuristic(int state_id)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	State* s = getHashEntry(state_id);
	assert(s);

	Pointf sf;
	DiscToCont(s->p, sf);
	float dist = EuclideanDist(sf, m_goalf);
	return (dist * COST_MULT);
}

unsigned int Agent::GetGoalHeuristic(const State& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	Pointf sf;
	DiscToCont(s.p, sf);
	float dist = EuclideanDist(sf, m_goalf);
	return (dist * COST_MULT);
}

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
State* Agent::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int Agent::getHashEntry(
	const int& x,
	const int& y,
	const int& t)
{
	State s;
	s.p.x = x;
	s.p.y = y;
	s.t = t;

	auto sit = m_state_to_id.find(&s);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int Agent::reserveHashEntry()
{
	State* entry = new State;
	int state_id = (int)m_states.size();

	// map state id -> state
	m_states.push_back(entry);

	// // map planner state -> graph state
	// int* pinds = new int[NUMOFINDICES_STATEID2IND];
	// std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
	// StateID2IndexMapping.push_back(pinds);

	return state_id;
}

int Agent::createHashEntry(
	const int& x,
	const int& y,
	const int& t)
{
	int state_id = reserveHashEntry();
	State* entry = getHashEntry(state_id);

	entry->p.x = x;
	entry->p.y = y;
	entry->t = t;

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

int Agent::getOrCreateState(
	const int& x,
	const int& y,
	const int& t)
{
	int state_id = getHashEntry(x, y, t);
	if (state_id < 0) {
		state_id = createHashEntry(x, y, t);
	}
	return state_id;
}

int Agent::getOrCreateState(const State& s)
{
	return getOrCreateState(s.p.x, s.p.y, s.t);
}

int Agent::getOrCreateState(const Point& p)
{
	return getOrCreateState(p.x, p.y, -1);
}

int Agent::generateSuccessor(
	const State* parent,
	int dx, int dy,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	State child;
	child.t = parent->t + 1;
	child.p.x = parent->p.x + dx;
	child.p.y = parent->p.y + dy;

	if (m_cc->ImmovableCollision(child, m_obj, m_priority)) {
		return -1;
	}

	int succ_state_id;
	if (m_priority > 0 && !m_cc->IsStateValid(child, m_obj, m_priority)) {
		return -1;
	}

	succ_state_id = getOrCreateState(child);
	State* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

unsigned int Agent::cost(
	const State* s1,
	const State* s2)
{
	if (s2->t <= m_t + WINDOW)
	{
		if (AtGoal(*s1) && AtGoal(*s2)){
			return 0;
		}

		Pointf s1f, s2f;
		DiscToCont(s1->p, s1f);
		DiscToCont(s2->p, s2f);
		float dist = 1.0f + EuclideanDist(s1f, s2f);
		return (dist * COST_MULT);

		// // Works okay for WINDOW = 20, but not for WINDOW <= 10
		// float bdist = m_cc->BoundaryDistance(s2f);
		// float bw = m_cc->GetBaseWidth(), bl = m_cc->GetBaseLength();
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
	Trajectory opath;

	if (idpath.empty()) {
		return true;
	}

	Statef state;

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
			DiscToCont(entry->p, state.p);
			state.t = entry->t;
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
			DiscToCont(entry->p, state.p);
			state.t = entry->t;
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
		DiscToCont(entry->p, state.p);
		state.t = entry->t;
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
		DiscToCont(entry->p, state.p);
		state.t = entry->t;
		opath.push_back(state);
	}
	m_solve = std::move(opath);
	return true;
}

} // namespace clutter
