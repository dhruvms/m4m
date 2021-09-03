#include <pushplan/movable.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/constants.hpp>

#include <smpl/console/console.h>

auto std::hash<clutter::LatticeState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
	boost::hash_combine(seed, s.t);
	return seed;
}

namespace clutter
{

void Movable::SetStartState(const LatticeState& s)
{
	m_start_id = getOrCreateState(s);
	m_start = s.coord;

	m_wastar->set_start(m_start_id);
}

void Movable::SetGoalState(const Coord& p)
{
	m_goal_id = getOrCreateState(p);
	m_goal = p;
	DiscToCont(m_goal, m_goalf);

	m_wastar->set_goal(m_goal_id);
}

bool Movable::Search(int robin)
{
	m_priority = robin;

	std::vector<int> solution;
	int solcost;
	bool result = m_wastar->replan(&solution, &solcost);

	if (result)
	{
		convertPath(solution);
		m_cc->UpdateTraj(m_priority, m_solve);

		return true;
	}

	return false;
}

bool Movable::IsGoal(int state_id)
{
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	return s->t == m_t + WINDOW + 1;
}

void Movable::reset(int phase)
{
	// reset everything
	for (LatticeState* s : m_states) {
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

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
LatticeState* Movable::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int Movable::getHashEntry(
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

int Movable::reserveHashEntry()
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

int Movable::createHashEntry(
	const Coord& coord,
	const State& state,
	const int& t)
{
	int state_id = reserveHashEntry();
	LatticeState* entry = getHashEntry(state_id);

	entry->coord = coord;
	entry->state = state;
	entry->t = t;

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

int Movable::getOrCreateState(
	const Coord& coord,
	const State& state,
	const int& t)
{
	int state_id = getHashEntry(coord, t);
	if (state_id < 0) {
		state_id = createHashEntry(coord, state, t);
	}
	return state_id;
}

int Movable::getOrCreateState(const LatticeState& s)
{
	return getOrCreateState(s.coord, s.state, s.t);
}

int Movable::getOrCreateState(const Coord& p)
{
	State temp;
	return getOrCreateState(p, temp, -1);
}

} // namespace clutter
