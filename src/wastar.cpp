#include <pushplan/wastar.hpp>
#include <pushplan/constants.hpp>
#include <pushplan/types.hpp>
#include <pushplan/movable.hpp>
#include <pushplan/helpers.hpp>

#include <smpl/console/console.h>

#include <iostream>
#include <algorithm>

namespace clutter
{

WAStar::WAStar(
	Movable* movable,
	double w)
:
m_movable(movable),
m_call_number(0),
m_w(w),
m_start_id(-1),
m_goal_id(-1)
{
	// Set default max planing time
	m_time_limit = 2.0; // seconds

	m_open = new OPEN[1];
	m_expands = new int[1];
}

WAStar::~WAStar()
{
	reset();
}

int WAStar::set_start(int start_id)
{
	m_start_id = start_id;
	m_start = get_state(m_start_id);
	return m_start_id;
}

int WAStar::set_goal(int goal_id)
{
	m_goal_id = goal_id;
	m_goal = get_state(m_goal_id);
	return m_goal_id;
}

int WAStar::get_n_expands() const
{
	return m_expands[0];
}

void WAStar::reset()
{
	// Clear OPEN list
	for (int i = 0; i < num_heuristics(); ++i) {
		if (!m_open[i].empty()) {
			m_open[i].clear();
		}
	}

	// free states
	for (size_t i = 0; i < m_states.size(); ++i)
	{
		if (m_states[i] != nullptr)
		{
			free(m_states[i]);
			// m_states[i] = nullptr;
		}
	}

	// Clear state table
	m_states.clear();
	// m_states.shrink_to_fit();

	m_start_id = -1;
	m_goal_id = -1;
	m_start = nullptr;
	m_goal = nullptr;
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
SearchState* WAStar::get_state(int state_id)
{
	assert(state_id >= 0);

	if (m_states.size() <= state_id)
	{
		size_t state_size = sizeof(SearchState);
		SearchState* s = (SearchState*)malloc(state_size);

		// Use placement new(s) to construct allocated memory
		new (s) SearchState;
		for (int i = 0; i < num_heuristics() - 1; ++i) { // loop not entered
			new (&s->od[1 + i]) SearchState::HeapData;
		}

		// assert(state_id == m_states.size());

		init_state(s, state_id);
		m_states.push_back(s);

		return s;
	}

	return m_states[state_id];
}

void WAStar::init_state(SearchState *state, int state_id)
{
	state->call_number = 0; // not initialized for any iteration
	state->state_id = state_id;
	for (int i = 0; i < num_heuristics(); ++i) {
		state->od[i].me = state;
	}
}

// Lazily (re)initialize a search state.
void WAStar::reinit_state(SearchState *state)
{
	if (state->call_number != m_call_number) {
		state->call_number = m_call_number;
		state->g = std::numeric_limits<unsigned int>::max();
		state->bp = nullptr;

		for (int i = 0; i < num_heuristics(); ++i) {
			state->od[i].h = compute_heuristic(state->state_id, i);
			state->od[i].f = std::numeric_limits<unsigned int>::max();
		}

		state->closed = false;
	}
}

int WAStar::replan(
	std::vector<int>* solution_path, int* solution_cost)
{
	// if (is_goal(m_start_id))
	// {
	// 	// m_logger->LogMsg("Start is goal!", LogLevel::WARN);
	// 	solution_path->push_back(m_start_id);
	// 	solution_cost = 0;
	// 	return 1;
	// }

	for (int i = 0; i < num_heuristics(); ++i) {
		m_expands[i] = 0;
	}

	m_call_number++;

	reinit_state(m_goal);
	reinit_state(m_start);
	m_start->g = 0;
	m_start->od[0].f = compute_key(m_start, 0);

	// clear all OPEN lists
	for (int i = 0; i < num_heuristics(); ++i) {
		m_open[i].clear();
	}
	insert_or_update(m_start, 0);

	m_search_time = 0.0;
	while (!m_open[0].empty() && m_search_time < m_time_limit)
	{
		double expand_time = GetTime();

		SearchState* s = m_open[0].min()->me;
		if (is_goal(s->state_id))
		{
			extract_path(s, *solution_path, *solution_cost);
			m_search_time += GetTime() - expand_time;
			++m_expands[0];

			// SMPL_INFO("%d , %f", get_n_expands(), m_search_time);

			return 1;
		}
		m_open[0].pop();

		expand(s, 0);
		++m_expands[0];
		m_search_time += GetTime() - expand_time;
	}

	// SMPL_WARN("A* failed! OPEN size = %d", m_open[0].size());
	return 0;
}

void WAStar::expand(SearchState *s, int hidx)
{
	s->closed = true;

	std::vector<int> succ_ids;
	std::vector<unsigned int> costs;
	m_movable->GetSuccs(s->state_id, &succ_ids, &costs);

	for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)
	{
		unsigned int cost = costs[sidx];

		SearchState *succ_state = get_state(succ_ids[sidx]);
		reinit_state(succ_state);

		unsigned int new_g = s->g + costs[sidx];
		if (new_g < succ_state->g)
		{
			succ_state->g = new_g;
			succ_state->bp = s;
			if (!succ_state->closed)
			{
				unsigned int f_0 = compute_key(succ_state, 0);
				succ_state->od[0].f = f_0;
				insert_or_update(succ_state, 0);
			}
		}
	}
}

bool WAStar::is_goal(int state_id)
{
	return m_movable->IsGoal(state_id);
}

unsigned int WAStar::compute_heuristic(int state_id, int hidx)
{
	assert(num_heuristics() >= hidx);
	return m_movable->GetGoalHeuristic(state_id);

}

unsigned int WAStar::compute_key(SearchState *state, int hidx)
{
	return state->g + m_w * state->od[hidx].h;
}

void WAStar::insert_or_update(SearchState *state, int hidx)
{
	if (m_open[hidx].contains(&state->od[hidx])) {
		m_open[hidx].update(&state->od[hidx]);
	}
	else {
		m_open[hidx].push(&state->od[hidx]);
	}
}

void WAStar::reorder_open()
{
	for (auto hidx = 0; hidx < num_heuristics(); hidx++)
	{
		for (auto it = m_open[hidx].begin(); it != m_open[hidx].end(); ++it) {
			(*it)->f = compute_key((*it)->me, hidx);
		}
		m_open[hidx].make();
	}
}

void WAStar::extract_path(
	SearchState *s, std::vector<int>& solution, int& cost)
{
	cost = s->g;
	m_solution_cost = s->g;

	solution.clear();

	// s->state_id == m_goal_id == 0 should be true
	for (SearchState *state = s; state; state = state->bp) {
		solution.push_back(state->state_id);
	}
	std::reverse(solution.begin(), solution.end());
}

}  // namespace CMUPlanner
