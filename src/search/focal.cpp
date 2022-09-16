#include <pushplan/agents/agent_lattice.hpp>
#include <pushplan/search/focal.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/types.hpp>
#include <pushplan/utils/helpers.hpp>

#include <smpl/console/console.h>

#include <iostream>
#include <algorithm>
#include <limits>

namespace clutter
{

Focal::Focal(
	AgentLattice* agent,
	double wf, double wo)
:
m_agent(agent),
m_call_number(0),
m_wf(wf), m_wo(wo)
{
	// Set default max planing time
	m_time_limit = 30.0; // seconds

	m_expands = new int[1];
	m_min_f = std::numeric_limits<int>::max();
}

Focal::~Focal()
{
	reset();
}

int Focal::set_start(int start_id)
{
	m_start_ids.clear();
	m_start_ids.push_back(start_id);
	get_state(m_start_ids.back(), m_b);
	return m_start_ids.back();
}

int Focal::set_goal(int goal_id)
{
	m_goal_ids.clear();
	m_goal_ids.push_back(goal_id);
	get_state(m_goal_ids.back(), m_b);
	return m_goal_ids.back();
}

std::size_t Focal::push_start(int start_id)
{
	m_start_ids.push_back(start_id);
	get_state(m_start_ids.back(), m_b);
	return m_start_ids.size();
}

std::size_t Focal::push_goal(int goal_id)
{
	m_goal_ids.push_back(goal_id);
	get_state(m_goal_ids.back(), m_b);
	return m_goal_ids.size();
}

int Focal::get_n_expands() const
{
	return m_expands[0];
}

int Focal::get_min_f() const
{
	return m_min_f;
}

void Focal::reset()
{
	// Clear OPEN and FOCAL lists
	m_OPEN.clear();
	m_FOCAL.clear();

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

	m_min_f = std::numeric_limits<int>::max();

	m_start_ids.clear();
	m_goal_ids.clear();
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
LowLevelNode* Focal::get_state(int state_id, bool& alloc)
{
	assert(state_id >= 0);

	if (m_states.size() <= state_id)
	{
		LowLevelNode* s = new LowLevelNode();
		init_state(s, state_id);
		m_states.push_back(s);
		alloc = true;

		return s;
	}

	alloc = false;
	return m_states[state_id];
}

void Focal::init_state(LowLevelNode* state, int state_id)
{
	state->call_number = 0; // not initialized for any iteration
	state->state_id = state_id;
}

// Lazily (re)initialize a search state.
void Focal::reinit_state(LowLevelNode* state)
{
	if (state->call_number != m_call_number)
	{
		state->call_number = m_call_number;
		state->g = std::numeric_limits<unsigned int>::max();
		state->h = compute_heuristic(state->state_id);
		state->f = std::numeric_limits<unsigned int>::max();
		state->h_c = std::numeric_limits<unsigned int>::max();
		state->bp = nullptr;
		state->closed = false;
	}
}

int Focal::replan(
	std::vector<int>* solution_path, int* solution_cost)
{
	m_expands[0] = 0;
	m_call_number++;

	m_OPEN.clear();
	m_FOCAL.clear();

	for (const auto& goal_id : m_goal_ids)
	{
		auto goal_state = get_state(goal_id, m_b);
		assert(!m_b);
		reinit_state(goal_state);
	}
	for (const auto& start_id : m_start_ids)
	{
		auto start_state = get_state(start_id, m_b);
		assert(!m_b);
		reinit_state(start_state);
		start_state->g = 0;
		start_state->f = compute_key(start_state);
		start_state->h_c = 0;
		start_state->m_OPEN_h = m_OPEN.push(start_state);
		start_state->m_FOCAL_h = m_FOCAL.push(start_state);
		m_min_f = std::min(m_min_f, (int)start_state->f);
	}

	m_search_time = 0.0;
	while (!m_OPEN.empty() && m_search_time < m_time_limit)
	{
		update_focal_list();

		double expand_time = GetTime();

		LowLevelNode* s = m_FOCAL.top();
		m_FOCAL.pop();
		m_OPEN.erase(s->m_OPEN_h);

		if (is_goal(s->state_id))
		{
			extract_path(s, *solution_path, *solution_cost);
			m_search_time += GetTime() - expand_time;
			++m_expands[0];

			return 1;
		}

		expand(s);
		++m_expands[0];
		m_search_time += GetTime() - expand_time;
	}

	// SMPL_WARN("FOCAL failed! OPEN size = %d", m_OPEN.size());
	return 0;
}

void Focal::expand(LowLevelNode* s)
{
	s->closed = true;

	std::vector<int> succ_ids;
	std::vector<unsigned int> costs;
	m_agent->GetSuccs(s->state_id, &succ_ids, &costs);

	for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)
	{
		unsigned int cost = costs[sidx];

		bool alloc;
		LowLevelNode* succ_state = get_state(succ_ids[sidx], alloc);
		reinit_state(succ_state);

		unsigned int new_g = s->g + costs[sidx];
		unsigned int new_hc = compute_conflict_heuristic(succ_ids[sidx]);

		if (alloc)
		{
			// first time seeing this state
			succ_state->g = new_g;
			succ_state->h_c = new_hc;
			succ_state->f = compute_key(succ_state);
			succ_state->bp = s;
			push_node(succ_state);

			continue;
		}

		if (new_g < succ_state->g || (new_g == succ_state->g && new_hc < succ_state->h_c))
		{
			succ_state->g = new_g;
			succ_state->h_c = new_hc;
			succ_state->bp = s;
			unsigned int new_f = compute_key(succ_state);

			if (!succ_state->closed)
			{
				bool add_f = false, update_f = false, update_o = false;
				if (new_f <= m_wf * m_min_f)
				{
					if (succ_state->f > m_wf * m_min_f) {
						add_f = true;
					}
					else {
						update_f = true;
					}
				}
				if (succ_state->f > new_f) {
					update_o = true;
				}

				succ_state->f = new_f;
				if (update_o) {
					m_OPEN.increase(succ_state->m_OPEN_h);
				}
				if (add_f) {
					succ_state->m_FOCAL_h = m_FOCAL.push(succ_state);
				}
				if (update_f) {
					m_FOCAL.update(succ_state->m_FOCAL_h);
				}
			}
			else
			{
				succ_state->f = new_f;
				push_node(succ_state);
			}
		}
	}
}

bool Focal::is_goal(int state_id)
{
	return m_agent->IsGoal(state_id);
}

unsigned int Focal::compute_heuristic(int state_id)
{
	return m_agent->GetGoalHeuristic(state_id);
}

unsigned int Focal::compute_conflict_heuristic(int state_id)
{
	return m_agent->GetConflictHeuristic(state_id);
}

unsigned int Focal::compute_key(LowLevelNode* state)
{
	return state->g + m_wo * state->h;
}

void Focal::push_node(LowLevelNode* state)
{
	state->m_OPEN_h = m_OPEN.push(state);
	state->closed = false;
	if (state->f <= m_wf * m_min_f) {
		state->m_FOCAL_h = m_FOCAL.push(state);
	}
}

void Focal::update_focal_list()
{
	LowLevelNode* o_top = m_OPEN.top();
	if (o_top->f > m_min_f)
	{
		unsigned int new_min_f = o_top->f;
		for (auto n: m_OPEN)
		{
			if (n->f > m_wf * m_min_f && n->f <= m_wf * new_min_f) {
				n->m_FOCAL_h = m_FOCAL.push(n);
			}
		}
		m_min_f = new_min_f;
	}
}

void Focal::extract_path(
	LowLevelNode* s, std::vector<int>& solution, int& cost)
{
	cost = s->g;
	m_solution_cost = s->g;

	solution.clear();

	// s->state_id == m_goal_id == 0 should be true
	for (LowLevelNode* state = s; state; state = state->bp) {
		solution.push_back(state->state_id);
	}
	std::reverse(solution.begin(), solution.end());
}

}  // namespace CMUPlanner
