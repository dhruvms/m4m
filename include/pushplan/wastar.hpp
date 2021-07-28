#ifndef WASTAR_HPP
#define WASTAR_HPP

#include <pushplan/types.hpp>

#include <smpl/heap/intrusive_heap.h>

#include <vector>

namespace clutter
{

struct SearchState
{
	int call_number;
	int state_id;
	unsigned int g;
	SearchState* bp;

	struct HeapData : public smpl::heap_element
	{
		// TODO: rather than map back to the state, the heap could know its
		// index into od for updates, though that might make it hard to
		// overallocate an array for the heap index, h, and f

		// TODO: in any case, this offset can be much smaller
		SearchState* me;
		unsigned int h;
		unsigned int f;
	};

	bool closed;
	HeapData od[1]; // overallocated for additional n heuristics
};

class Movable;

class WAStar : public Search
{
public:
	WAStar(
		Movable* movable,
		double w=1.0);
	~WAStar();

	int set_start(int start_id) override;
	int set_goal(int goal_id) override;
	void set_max_planning_time(double max_planning_time_ms) override {
		m_time_limit = max_planning_time_ms * 1e-3;
	};
	int get_n_expands() const override;
	void reset() override;

	int replan(
		std::vector<int>* solution_path, int* solution_cost) override;

private:
	Movable* m_movable = nullptr;

	struct HeapCompare {
		bool operator()(
				const SearchState::HeapData& s,
				const SearchState::HeapData& t) const
		{
			return s.f < t.f;
		}
	};

	using OPEN = smpl::intrusive_heap<SearchState::HeapData, HeapCompare>;
	OPEN* m_open = nullptr;  // sequence of (m_heur_count + 1) open lists

	// Search params
	int m_call_number;
	double m_time_limit, m_w;
	SearchState *m_goal, *m_start;

	int m_start_id, m_goal_id;

	std::vector<SearchState*> m_states;

	// Search statistics
	double m_search_time;
	int *m_expands; // expansions per queue
	int m_solution_cost;

	int num_heuristics() const { return 1; }

	SearchState* get_state(int state_id);
	// SearchState* create_state(int state_id);
	void init_state(SearchState *state, int state_id);
	void reinit_state(SearchState *state);

	void expand(SearchState *state, int hidx);
	bool is_goal(int state_id);

	unsigned int compute_heuristic(int state_id, int hidx);
	unsigned int compute_key(SearchState *state, int hidx);
	void insert_or_update(SearchState *state, int hidx);
	void reorder_open();

	void extract_path(
		SearchState *s, std::vector<int>& solution, int& cost);
};

}  // namespace clutter

#endif  // WASTAR_HPP
