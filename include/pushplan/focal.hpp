#ifndef FOCAL_HPP
#define FOCAL_HPP

#include <pushplan/types.hpp>
#include <pushplan/cbs_nodes.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <vector>

namespace clutter
{

class AgentLattice;

class Focal : public Search
{
public:
	Focal(
		AgentLattice* agent,
		double wf=1.0, double wo=1.0);
	~Focal();

	int set_start(int start_id) override;
	int set_goal(int goal_id) override;
	std::size_t push_start(int start_id) override;
	std::size_t push_goal(int goal_id) override;

	void set_max_planning_time(double max_planning_time_ms) override {
		m_time_limit = max_planning_time_ms * 1e-3;
	};
	int get_n_expands() const override;
	int get_min_f() const override;
	void reset() override;

	int replan(
		std::vector<int>* solution_path, int* solution_cost) override;

private:
	AgentLattice* m_agent = nullptr;

	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::OPENCompare> > m_OPEN;
	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::FOCALCompare> > m_FOCAL;
	std::vector<LowLevelNode*> m_states;

	// Search params
	int m_call_number;
	double m_time_limit, m_wo, m_wf;
	LowLevelNode *m_goal, *m_start;

	bool m_b;

	LowLevelNode* get_state(int state_id, bool& alloc);
	// LowLevelNode* create_state(int state_id);
	void init_state(LowLevelNode *state, int state_id);
	void reinit_state(LowLevelNode *state);

	void expand(LowLevelNode *state);
	bool is_goal(int state_id);

	unsigned int compute_heuristic(int state_id);
	unsigned int compute_conflict_heuristic(int state_id);
	unsigned int compute_key(LowLevelNode *state);
	void push_node(LowLevelNode* state);
	void update_focal_list();

	void extract_path(
		LowLevelNode *s, std::vector<int>& solution, int& cost);
};

}  // namespace clutter

#endif  // FOCAL_HPP
