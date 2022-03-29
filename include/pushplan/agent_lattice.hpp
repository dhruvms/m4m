#ifndef AGENT_LATTICE_HPP
#define AGENT_LATTICE_HPP

#include <pushplan/types.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/cbs_nodes.hpp>

#include <smpl/types.h>

#include <vector>

namespace clutter
{

class AgentLattice
{
public:

	void init(Agent* agent);
	void reset();

	void PushStart(const LatticeState& s);
	void PushGoal(const Coord& p);

	void SetCTNode(HighLevelNode* ct_node);
	void AvoidAgents(const std::unordered_set<int>& to_avoid);

	bool IsGoal(int state_id);
	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);

	unsigned int GetGoalHeuristic(int state_id);
	unsigned int GetConflictHeuristic(int state_id);
	unsigned int GetGoalHeuristic(const LatticeState& s);
private:

	Agent* m_agent = nullptr;

	std::vector<int> m_start_ids;
	std::vector<int> m_goal_ids;
	bool m_backwards;
	STATES m_states, m_closed;

	std::list<std::shared_ptr<Constraint> > m_constraints;
	std::vector<std::pair<int, Trajectory> >* m_cbs_solution; // all agent trajectories
	int m_cbs_id, m_max_time;
	std::unordered_set<int> m_to_avoid;

	// maps from coords to stateID
	typedef LatticeState StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

	int generateSuccessor(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2);

	int conflictHeuristic(const LatticeState& state);
	bool goalConflict(const LatticeState& state);

	LatticeState* getHashEntry(int state_id) const;
	int getHashEntry(
		const Coord& coord,
		const int& t);
	int reserveHashEntry();
	int createHashEntry(
		const Coord& coord,
		const State& state,
		const int& t,
		const int& hc);
	int getOrCreateState(
		const Coord& coord,
		const State& state,
		const int& t,
		const int& hc);
	int getOrCreateState(const LatticeState& s);
	int getOrCreateState(const Coord& p);

	bool convertPath(
		const std::vector<int>& idpath);
};

} // namespace clutter


#endif // AGENT_LATTICE_HPP
