#ifndef AGENT_LATTICE_HPP
#define AGENT_LATTICE_HPP

namespace clutter
{

class AgentLattice
{
public:
	void reset();

	void SetStartState(const LatticeState& s);
	void SetGoalState(const Coord& p);

	bool IsGoal(int state_id);
	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);

	unsigned int GetGoalHeuristic(int state_id);
	unsigned int GetConflictHeuristic(int state_id);
	unsigned int GetGoalHeuristic(const LatticeState& s);
private:

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
