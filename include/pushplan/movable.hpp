#ifndef MOVABLE_HPP
#define MOVABLE_HPP

#include <pushplan/types.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/wastar.hpp>

#include <smpl/types.h>

#include <vector>
#include <string>
#include <memory>

namespace clutter
{

class Movable
{
public:
	virtual bool Init() = 0;

	void SetStartState(const LatticeState& s);
	void SetGoalState(const Coord& p);

	bool Search(int robin);
	virtual bool AtGoal(const LatticeState& s, bool verbose=false) = 0;
	virtual void Step(int k) = 0;

	bool IsGoal(int state_id);
	virtual void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs) = 0;
	virtual unsigned int GetGoalHeuristic(int state_id) = 0;
	virtual unsigned int GetGoalHeuristic(const LatticeState& s) = 0;

	void reset(int phase);

	const LatticeState* GetCurrentState() const { return &m_current; };
	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	const std::vector<Object>* GetObject() const { return &m_objs; };
	virtual const std::vector<Object>* GetObject(const LatticeState& s) = 0;

protected:
	std::vector<Object> m_objs;
	LatticeState m_current, m_init;
	Coord m_start, m_goal;
	State m_goalf;
	int m_t, m_priority, m_phase;

	int m_start_id, m_goal_id, m_expansions = 0;
	STATES m_states, m_closed;
	Trajectory m_solve, m_move, m_retrieve;
	int m_retrieved;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<WAStar> m_wastar;

	// maps from coords to stateID
	typedef LatticeState StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

	virtual unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2) = 0;
	virtual bool convertPath(
		const std::vector<int>& idpath) = 0;

	LatticeState* getHashEntry(int state_id) const;
	int getHashEntry(
		const Coord& coord,
		const int& t);
	int reserveHashEntry();
	int createHashEntry(
		const Coord& coord,
		const State& state,
		const int& t);
	int getOrCreateState(
		const Coord& coord,
		const State& state,
		const int& t);
	int getOrCreateState(const LatticeState& s);
	int getOrCreateState(const Coord& p);
};

} // namespace clutter


#endif // MOVABLE_HPP
