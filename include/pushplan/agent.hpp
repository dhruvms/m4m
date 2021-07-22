#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/wastar.hpp>

#include <smpl/types.h>

#include <memory>

namespace std {

template <>
struct hash<clutter::State>
{
	typedef clutter::State argument_type;
	typedef std::size_t result_type;
	result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace clutter
{

class CollisionChecker;

class Agent
{
public:
	Agent() {};
	Agent(const Object& o) {
		m_obj = o;
	};
	void SetObject(const Object& o) {
		m_obj = o;
	}

	void Init();
	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	void SetStartState(const State& s);
	void SetGoalState(const Point& p);

	void Search(int robin);
	bool AtGoal(const State& s, bool verbose=false);
	void Step(int k);

	const Object* GetObject() const { return &m_obj; };
	const State* GetCurrentState() const { return &m_current; };

	void reset(int phase);

	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);
	bool IsGoal(int state_id);
	unsigned int GetGoalHeuristic(int state_id);
	unsigned int GetGoalHeuristic(const State& s);

	int GetID() { return m_obj.id; };

private:
	Object m_obj;
	State m_current, m_start, m_init;
	Point m_goal;
	Pointf m_goalf;
	int m_t, m_priority, m_phase;
	Trajectory m_solve, m_move, m_retrieve;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<WAStar> m_wastar;

	int m_start_id, m_goal_id, m_expansions = 0;
	std::vector<State*> m_states;
	CLOSED m_closed;

	// maps from coords to stateID
	typedef State StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

	State* getHashEntry(int state_id) const;
	int getHashEntry(
		const int& x,
		const int& y,
		const int& t);
	int reserveHashEntry();
	int createHashEntry(
		const int& x,
		const int& y,
		const int& t);
	int getOrCreateState(
		const int& x,
		const int& y,
		const int& t);
	int getOrCreateState(const State& s);
	int getOrCreateState(const Point& p);

	int generateSuccessor(
		const State* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const State* s1,
		const State* s2);

	bool convertPath(
		const std::vector<int>& idpath);
};

} // namespace clutter


#endif // AGENT_HPP
