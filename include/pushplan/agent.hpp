#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/object.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/focal.hpp>

#include <smpl/types.h>

#include <vector>
#include <string>
#include <memory>

namespace clutter
{

class HighLevelNode;
class Constraint;

class Agent
{
public:
	Agent() {};
	Agent(const Object& o) {
		m_objs.push_back(o);
	};
	void SetObject(const Object& o) {
		m_objs.push_back(o);
	}

	bool Setup();
	void ResetObject();
	bool SetObjectPose(
		const std::vector<double>& xyz,
		const std::vector<double>& rpy);
	bool Init();
	void reset();

	void SetStartState(const LatticeState& s);
	void SetGoalState(const Coord& p);
	bool SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f);
	bool IsGoal(int state_id);
	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);

	unsigned int GetGoalHeuristic(int state_id);
	unsigned int GetConflictHeuristic(int state_id);
	unsigned int GetGoalHeuristic(const LatticeState& s);

	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}

	void GetSE2Push(std::vector<double>& push);
	const Trajectory* GetLastTraj() const { return &m_solve; };
	int GetID() { return m_objs.back().id; };

	const std::vector<Object>* GetObject() const { return &m_objs; };
	const std::vector<Object>* GetObject(const LatticeState& s);
	fcl::CollisionObject* GetFCLObject() { return m_objs.back().GetFCLObject(); };
	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		m_objs.back().GetMoveitObj(msg);
	};

	void UpdatePose(const LatticeState&s) { m_objs.back().UpdatePose(s); };

private:
	Object m_orig_o;
	std::list<std::shared_ptr<Constraint> > m_constraints;
	std::vector<std::pair<int, Trajectory> >* m_cbs_solution; // all agent trajectories
	int m_cbs_id, m_max_time;

	std::vector<Object> m_objs;
	LatticeState m_init;
	Coord m_start, m_goal;
	State m_goalf;
	int m_t, m_priority;

	int m_start_id, m_goal_id, m_expansions = 0;
	STATES m_states, m_closed;
	Trajectory m_solve;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Focal> m_focal;

	// maps from coords to stateID
	typedef LatticeState StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

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

	int conflictHeuristic(const LatticeState& state);
	bool goalConflict(const LatticeState& state);

	int generateSuccessor(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2);
	bool convertPath(
		const std::vector<int>& idpath);
};

} // namespace clutter


#endif // AGENT_HPP
