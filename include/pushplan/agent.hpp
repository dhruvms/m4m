#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/movable.hpp>

#include <memory>

namespace clutter
{

class HighLevelNode;
class Robot;
class Constraint;

class Agent : public Movable
{
public:
	Agent() {};
	Agent(const Object& o) {
		m_objs.push_back(o);
	};
	void SetObject(const Object& o) {
		m_objs.push_back(o);
	}

	bool Setup() override;
	void ResetObject();
	bool SetObjectPose(
		const std::vector<double>& xyz,
		const std::vector<double>& rpy);
	bool Init() override;

	bool SatisfyPath(HighLevelNode* ct_node, Robot* robot, Trajectory** sol_path);
	bool SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path);
	bool IsGoal(int state_id) override;
	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs) override;

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetGoalHeuristic(const LatticeState& s) override;

	void GetSE2Push(std::vector<double>& push);
	const std::vector<Object>* GetObject(const LatticeState& s) override;
	using Movable::GetObject;

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

	bool knownConflict(const LatticeState& state);
	bool goalConflict(const LatticeState& state);

	int generateSuccessor(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2,
		bool movable=false) override;
	bool convertPath(
		const std::vector<int>& idpath) override;
};

} // namespace clutter


#endif // AGENT_HPP
