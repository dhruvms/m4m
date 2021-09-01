#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/movable.hpp>

#include <memory>

namespace clutter
{

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

	bool AtGoal(const LatticeState& s, bool verbose=false) override;
	void Step(int k) override;

	void GetSE2Push(std::vector<double>& push);

	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs) override;

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetGoalHeuristic(const LatticeState& s) override;

	const std::vector<Object>* GetObject(const LatticeState& s) override;
	using Movable::GetObject;

private:
	Object m_orig_o;

	int generateSuccessor(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2) override;
	bool convertPath(
		const std::vector<int>& idpath) override;
};

} // namespace clutter


#endif // AGENT_HPP
