#ifndef PBS_HPP
#define PBS_HPP

#include <pushplan/search/cbs.hpp>
#include <pushplan/search/cbs_nodes.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <memory>
#include <stack>

namespace clutter
{

class Robot;
class Agent;
class CollisionChecker;

class PBS : public CBS
{
public:
	PBS();
	PBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs, int scene_id=-1);

	void InitPriorities(const DAG& p0) {
		m_p0.Copy(p0);
	}
	void AddInitPriority(int from, int to) {
		m_p0.Add(from, to);
	}

	bool Solve(bool backwards);
	void SaveStats();

protected:
	std::list<HighLevelNode*> m_OPEN;
	DAG m_p0;

	bool updatePlan(HighLevelNode* node, int agent_id);
	void removeConflicts(HighLevelNode* node, int agent_id);
	void findConflicts(HighLevelNode* node, int agent_id);

	virtual bool initialiseRoot() override;
	virtual void growConstraintTree(HighLevelNode* parent) override;
	virtual bool updateChild(HighLevelNode* parent, HighLevelNode* child) override;
};

} // namespace clutter


#endif // PBS_HPP
