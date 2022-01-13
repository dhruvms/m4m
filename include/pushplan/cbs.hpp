#ifndef CBS_HPP
#define CBS_HPP

#include <pushplan/cbs_h_node.hpp>

#include <boost/heap/fibonacci_heap.hpp>

namespace clutter
{

class Robot;
class Agent;

class CBS
{
public:
	CBS();
	CBS(Robot* r, std::vector<Agent*> objs);

	void SetRobot(Robot* r) { m_robot = r; };
	void AddObject(Agent* o) { m_objs.push_back(o); };
	void AddObjects(std::vector<Agent*> objs) {
		m_objs.insert(m_objs.end(), objs.begin(), objs.end());
	};

	bool Solve();

private:
	Robot* m_robot;
	std::vector<Agent*> m_objs;
	int m_num_agents;
	std::vector<Trajectory*> m_paths;

	int m_ct_generated, m_ct_expanded, m_soln_cost;
	bool m_solved;
	HighLevelNode* m_goal;

	boost::heap::fibonacci_heap<HighLevelNode, boost::heap::compare<HighLevelNode::HeapCompare> > m_OPEN;

	void initialiseRoot();

	void findConflicts(HighLevelNode& node);
	void findConflicts(HighLevelNode& curr, Robot* r, Agent* a);
	void findConflicts(HighLevelNode& curr, Agent* a1, Agent* a2);

	bool done(HighLevelNode* node);

	void copyRelevantConflicts(HighLevelNode& node) const;
	void selectConflict(HighLevelNode* node) const;
	void addConstraints(
		const HighLevelNode* curr,
		HighLevelNode* child1, HighLevelNode* child2) const;
	bool updateChild(HighLevelNode* parent, HighLevelNode* child);
};

} // namespace clutter


#endif // CBS_HPP
