#ifndef CBS_HPP
#define CBS_HPP

#include <pushplan/cbs_h_node.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <memory>

namespace clutter
{

class Robot;
class Agent;

class CBS
{
public:
	CBS();
	CBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs);

	void SetRobot(std::shared_ptr<Robot> r) { m_robot = r; };
	void AddObject(std::shared_ptr<Agent> o) { m_objs.push_back(o); };
	void AddObjects(std::vector<std::shared_ptr<Agent> > objs) {
		m_objs.insert(m_objs.end(), objs.begin(), objs.end());
	};

	bool Solve();

private:
	std::shared_ptr<Robot> m_robot;
	std::vector<std::shared_ptr<Agent> > m_objs;
	int m_num_agents;
	std::vector<Trajectory*> m_paths;

	int m_ct_generated, m_ct_expanded, m_soln_cost;
	bool m_solved;
	HighLevelNode* m_goal;

	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::HeapCompare> > m_OPEN;

	void initialiseRoot();

	void findConflicts(HighLevelNode& node);
	void findConflicts(HighLevelNode& curr, size_t oid);
	void findConflicts(HighLevelNode& curr, size_t o1, size_t o2);

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
