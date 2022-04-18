#ifndef CBS_HPP
#define CBS_HPP

#include <pushplan/search/cbs_nodes.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <memory>

namespace clutter
{

class Robot;
class Agent;
class CollisionChecker;

class CBS
{
public:
	CBS();
	CBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs, int scene_id=-1);
	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}

	void SetRobot(std::shared_ptr<Robot> r) { m_robot = r; };
	void AddObject(std::shared_ptr<Agent> o) { m_objs.push_back(o); };
	void AddObjects(std::vector<std::shared_ptr<Agent> > objs) {
		m_objs.insert(m_objs.end(), objs.begin(), objs.end());
	};

	virtual bool Solve(bool backwards);
	void SaveStats();

protected:
	std::shared_ptr<CollisionChecker> m_cc;
	std::shared_ptr<Robot> m_robot;
	std::vector<std::shared_ptr<Agent> > m_objs;
	std::unordered_map<int, std::size_t> m_obj_id_to_idx;
	std::unordered_map<std::size_t, int> m_obj_idx_to_id;
	int m_num_agents;
	std::vector<Trajectory*> m_paths;
	std::vector<unsigned int> m_min_fs;

	int m_ct_generated, m_ct_deadends, m_ct_expanded, m_ll_expanded, m_soln_cost, m_scene_id, m_soln_lb, m_wf;
	double m_search_time, m_time_limit, m_ll_time, m_conflict_time;
	bool m_solved, m_backwards;
	HighLevelNode* m_goal;

	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::OPENCompare> > m_OPEN;
	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::FOCALCompare> > m_FOCAL;

	virtual bool initialiseRoot();
	virtual void growConstraintTree(HighLevelNode* parent);
	void pushNode(HighLevelNode* node);

	void findConflicts(HighLevelNode& node);
	void findConflictsRobot(HighLevelNode& curr, size_t oid);
	void findConflictsObjects(HighLevelNode& curr, size_t o1, size_t o2);

	bool done(HighLevelNode* node);

	void copyRelevantConflicts(HighLevelNode& node) const;
	void selectConflict(HighLevelNode* node) const;
	void addConstraints(
		const HighLevelNode* curr,
		HighLevelNode* child1, HighLevelNode* child2) const;
	virtual bool updateChild(HighLevelNode* parent, HighLevelNode* child);

	void writeSolution(HighLevelNode* node);
};

} // namespace clutter


#endif // CBS_HPP
