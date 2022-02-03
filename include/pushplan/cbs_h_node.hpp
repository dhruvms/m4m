#ifndef CBS_H_NODE_HPP
#define CBS_H_NODE_HPP

#include <pushplan/conflicts.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <list>
#include <memory>

namespace clutter
{

struct HighLevelNode
{
	struct HeapCompare {
		bool operator()(
				const HighLevelNode* s, const HighLevelNode* t) const
		{
			return s->m_g < t->m_g;
		}
	};

	std::list<std::shared_ptr<Constraint> > m_constraints; // constraints to be satisfied (parent + 1)
	std::list<std::shared_ptr<Conflict> > m_conflicts; // conflicts at this node
	std::shared_ptr<Conflict> m_conflict; // selected conflict
	std::vector<std::pair<int, Trajectory> > m_solution; // agent solutions

	int m_g, m_depth, m_makespan, m_generate, m_expand;
	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::HeapCompare> >::handle_type m_OPEN_h;

	HighLevelNode* m_parent;
	std::vector<HighLevelNode*> m_children;
	int m_replanned;

	void recalcMakespan()
	{
		m_makespan = 0;
		for (const auto& s : m_solution) {
			m_makespan = std::max(m_makespan, (int)s.second.size());
		}
	}

	void clear()
	{
		m_conflicts.clear();
	}
};

} // namespace clutter


#endif // CBS_H_NODE_HPP
