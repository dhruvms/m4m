#ifndef CBS_H_NODE_HPP
#define CBS_H_NODE_HPP

#include <pushplan/conflicts.hpp>
#include <pushplan/constants.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <list>
#include <memory>
#include <set>

namespace clutter
{

struct HighLevelNode
{
	struct OPENCompare
	{
		bool operator()(const HighLevelNode* p, const HighLevelNode* q) const
		{
			if (p->m_g + p->m_h == q->m_g + q->m_h)
			{
				if (p->m_d == q->m_d)
				{
					if (p->m_soc == q->m_soc)
					{
						return p->m_h >= q->m_h;
					}

					return p->m_soc >= q->m_soc;
				}

				return p->m_d >= q->m_d;
			}

			return p->m_g + p->m_h >= q->m_g + q->m_h;
		}
	};

	struct FOCALCompare
	{
		bool operator()(const HighLevelNode* p, const HighLevelNode* q) const
		{
			if (p->m_d == q->m_d)
			{
				if (p->m_g + p->m_h == q->m_g + q->m_h)
				{
					if (p->m_soc == q->m_soc)
					{
						return p->m_h >= q->m_h;
					}

					return p->m_soc >= q->m_soc;
				}

				return p->m_g + p->m_h >= q->m_g + q->m_h;
			}

			return p->m_d >= q->m_d;
		}
	};

	std::list<std::shared_ptr<Constraint> > m_constraints; // constraints to be satisfied (parent + 1)
	std::list<std::shared_ptr<Conflict> > m_conflicts; // conflicts at this node
	std::shared_ptr<Conflict> m_conflict; // selected conflict
	std::vector<std::pair<int, Trajectory> > m_solution; // agent solutions

	int m_g, m_soc, m_depth, m_makespan, m_generate, m_expand;
	int m_h, m_d;
	bool m_h_computed;

	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::OPENCompare> >::handle_type m_OPEN_h;
	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::FOCALCompare> >::handle_type m_FOCAL_h;

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

	void updateDistanceToGo()
	{
		std::set<std::pair<int, int> > conflict_pairs;
		for (auto& conflict : m_conflicts)
		{
			auto pair = std::make_pair(std::min(conflict->m_a1, conflict->m_a2), std::max(conflict->m_a1, conflict->m_a2));
			if (conflict_pairs.find(pair) == conflict_pairs.end()) {
				conflict_pairs.insert(pair);
			}
		}
		m_d = (int)(m_conflicts.size() + conflict_pairs.size());
	}

	void computeH()
	{
		m_h_computed = true;
		int h = -1;
		switch (HLHC)
		{
			case HighLevelConflictHeuristic::CONFLICT_COUNT:
			{
				h = (int)m_conflicts.size();
				break;
			}
			case HighLevelConflictHeuristic::AGENT_COUNT:
			{
				std::set<int> conflict_agents;
				for (auto& conflict : m_conflicts)
				{
					if (conflict_agents.find(conflict->m_a1) == conflict_agents.end()) {
						conflict_agents.insert(conflict->m_a1);
					}
					if (conflict_agents.find(conflict->m_a2) == conflict_agents.end()) {
						conflict_agents.insert(conflict->m_a2);
					}
				}
				h = (int)conflict_agents.size();
				break;
			}
			case HighLevelConflictHeuristic::AGENT_PAIRS:
			{
				h = m_d - (int)m_conflicts.size();
				break;
			}
		}
		m_h = std::max(h, m_h);
	}
};

struct LowLevelNode
{
	int call_number;
	int state_id;
	unsigned int g, h, f, h_c;
	LowLevelNode* bp;
	bool closed;

	struct OPENCompare
	{
		bool operator()(const LowLevelNode* p, const LowLevelNode* q) const
		{
			if (p->f == q->f)
			{
				if (p->h == q->h)
				{
					return rand() % 2 == 0;
				}

				return p->h >= q->h;
			}

			return p->f >= q->f;
		}
	};

	struct FOCALCompare
	{
		bool operator()(const LowLevelNode* p, const LowLevelNode* q) const
		{
			if (p->h_c == q->h_c)
			{
				if (p->f == q->f)
				{
					if (p->h == q->h)
					{
						return rand() % 2 == 0;
					}

					return p->h >= q->h;
				}

				return p->f >= q->f;
			}

			return p->h_c >= q->h_c;
		}
	};

	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::OPENCompare> >::handle_type m_OPEN_h;
	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::FOCALCompare> >::handle_type m_FOCAL_h;
};

} // namespace clutter


#endif // CBS_H_NODE_HPP
