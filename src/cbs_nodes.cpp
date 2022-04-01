#include <pushplan/cbs_nodes.hpp>

#include <numeric>

namespace clutter
{

void DAG::Remove(int from, int to)
{
	if (m_G.find(from) != m_G.end()) {
		m_G[from].erase(to);
	}
}

bool DAG::Connected(int from, int to) const
{
	std::list<int> OPEN;
	std::set<int> CLOSED;

	OPEN.push_back(from);
	while (!OPEN.empty())
	{
		int parent = OPEN.back();
		OPEN.pop_back();
		CLOSED.insert(parent);

		auto children = m_G.find(parent);
		if (children == m_G.end()) {
			continue;
		}

		for (auto child : children->second)
		{
			if (child == to) {
				return true;
			}

			if (CLOSED.find(child) == CLOSED.end())
			{
				if (std::find(OPEN.begin(), OPEN.end(), child) == OPEN.end()) {
					OPEN.push_back(child);
				}
			}
		}
	}

	return false;
}

std::unordered_set<int> DAG::GetHigherPriorities(int root)
{
	std::list<int> OPEN;
	std::unordered_set<int> CLOSED;

	OPEN.push_back(root);
	while (!OPEN.empty())
	{
		int parent = OPEN.back();
		OPEN.pop_back();

		auto children = m_G.find(parent);
		if (children == m_G.end()) {
			continue;
		}

		for (auto child : children->second)
		{
			if (CLOSED.find(child) == CLOSED.end())
			{
				CLOSED.insert(child);
				if (std::find(OPEN.begin(), OPEN.end(), child) == OPEN.end()) {
					OPEN.push_back(child);
				}
			}
		}
	}

	return CLOSED;
}

std::vector<int> DAG::GetParents(int root)
{
	std::vector<int> lower;
	for (const auto& n : m_G)
	{
		if (n.first == root) {
			continue;
		}

		if (n.second.find(root) != n.second.end())
		{
			auto it = std::find(lower.begin(), lower.end(), n.first);
			if (it == lower.end()) {
				lower.push_back(n.first);
			}
		}
	}

	return lower;
}

void DAG::TopologicalSort(std::stack<std::size_t>& order)
{
	std::vector<bool> v(m_G.size(), false);

	for (std::size_t i = 0; i < m_G.size(); ++i)
	{
		if (!v.at(i)) {
			traverse(i, v, order);
		}
	}
}

void DAG::TopologicalSort(const std::vector<int>& check, std::queue<int>& order)
{
	std::vector<bool> v(check.size(), false);

	for (std::size_t i = 0; i < check.size(); ++i)
	{
		if (!v.at(i)) {
			traverse(check, i, v, order);
		}
	}
}

void DAG::traverse(std::size_t i, std::vector<bool>& v, std::stack<std::size_t>& order)
{
	v.at(i) = true;
	auto git = m_G.begin();
	std::advance(git, i);

	for (auto child : git->second)
	{
		auto d = distance(m_G.begin(), m_G.find(child));
		if (!v.at(d)) {
			traverse(d, v, order);
		}
	}

	order.push(i);
}

void DAG::traverse(const std::vector<int>& check, std::size_t i, std::vector<bool>& v, std::queue<int>& order)
{
	v.at(i) = true;
	auto git = m_G.begin();
	auto d1 = std::distance(m_G.begin(), m_G.find(check.at(i)));
	std::advance(git, i);

	for (auto child : git->second)
	{
		auto it = std::find(check.begin(), check.end(), child);
		if (it != check.end())
		{
			auto d2 = std::distance(check.begin(), it);
			if (!v.at(d2)) {
				traverse(check, d2, v, order);
			}
		}
	}

	order.push(check.at(i));
}

void HighLevelNode::recalcFlowtime()
{
	m_flowtime = 0;
	for (const auto& s : m_solution) {
		m_flowtime += (int)s.second.size();
	}
}

void HighLevelNode::recalcMakespan()
{
	m_makespan = 0;
	for (const auto& s : m_solution) {
		m_makespan = std::max(m_makespan, (int)s.second.size());
	}
}

void HighLevelNode::recalcG(const std::vector<unsigned int>& min_fs)
{
	m_g = std::accumulate(min_fs.begin(), min_fs.end(), 0);
}

void HighLevelNode::updateDistanceToGo()
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

void HighLevelNode::computeH()
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

} // namespace clutter
