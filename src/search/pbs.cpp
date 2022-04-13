#include <pushplan/agents/robot.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/pbs.hpp>
#include <pushplan/search/cbs.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/console/console.h>

namespace clutter
{

PBS::PBS() :
CBS()
{
	m_OPEN.clear();
	m_p0.Clear();
}

PBS::PBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs,
	int scene_id) :
CBS(r, objs, scene_id)
{
	m_OPEN.clear();
	m_p0.Clear();
}

bool PBS::Solve(bool backwards)
{
	m_backwards = backwards;
	m_search_time = 0.0;
	m_conflict_time = 0.0;
	m_ll_time = 0.0;
	double start_time = GetTime();
	if (!initialiseRoot()) {
		SMPL_ERROR("Failed to initialiseRoot");
		return false;
	}
	m_search_time += GetTime() - start_time; // add initialiseRoot time

	while (!m_OPEN.empty())
	{
		start_time = GetTime(); // reset clock with every loop iteration

		auto next = m_OPEN.back();
		m_OPEN.pop_back();

		selectConflict(next);
		if (done(next)) {
			m_search_time += GetTime() - start_time;
			SMPL_WARN("YAYAYAY! We did it!");
			CBS::writeSolution(next);
			return m_solved;
		}
		// writeSolution(next);

		++m_ct_expanded;
		next->m_expand = m_ct_expanded;

		growConstraintTree(next);

		m_search_time += GetTime() - start_time;
	}

	SMPL_ERROR("CBS high-level OPEN is empty");
	return false;
}

void PBS::growConstraintTree(HighLevelNode* parent)
{
	// expand CT node
	HighLevelNode* child[2] = { new HighLevelNode(), new HighLevelNode() };
	int child_cost[2];
	addConstraints(parent, child[0], child[1]);
	for (int i = 0; i < 2; ++i)
	{
		if (updateChild(parent, child[i])) {
			parent->m_children.push_back(child[i]);
			child_cost[i] = child[i]->m_flowtime;
		}
		else {
			delete (child[i]);
			child_cost[i] = -1;
			continue;
		}
	}

	// push child nodes into stack in non-decreasing cost order
	if (child_cost[0] >= 0)
	{
		if (child_cost[1] >= 0)
		{
			if (child_cost[0] < child_cost[1])
			{
				m_OPEN.push_back(child[0]);
				m_OPEN.push_back(child[1]);
			}
			else
			{
				m_OPEN.push_back(child[1]);
				m_OPEN.push_back(child[0]);
			}
		}
		else {
			m_OPEN.push_back(child[0]);
		}
	}
	else if (child_cost[1] >= 0) {
		m_OPEN.push_back(child[1]);
	}

	parent->clear();
}

bool PBS::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_flowtime = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();
	root->m_priorities.Copy(m_p0);

	// Plan for robot
	int expands, min_f;
	double start_time = GetTime();
	// if (!m_robot->SatisfyPath(root, &m_paths[0], expands, min_f)) { // (CT node, path location)
	// 	++m_ct_deadends;
	// 	return false;
	// }
	// m_ll_time += GetTime() - start_time;
	// m_ll_expanded += expands;
	// m_min_fs[0] = min_f;
	// root->m_solution.emplace_back(m_robot->GetID(), *(m_paths[0]));
	// root->m_g += m_min_fs[0];
	// root->m_flowtime += m_paths[0]->size();
	// root->m_makespan = std::max(root->m_makespan, (int)m_paths[0]->size());

	// Plan for objects
	std::stack<std::size_t> order;
	root->m_priorities.TopologicalSort(order);
	std::vector<bool> planned(m_objs.size(), false);

	while (!order.empty())
	{
		auto git = root->m_priorities.GetDAG().begin();
	    std::advance(git, order.top());
	    int agent_id = git->first;

		start_time = GetTime();
		if (!updatePlan(root, agent_id)) {
			return false;
		}
		m_ll_time += GetTime() - start_time;

		planned.at(m_obj_id_to_idx[agent_id]) = true;
	    order.pop();
	}

	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		if (!planned.at(i))
		{
			start_time = GetTime();
			if (!updatePlan(root, m_obj_idx_to_id[i])) {
				return false;
			}
			m_ll_time += GetTime() - start_time;
		}
		planned.at(i) = true;
	}

	// root->m_conflicts.clear();
	CBS::findConflicts(*root);

	root->m_h = 0;
	root->m_h_computed = false;
	root->recalcMakespan();
	root->recalcFlowtime();
	root->recalcG(m_min_fs);
	// root->updateDistanceToGo();

	m_OPEN.push_back(root);
	return true;
}

bool PBS::updateChild(HighLevelNode* parent, HighLevelNode* child)
{
	child->m_g = parent->m_g; // for now
	child->m_flowtime = parent->m_flowtime; // for now
	child->m_makespan = parent->m_makespan; // for now
	child->m_solution = parent->m_solution; // for now
	child->m_depth = parent->m_depth + 1;
	child->m_parent = parent;
	child->m_children.clear();

	// agent to be replanned for
	child->m_replanned = child->m_constraints.back()->m_me;
	int agent_id = child->m_replanned;
	std::size_t agent_idx = m_obj_id_to_idx[agent_id];

	child->m_priorities.Copy(parent->m_priorities);
	// Give other agent involved in conflict higher priority
	child->m_priorities.Add(agent_id, child->m_constraints.back()->m_other);

	double start_time = GetTime();
	if (!updatePlan(child, child->m_replanned)) {
		return false;
	}
	m_ll_time += GetTime() - start_time;

	child->m_h = 0;
	child->m_h_computed = false;
	child->recalcMakespan();
	child->recalcFlowtime();
	child->recalcG(m_min_fs);

	return true;
}

bool PBS::updatePlan(HighLevelNode* node, int agent_id)
{
	// get nodes that have lower priority than me - consider replanning for all of these
	std::vector<int> replan_ids = node->m_priorities.GetParents(agent_id);
	// consider replanning for me
	replan_ids.push_back(agent_id);
	// topological sort agents to replan to determine good order to replan in
	std::queue<int> replan_order;
	if (!node->m_priorities.Empty() && node->m_priorities.Contains(agent_id)) {
		node->m_priorities.TopologicalSort(replan_ids, replan_order);
	}
	else {
		replan_order.push(agent_id);
	}

	while (!replan_order.empty())
	{
		int replan_id = replan_order.front();
		auto replan_idx = m_obj_id_to_idx[replan_id];
		bool replan = replan_id == agent_id ? true : false;

		// no need to replan if existing path is conflict free
		if (!replan && m_paths[replan_idx] != nullptr)
		{
			// have a path for this agent "replan_id"
			for (const auto& conflict : node->m_conflicts)
			{
				if (conflict->m_a1 == replan_id || conflict->m_a2 == replan_id)
				{
					// path has conflicts, need to replan it
					replan = true;
					break;
				}
			}
		}
		else if (!replan && m_paths[replan_idx] == nullptr)
		{
			// do not have a path for this agent "replan_id", must (re)plan one
			replan = true;
		}

		if (!replan)
		{
			replan_order.pop();
			continue;
		}

		// find high-priority agents with which to avoid collisions
		auto to_avoid = node->m_priorities.GetHigherPriorities(replan_id);
		int expands, min_f;
		// run low-level with spacetime collision checking against higher priority agents
		m_objs[replan_idx]->Init(m_backwards);
		if (!m_objs[replan_idx]->SatisfyPath(node, &m_paths[replan_idx], expands, min_f, &to_avoid)) {
			++m_ct_deadends;
			return false;
		}
		m_ll_expanded += expands;
		m_min_fs[replan_idx] = min_f;

		if (node->m_parent != nullptr)
		{
			// update conflicts replanned agent is involved in
			removeConflicts(node, replan_id);
			// find new conflicts created by path just computed
			std::list<std::shared_ptr<Conflict> > new_conflicts;
			findConflicts(node, replan_id);
		}

		// update tree node's solution
		bool soln_updated = false;
		for (auto& agent_soln : node->m_solution)
		{
			if (agent_soln.first == replan_id)
			{
				agent_soln.second = *(m_paths[replan_idx]);
				soln_updated = true;
				break;
			}
		}
		if (!soln_updated) {
			node->m_solution.emplace_back(replan_id, *(m_paths[replan_idx]));
		}

		// on to the next one
		replan_order.pop();
	}

	return true;
}

void PBS::removeConflicts(HighLevelNode* node, int agent_id)
{
	for (auto it = node->m_conflicts.begin(); it != node->m_conflicts.end(); )
	{
		if ((*it)->m_a1 == agent_id || (*it)->m_a2 == agent_id) {
			it = node->m_conflicts.erase(it);
		}
		else {
			++it;
		}
	}
}

void PBS::findConflicts(HighLevelNode* node, int agent_id)
{
	auto agent_idx = m_obj_id_to_idx[agent_id];
	// findConflictsRobot(*node, agent_idx);
	for (size_t k = 0; k < m_objs.size(); ++k)
	{
		if (m_obj_idx_to_id[k] == agent_id) {
			continue;
		}
		findConflictsObjects(*node, agent_idx, k);
	}
}

void PBS::SaveStats()
{
	CBS::SaveStats();
}

} // namespace clutter
