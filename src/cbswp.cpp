#include <pushplan/cbswp.hpp>
#include <pushplan/helpers.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/constants.hpp>

#include <smpl/console/console.h>

namespace clutter
{

CBSwP::CBSwP() :
CBS()
{
}

CBSwP::CBSwP(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs,
	int scene_id) :
CBS(r, objs, scene_id)
{
}

void CBSwP::growConstraintTree(HighLevelNode* parent)
{
	// expand CT node
	HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
	addConstraints(parent, child[0], child[1]);
	for (int i = 0; i < 2; ++i)
	{
		// priority based pruning
		std::pair<int, int> p_new = { parent->m_conflict[i], parent->m_conflict[1 - i] };
		std::pair<int, int> p_counter = { parent->m_conflict[1 - i], parent->m_conflict[i] };

		// does the parent contain the opposite prioritisation?
		if (parent->m_priorities.find(p_counter) == parent->m_priorities.end())
		{
			child[i]->m_priorities = parent->m_priorities;
			// does the child already contain the "new" prioritisation?
			if (child[i]->m_priorities.find(p_new) == child[i]->m_priorities.end())
			{
				// sanity check
				if (p_new.first == -1 || p_new.second == -1)
				{
					SMPL_ERROR("Improper access into conflict: -1 returned!");
					delete (child[i]);
					continue;
				}
				child[i]->m_priorities.insert(p_new);
			}
		}
		else
		{
			// cyclical prioritisation is not allowed - do not generate child
			delete (child[i]);
			continue;
		}

		if (updateChild(parent, child[i])) {
			parent->m_children.push_back(child[i]);
		}
		else {
			delete (child[i]);
			continue;
		}
	}
	parent->clear();
}

bool CBSwP::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_flowtime = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();
	root->m_priorities.clear();

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
	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		start_time = GetTime();
		if (!m_objs[i]->SatisfyPath(root, &m_paths[i], expands, min_f)) {
			++m_ct_deadends;
			return false;
		}
		m_ll_time += GetTime() - start_time;
		m_ll_expanded += expands;
		m_min_fs[i] = min_f;
		root->m_solution.emplace_back(m_objs[i]->GetID(), *(m_paths[i]));
		root->m_g += m_min_fs[i];
		root->m_flowtime += m_paths[i]->size();
		root->m_makespan = std::max(root->m_makespan, (int)m_paths[i]->size());
	}

	findConflicts(*root);

	root->m_h = 0;
	root->m_h_computed = false;
	root->updateDistanceToGo();

	pushNode(root);
	return true;
}

} // namespace clutter
