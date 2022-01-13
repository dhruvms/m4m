#include <pushplan/cbs.hpp>
#include <pushplan/helpers.hpp>

namespace clutter
{

CBS::CBS() :
m_ct_generated(0), m_ct_expanded(0)
{
	m_robot = nullptr;
	m_objs.clear();
	m_num_agents = 0;
	m_paths.clear();
}

CBS::CBS(Robot* r, std::vector<Agent*> objs) :
m_ct_generated(0), m_ct_expanded(0)
{
	m_robot = r;
	m_objs.insert(m_objs.end(), objs.begin(), objs.end());
	m_num_agents = (int)m_objs.size() + 1;
	m_paths.resize(m_num_agents, nullptr);
}

bool CBS::Solve()
{
	if (!initialiseRoot()) {
		return false;
	}

	m_search_time = 0.0;
	while (!m_OPEN.empty())
	{
		auto next = m_OPEN.top();
		m_OPEN.pop();

		if (done(next)) {
			return m_solved;
		}

		double expand_time = GetTime();

		// expand CT node

		++m_ct_expanded;
		next->m_expand = m_ct_expanded;
		m_search_time += GetTime() - expand_time;
	}
}

void CBS::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();

	// Plan for robot
	if (!m_robot->SatisfyPath(0, root, m_paths[0])) { // (agent id, CT node, path location)
		return false;
	}
	root->m_solution.emplace_back(0, m_paths[0]);
	root->m_g += m_paths[0]->size();
	root->m_makespan = std::max(root->m_makespan, m_paths[0]->size());

	// Plan for objects
	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		if (!m_objs[i]->SatisfyPath(int(i+1), root, m_paths[i+1])) {
			return false;
		}
		root->m_solution.emplace_back(i+1, m_paths[i+1]);
		root->m_g += m_paths[i+1]->size();
		root->m_makespan = std::max(root->m_makespan, m_paths[i+1]->size());
	}

	findConflicts(*root);

	++m_ct_generated;
	root->m_generate = m_ct_generated;
	root->m_OPEN_h = m_OPEN.push(root);
	return true;
}

void CBS::findConflicts(HighLevelNode& node)
{
	if (node->m_parent == nullptr) // root node
	{
		// robot-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			findConflicts(node, m_robot, m_objs[i]);
		}

		// object-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			for (size_t j = i+1; j < m_objs.size(); ++j)
			{
				findConflicts(node, m_objs[i], m_objs[j]);
			}
		}
	}
	else
	{
		copyRelevantConflicts(node);

		if (node.m_replanned == 0) // robot
		{
			for (size_t i = 0; i < m_objs.size(); ++i)
			{
				findConflicts(node, m_robot, m_objs[i]);
			}
		}
		else
		{
			for (size_t i = 0; i < m_objs.size(); ++i)
			{
				if (m_objs[i]->GetID() != node.m_replanned) {
					continue;
				}

				findConflicts(node, m_robot, m_objs[i]);
				for (size_t j = 0; j < m_objs.size(); ++j)
				{
					if (j == i) {
						continue;
					}
					findConflicts(node, m_objs[i], m_objs[j]);
				}
			}
		}
	}
}

void CBS::findConflicts(HighLevelNode& curr, Robot* r, Agent* a)
{
	Trajectory* r_traj = r->GetLastTraj();
	Trajectory* a_traj = a->GetLastTraj();
	int tmin = std::min(r_traj->size(), a_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		a->UpdatePose(a_traj->at(t));
		if (r->CheckCollision(r_traj->at(t), a))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(r->GetID(), a->GetID(), t, r_traj->at(t), a_traj->at(t), true);
			curr.conflicts.push_back(conflict);
		}
	}

	if (r_traj->size() != a_traj->size())
	{
		bool robot_shorter = r_traj->size() < a_traj->size();
		Trajectory* shorter = robot_shorter ? r_traj : a_traj;
		Trajectory* longer = robot_shorter ? a_traj : r_traj;

		a->UpdatePose(a_traj->back());
		for (int t = tmin; t < longer->size(); ++tmin)
		{
			if (robot_shorter)
			{
				a->UpdatePose(longer->at(t));
				if (r->CheckCollision(shorter->back(), a))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(r->GetID(), a->GetID(), t, shorter->back(), longer->at(t), true);
					curr.conflicts.push_back(conflict);
				}
			}
			else
			{
				if (r->CheckCollision(longer->at(t), a))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(r->GetID(), a->GetID(), t, longer->at(t), shorter->back(), true);
					curr.conflicts.push_back(conflict);
				}
			}
		}
	}
}

void CBS::findConflicts(HighLevelNode& curr, Agent* a1, Agent* a2)
{
	Trajectory* a1_traj = a1->GetLastTraj();
	Trajectory* a2_traj = a2->GetLastTraj();
	int tmin = std::min(a1_traj->size(), a2_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		a1->UpdatePose(a1_traj->at(t));
		a2->UpdatePose(a2_traj->at(t));
		if (m_cc->FCLCollision(a1, a2))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(a1->GetID(), a2->GetID(), t, a1_traj->at(t), a2_traj->at(t), false);
			curr.conflicts.push_back(conflict);
		}
	}

	if (a1_traj->size() != a2_traj->size())
	{
		LatticeState terminal;
		bool a1_shorter = a1_traj->size() < a2_traj->size();
		Agent* shorter = a1_shorter ? a1 : a2;
		Agent* longer = a1_shorter ? a2 : a1;
		Trajectory* shorter_traj = a1_shorter ? a1_traj : a2_traj;
		Trajectory* longer_traj = a1_shorter ? a2_traj : a1_traj;

		shorter->UpdatePose(shorter_traj->back());
		for (int t = tmin; t < longer->size(); ++tmin)
		{
			longer->UpdatePose(longer_traj->at(t));
			if (m_cc->FCLCollision(shorter, longer))
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				conflict->InitConflict(shorter->GetID(), longer->GetID(), t, shorter->back(), longer->at(t), false);
				curr.conflicts.push_back(conflict);
			}
		}
	}
}

void CBS::copyRelevantConflicts(HighLevelNode& node)
{
	for (auto& conflict : node.parent->m_conflicts)
	{
		if (conflict->m_a1 == node.m_replanned || conflict->m_a2 == node.m_replanned) {
			continue;
		}
		node.m_conflicts.push_back(conflict);
	}
}

bool CBS::done(HighLevelNode* node)
{
	if (!node->m_conflict.on)
	{
		m_solved = true;
		m_goal = node;
		m_soln_cost = m_goal->m_g;
		return true;
	}

	if (m_search_time > m_time_limit)
	{
		m_solved = false;
		m_goal = nullptr;
		m_soln_cost = -1;
		return true;
	}

	// not done
	return false;
}

} // namespace clutter
