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

CBS::CBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs) :
m_ct_generated(0), m_ct_expanded(0)
{
	m_robot = r;
	m_objs = objs;
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

		selectConflict(next);
		if (done(next)) {
			return m_solved;
		}

		double expand_time = GetTime();
		++m_ct_expanded;
		next->m_expand = m_ct_expanded;

		// expand CT node
		HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
		addConstraints(curr, child[0], child[1]);
		for (int i = 0; i < 2; ++i)
		{
			if (updateChild(curr, child[i])) {
				parent->m_children.push_back(child[i]);
			}
			else {
				delete (child[i]);
				continue;
			}
		}
		next->clear();

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
	if (!m_robot->SatisfyPath(root, m_paths[0], m_objs.at(0)->GetObject()->back())) { // (CT node, path location)
		return false;
	}
	root->m_solution.emplace_back(m_robot->GetID(), m_paths[0]);
	root->m_g += m_paths[0]->size();
	root->m_makespan = std::max(root->m_makespan, m_paths[0]->size());

	// Plan for objects
	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		if (!m_objs[i]->SatisfyPath(root, m_paths[i+1])) {
			return false;
		}
		root->m_solution.emplace_back(m_objs[i]->GetID(), m_paths[i+1]);
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
			findConflictsRobot(node, i);
		}

		// object-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			for (size_t j = i+1; j < m_objs.size(); ++j)
			{
				findConflictsObjects(node, i, j);
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
				findConflicts(node, i);
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
					findConflicts(node, i, j);
				}
			}
		}
	}
}

void CBS::findConflicts(HighLevelNode& curr, size_t oid)
{
	Trajectory* r_traj = m_robot->GetLastTraj();
	Trajectory* a_traj = m_objs[oid]->GetLastTraj();
	int tmin = std::min(r_traj->size(), a_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[oid]->UpdatePose(a_traj->at(t));
		if (r->CheckCollision(r_traj->at(t), m_objs[oid]))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(r->GetID(), m_objs[oid]->GetID(), t, r_traj->at(t), a_traj->at(t), true);
			curr.m_conflicts.push_front(conflict);
		}
	}

	if (r_traj->size() != a_traj->size())
	{
		bool robot_shorter = r_traj->size() < a_traj->size();
		Trajectory* shorter = robot_shorter ? r_traj : a_traj;
		Trajectory* longer = robot_shorter ? a_traj : r_traj;

		m_objs[oid]->UpdatePose(a_traj->back());
		for (int t = tmin; t < longer->size(); ++tmin)
		{
			if (robot_shorter)
			{
				m_objs[oid]->UpdatePose(longer->at(t));
				if (r->CheckCollision(shorter->back(), m_objs[oid]))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(r->GetID(), m_objs[oid]->GetID(), t, shorter->back(), longer->at(t), true);
					curr.m_conflicts.push_front(conflict);
				}
			}
			else
			{
				if (r->CheckCollision(longer->at(t), m_objs[oid]))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(r->GetID(), m_objs[oid]->GetID(), t, longer->at(t), shorter->back(), true);
					curr.m_conflicts.push_front(conflict);
				}
			}
		}
	}
}

void CBS::findConflicts(HighLevelNode& curr, size_t o1, size_t o2)
{
	Trajectory* a1_traj = m_objs[o1]->GetLastTraj();
	Trajectory* a2_traj = m_objs[o2]->GetLastTraj();
	int tmin = std::min(a1_traj->size(), a2_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[o1]->UpdatePose(a1_traj->at(t));
		m_objs[o2]->UpdatePose(a2_traj->at(t));
		if (m_cc->FCLCollision(m_objs[o1], m_objs[o2]))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_objs[o1]->GetID(), m_objs[o2]->GetID(), t, a1_traj->at(t), a2_traj->at(t), false);
			curr.m_conflicts.push_back(conflict);
		}
	}

	if (a1_traj->size() != a2_traj->size())
	{
		LatticeState terminal;
		bool a1_shorter = a1_traj->size() < a2_traj->size();
		Agent* shorter = a1_shorter ? m_objs[o1] : m_objs[o2];
		Agent* longer = a1_shorter ? m_objs[o2] : m_objs[o1];
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
				curr.m_conflicts.push_back(conflict);
			}
		}
	}
}

void CBS::copyRelevantConflicts(HighLevelNode& node) const
{
	for (auto& conflict : node.parent->m_conflicts)
	{
		if (conflict->m_a1 == node.m_replanned || conflict->m_a2 == node.m_replanned) {
			continue;
		}
		node.m_conflicts.push_back(conflict);
	}
}

void CBS::selectConflict(HighLevelNode* node) const
{
	if (!node->m_conflicts.empty())
	{
		node->m_conflict = node->m_conflicts.front();
		node->m_conflict->m_on = true;
	}
	else
	{
		node->m_conflict = std::make_shared<Conflict>();
		node->m_conflict->m_on = false;
	}
}

void CBS::addConstraints(const HighLevelNode* curr, HighLevelNode* child1, HighLevelNode* child2) const
{
	// children inherit all constraints of parent node
	child1->m_constraints = curr->m_constraints;
	child2->m_constraints = curr->m_constraints;

	// children get one new constraint each
	child1->m_constraints.push_back(curr->m_conflict->m_c1);
	child2->m_constraints.push_back(curr->m_conflict->m_c2);
}

bool CBS::updateChild(HighLevelNode* parent, HighLevelNode* child)
{
	child->m_g = parent->m_g; // for now
	child->m_makespan = parent->m_makespan; // for now
	child->m_solution = parent->m_solution; // for now
	child->m_depth = parent->m_depth + 1;
	child->m_parent = parent;
	child->m_children.clear();

	// agent to be replanned for
	child->m_replanned = child->m_constraints.back()->m_me;

	// replan for agent
	bool recalc_makespan = false;
	if (child->m_replanned == 0)
	{
		child->m_g -= m_paths[0]->size();
		if (child->m_makespan == m_paths[0]->size()) {
			recalc_makespan = true;
		}

		if (!m_robot->SatisfyPath(child, m_paths[0], m_objs.at(0)->GetObject()->back())) {
			return false;
		}
		// update solution in CT node
		for (auto& solution : child->m_solution)
		{
			if (solution.first == m_robot->GetID()) {
				solution.second = m_paths[0];
				break;
			}
		}

		child->m_g += m_paths[0]->size();
		if (recalc_makespan) {
			child->recalcMakespan();
		}
		else {
			child->m_makespan = std::max(child->m_makespan, m_paths[0]->size());
		}

	}
	else
	{
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			if (m_objs[i]->GetID() != child->m_replanned) {
				continue;
			}

			child->m_g -= m_paths[i+1]->size();
			if (child->m_makespan == m_paths[i+1]->size()) {
				recalc_makespan = true;
			}

			if (!m_objs[i]->SatisfyPath(child, m_paths[i+1])) {
				return false;
			}
			// update solution in CT node
			for (auto& solution : child->m_solution)
			{
				if (solution.first == m_objs[i]->GetID()) {
					solution.second = m_paths[i+1];
					break;
				}
			}

			child->m_g += m_paths[i+1]->size();
			if (recalc_makespan) {
				child->recalcMakespan();
			}
			else {
				child->m_makespan = std::max(child->m_makespan, m_paths[0]->size());
			}

			break;
		}
	}

	findConflicts(*child);
	++m_ct_generated;
	child->m_generate = m_ct_generated;
	child->m_OPEN_h = m_OPEN.push(child);

	return true;
}

bool CBS::done(HighLevelNode* node)
{
	if (!node->m_conflict->m_on)
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
