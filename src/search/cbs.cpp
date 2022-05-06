#include <pushplan/agents/robot.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/cbs.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/console/console.h>

#include <ctime>

namespace clutter
{

CBS::CBS() :
m_ct_generated(0), m_ct_deadends(0), m_ct_expanded(0), m_ll_expanded(0), m_time_limit(30.0), m_soln_lb(0), m_wf(1000)
{
	m_robot = nullptr;
	m_objs.clear();
	m_num_agents = 0;
	m_paths.clear();
	m_min_fs.clear();
	m_goal = nullptr;
}

CBS::CBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs,
	int scene_id) :
m_ct_generated(0), m_ct_deadends(0), m_ct_expanded(0), m_ll_expanded(0), m_time_limit(30.0), m_scene_id(scene_id), m_soln_lb(0), m_wf(1000)
{
	m_robot = r;
	m_objs = objs;
	m_num_agents = (int)m_objs.size() + 1;
	m_paths.resize(m_num_agents, nullptr);
	m_min_fs.resize(m_num_agents, 0);

	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		m_obj_id_to_idx[m_objs[i]->GetID()] = i;
		m_obj_idx_to_id[i] = m_objs[i]->GetID();
	}
	m_goal = nullptr;
}

bool CBS::Solve(bool backwards)
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

		// select high level node
		if (m_OPEN.top()->fval() > m_soln_lb)
		{
			int f_thresh = m_wf * m_soln_lb;
			m_soln_lb = std::max(m_soln_lb, m_OPEN.top()->fval());
			int new_f_thresh = m_wf * m_soln_lb;
			for (auto& n : m_OPEN)
			{
				if (n->m_flowtime > f_thresh && n->m_flowtime <= new_f_thresh) {
					n->m_FOCAL_h = m_FOCAL.push(n);
				}
			}
		}
		auto next = m_FOCAL.top();
		m_FOCAL.pop();
		m_OPEN.erase(next->m_OPEN_h);

		selectConflict(next);
		if (done(next)) {
			m_search_time += GetTime() - start_time;
			// writeSolution(next);
			return m_solved;
		}
		// writeSolution(next);

		if (!next->m_h_computed) {
			next->computeH();
			// reinsert into OPEN because lowerbound changed?
			if (next->m_flowtime > m_wf * m_soln_lb)
			{
				next->m_OPEN_h = m_OPEN.push(next);
				continue;
			}
		}

		++m_ct_expanded;
		next->m_expand = m_ct_expanded;

		growConstraintTree(next);

		m_search_time += GetTime() - start_time;
	}

	SMPL_ERROR("CBS high-level OPEN is empty");
	return false;
}

void CBS::growConstraintTree(HighLevelNode* parent)
{
	// expand CT node
	HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
	addConstraints(parent, child[0], child[1]);
	for (int i = 0; i < 2; ++i)
	{
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

bool CBS::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_flowtime = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();

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
	// root->m_flowtime += m_paths[0]->size() - 1;
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
		root->m_solution.emplace_back(m_obj_idx_to_id[i], *(m_paths[i]));
		root->m_g += m_min_fs[i];
		root->m_flowtime += m_paths[i]->size() - 1;
		root->m_makespan = std::max(root->m_makespan, (int)m_paths[i]->size());
	}

	findConflicts(*root);

	root->m_h = 0;
	root->m_h_computed = false;
	root->updateDistanceToGo();

	pushNode(root);
	m_root = root;
	return true;
}

void CBS::pushNode(HighLevelNode* node)
{
	++m_ct_generated;
	node->m_generate = m_ct_generated;
	node->m_OPEN_h = m_OPEN.push(node);
	if (node->m_flowtime <= m_wf * m_soln_lb) {
		node->m_FOCAL_h = m_FOCAL.push(node);
	}
}

void CBS::findConflicts(HighLevelNode& node)
{
	double start_time = GetTime();
	if (node.m_parent == nullptr) // root node
	{
		// robot-object conflicts
		// for (size_t i = 0; i < m_objs.size(); ++i) {
		// 	findConflictsRobot(node, i);
		// }

		// object-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			for (size_t j = i+1; j < m_objs.size(); ++j) {
				findConflictsObjects(node, i, j);
			}
		}
	}
	else
	{
		copyRelevantConflicts(node);

		// if (node.m_replanned == 0) // robot
		// {
		// 	for (size_t i = 0; i < m_objs.size(); ++i) {
		// 		findConflictsRobot(node, i);
		// 	}
		// }
		// else
		{
			for (size_t i = 0; i < m_objs.size(); ++i)
			{
				if (m_obj_idx_to_id[i] != node.m_replanned) {
					continue;
				}

				// findConflictsRobot(node, i);
				for (size_t j = 0; j < m_objs.size(); ++j)
				{
					if (j == i) {
						continue;
					}
					findConflictsObjects(node, i, j);
				}
			}
		}
	}
	m_conflict_time += GetTime() - start_time;
}

void CBS::findConflictsRobot(HighLevelNode& curr, size_t oid)
{
	Trajectory* r_traj = &(curr.m_solution[0].second);
	Trajectory* a_traj = nullptr;
	for (auto& solution: curr.m_solution)
	{
		if (solution.first == m_obj_idx_to_id[oid])
		{
			a_traj = &(solution.second);
			break;
		}
	}

	if (a_traj == nullptr) {
		return;
	}

	int tmin = std::min(r_traj->size(), a_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[oid]->UpdatePose(a_traj->at(t));
		// if (m_robot->CheckCollisionWithObject(r_traj->at(t), m_objs[oid].get(), t))
		if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->at(t), r_traj->at(t), t))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, r_traj->at(t), a_traj->at(t), true);
			curr.m_conflicts.push_back(conflict);
		}
	}

	if (r_traj->size() != a_traj->size())
	{
		bool robot_shorter = r_traj->size() < a_traj->size();
		auto* shorter = robot_shorter ? r_traj : a_traj;
		auto* longer = robot_shorter ? a_traj : r_traj;

		if (!robot_shorter)
		{
			m_objs[oid]->UpdatePose(a_traj->back());
			// add object to robot collision space
			std::vector<Object*> o;
			o.push_back(m_objs[oid]->GetObject());
			m_robot->ProcessObstacles(o);
		}

		for (int t = tmin; t < longer->size(); ++t)
		{
			if (robot_shorter)
			{
				m_objs[oid]->UpdatePose(longer->at(t));
				// if (m_robot->CheckCollisionWithObject(shorter->back(), m_objs[oid].get(), t))
				if (m_cc->RobotObjectCollision(m_objs[oid].get(), longer->at(t), shorter->back(), t))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, shorter->back(), longer->at(t), true);
					curr.m_conflicts.push_back(conflict);
				}
			}
			else
			{
				// if (m_robot->CheckCollision(longer->at(t), t))
				if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->back(), longer->at(t), t, false))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, longer->at(t), shorter->back(), true);
					curr.m_conflicts.push_back(conflict);
				}
			}
		}

		if (!robot_shorter)
		{
			// remove object from robot collision space
			std::vector<Object*> o;
			o.push_back(m_objs[oid]->GetObject());
			m_robot->ProcessObstacles(o, true);
		}
	}
}

void CBS::findConflictsObjects(HighLevelNode& curr, size_t o1, size_t o2)
{
	Trajectory* a1_traj = nullptr;
	Trajectory* a2_traj = nullptr;
	for (auto& solution: curr.m_solution)
	{
		if (solution.first == m_obj_idx_to_id[o1])
		{
			a1_traj = &(solution.second);
			continue;
		}
		else if (solution.first == m_obj_idx_to_id[o2])
		{
			a2_traj = &(solution.second);
			continue;
		}
		else {
			continue;
		}
	}

	if (a1_traj == nullptr || a2_traj == nullptr) {
		return;
	}

	int tmin = std::min(a1_traj->size(), a2_traj->size());
	for (int t = 1; t < tmin; ++t)
	{
		if (a1_traj->at(t).coord == a1_traj->at(0).coord && a2_traj->at(t).coord == a2_traj->at(0).coord) {
			continue;
		}

		m_objs[o1]->UpdatePose(a1_traj->at(t));
		m_objs[o2]->UpdatePose(a2_traj->at(t));
		if (m_cc->ObjectObjectCollision(m_objs[o1]->GetFCLObject(), m_objs[o2]->GetFCLObject()))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_obj_idx_to_id[o1], m_obj_idx_to_id[o2], t, a1_traj->at(t), a2_traj->at(t), false);
			curr.m_conflicts.push_back(conflict);
			// SMPL_INFO("Conflict between objects %d (ID %d) and %d (ID %d)", o1, m_obj_idx_to_id[o1], o2, m_obj_idx_to_id[o2]);
		}
	}

	if (a1_traj->size() != a2_traj->size())
	{
		LatticeState terminal;
		bool a1_shorter = a1_traj->size() < a2_traj->size();
		Agent* shorter = a1_shorter ? m_objs[o1].get() : m_objs[o2].get();
		Agent* longer = a1_shorter ? m_objs[o2].get() : m_objs[o1].get();
		auto* shorter_traj = a1_shorter ? a1_traj : a2_traj;
		auto* longer_traj = a1_shorter ? a2_traj : a1_traj;

		shorter->UpdatePose(shorter_traj->back());
		for (int t = tmin; t < longer_traj->size(); ++t)
		{
			if (shorter_traj->back().coord == shorter_traj->at(0).coord && longer_traj->at(t).coord == longer_traj->at(0).coord) {
				continue;
			}

			longer->UpdatePose(longer_traj->at(t));
			if (m_cc->ObjectObjectCollision(shorter->GetFCLObject(), longer->GetFCLObject()))
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				conflict->InitConflict(shorter->GetID(), longer->GetID(), t, shorter_traj->back(), longer_traj->at(t), false);
				curr.m_conflicts.push_back(conflict);
				// SMPL_INFO("Conflict between objects %d (ID %d) and %d (ID %d)", o1, m_obj_idx_to_id[o1], o2, m_obj_idx_to_id[o2]);
			}
		}
	}
}

void CBS::copyRelevantConflicts(HighLevelNode& node) const
{
	for (auto& conflict : node.m_parent->m_conflicts)
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
	child->m_flowtime = parent->m_flowtime; // for now
	child->m_makespan = parent->m_makespan; // for now
	child->m_solution = parent->m_solution; // for now
	child->m_depth = parent->m_depth + 1;
	child->m_parent = parent;
	child->m_children.clear();

	// agent to be replanned for
	child->m_replanned = child->m_constraints.back()->m_me;
	int expands, min_f;

	// replan for agent
	bool recalc_makespan = true;
	double start_time;
	// if (child->m_replanned == 0)
	// {
	// 	child->m_g -= m_min_fs[0];
	// 	child->m_flowtime -= m_paths[0]->size() - 1;
	// 	if (child->m_makespan == m_paths[0]->size()) {
	// 		recalc_makespan = true;
	// 	}

	// 	start_time = GetTime();
	// 	if (!m_robot->SatisfyPath(child, &m_paths[0], expands, min_f)) {
	// 		++m_ct_deadends;
	// 		return false;
	// 	}
	// 	m_ll_time += GetTime() - start_time;
	// 	m_ll_expanded += expands;

	// 	// update solution in CT node
	// 	for (auto& solution : child->m_solution)
	// 	{
	// 		if (solution.first == m_robot->GetID()) {
	// 			solution.second = *(m_paths[0]);
	// 			break;
	// 		}
	// 	}

	// 	child->m_g += min_f;
	// 	child->m_flowtime += m_paths[0]->size() - 1;
	// 	m_min_fs[0] = min_f;
	// 	if (recalc_makespan) {
	// 		child->recalcMakespan();
	// 	}
	// 	else {
	// 		child->m_makespan = std::max(child->m_makespan, (int)m_paths[0]->size());
	// 	}
	// }
	// else
	{
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			if (m_obj_idx_to_id[i] != child->m_replanned) {
				continue;
			}

			child->m_g -= m_min_fs[i];
			child->m_flowtime -= m_paths[i]->size() - 1;
			if (child->m_makespan == m_paths[i]->size()) {
				recalc_makespan = true;
			}

			m_objs[i]->Init(m_backwards);
			start_time = GetTime();
			if (!m_objs[i]->SatisfyPath(child, &m_paths[i], expands, min_f))
			{
				++m_ct_deadends;
				return false;
			}
			m_ll_time += GetTime() - start_time;
			m_ll_expanded += expands;

			// update solution in CT node
			for (auto& solution : child->m_solution)
			{
				if (solution.first == m_obj_idx_to_id[i]) {
					solution.second = *(m_paths[i]);
					break;
				}
			}

			child->m_g += min_f;
			child->m_flowtime += m_paths[i]->size() - 1;
			m_min_fs[i] = min_f;
			if (recalc_makespan) {
				child->recalcMakespan();
			}
			else {
				child->m_makespan = std::max(child->m_makespan, (int)m_paths[0]->size());
			}

			break;
		}
	}

	findConflicts(*child);

	child->m_h = std::max(0, parent->m_g + (COST_MULT * parent->m_h) - child->m_g);
	child->m_h_computed = false;
	child->updateDistanceToGo();

	pushNode(child);
	return true;
}

bool CBS::done(HighLevelNode* node)
{
	if (!node->m_conflict->m_on)
	{
		m_solved = true;
		m_goal = node;
		m_soln_cost = m_goal->m_flowtime;

		// if (!m_goal->m_priorities.Empty())
		// {
		// 	SMPL_WARN("Solution Priority DAG:");
		// 	auto G = m_goal->m_priorities.GetDAG();
		// 	for (const auto& parent: G) {
		// 		for (const auto& child: parent.second) {
		// 			SMPL_WARN("\t%d -> %d", parent.first, child);
		// 		}
		// 	}
		// }
		// else {
		// 	SMPL_WARN("Solution has empty Priority DAG!");
		// }

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

void CBS::writeSolution(HighLevelNode* node)
{
	int makespan = node->m_makespan;
	for (int tidx = 0; tidx < 1; tidx += 1)
	{
		std::string filename(__FILE__);
		auto found = filename.find_last_of("/\\");
		filename = filename.substr(0, found + 1) + "../../dat/txt/";

		std::stringstream ss;
		// ss << std::setw(4) << std::setfill('0') << node->m_expand;
		// ss << "_";
		// ss << std::setw(4) << std::setfill('0') << node->m_depth;
		// ss << "_";
		// ss << std::setw(4) << std::setfill('0') << node->m_replanned;
		// ss << "_";
		ss << std::setw(6) << std::setfill('0') << m_scene_id;
		ss << "_";
		ss << std::setw(4) << std::setfill('0') << tidx;
		// ss << std::setw(4) << std::setfill('0') << node->m_depth;

		ss << "_";
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		ss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");

		std::string s = ss.str();

		filename += s;
		filename += ".txt";

		std::ofstream DATA;
		DATA.open(filename, std::ofstream::out);

		DATA << 'O' << '\n';
		int o = m_cc->NumObstacles() + m_objs.size() + 3;
		DATA << o << '\n';

		std::string movable;
		auto obstacles = m_cc->GetObstacles();
		for (const auto& obs: *obstacles)
		{
			movable = obs.desc.movable ? "True" : "False";
			DATA << obs.desc.id << ','
					<< obs.Shape() << ','
					<< obs.desc.type << ','
					<< obs.desc.o_x << ','
					<< obs.desc.o_y << ','
					<< obs.desc.o_z << ','
					<< obs.desc.o_roll << ','
					<< obs.desc.o_pitch << ','
					<< obs.desc.o_yaw << ','
					<< obs.desc.x_size << ','
					<< obs.desc.y_size << ','
					<< obs.desc.z_size << ','
					<< obs.desc.mass << ','
					<< obs.desc.mu << ','
					<< movable << '\n';
		}

		// State loc = m_ooi->GetCurrentState()->state;
		// auto agent_obs = m_ooi->GetObject();
		// movable = "False";
		// DATA << 999 << ',' // for visualisation purposes
		// 		<< agent_obs->Shape() << ','
		// 		<< agent_obs->desc.type << ','
		// 		<< loc.at(0) << ','
		// 		<< loc.at(1) << ','
		// 		<< agent_obs->desc.o_z << ','
		// 		<< agent_obs->desc.o_roll << ','
		// 		<< agent_obs->desc.o_pitch << ','
		// 		<< agent_obs->desc.o_yaw << ','
		// 		<< agent_obs->desc.x_size << ','
		// 		<< agent_obs->desc.y_size << ','
		// 		<< agent_obs->desc.z_size << ','
		// 		<< agent_obs->desc.mass << ','
		// 		<< agent_obs->desc.mu << ','
		// 		<< movable << '\n';

		State loc;
		movable = "True";
		for (size_t oidx = 0; oidx < m_objs.size(); ++oidx)
		{
			if (node->m_solution[oidx].second.size() <= tidx) {
				loc = node->m_solution[oidx].second.back().state;
			}
			else {
				loc = node->m_solution[oidx].second.at(tidx).state;
			}

			auto agent_obs = m_objs[oidx]->GetObject();
			DATA << agent_obs->desc.id << ','
				<< agent_obs->Shape() << ','
				<< agent_obs->desc.type << ','
				<< loc.at(0) << ','
				<< loc.at(1) << ','
				<< agent_obs->desc.o_z << ','
				<< agent_obs->desc.o_roll << ','
				<< agent_obs->desc.o_pitch << ','
				<< agent_obs->desc.o_yaw << ','
				<< agent_obs->desc.x_size << ','
				<< agent_obs->desc.y_size << ','
				<< agent_obs->desc.z_size << ','
				<< agent_obs->desc.mass << ','
				<< agent_obs->desc.mu << ','
				<< movable << '\n';
		}

		// const std::vector<Object>* robot_obs = nullptr;
		// if (node->m_solution[0].second.size() <= tidx) {
		// 	robot_obs = m_robot->GetObject(node->m_solution[0].second.back());
		// }
		// else {
		// 	robot_obs = m_robot->GetObject(node->m_solution[0].second.at(tidx));
		// }

		// for (const auto& robot_o: *robot_obs)
		// {
		// 	DATA << robot_o.desc.id << ','
		// 			<< robot_o.Shape() << ','
		// 			<< robot_o.desc.type << ','
		// 			<< robot_o.desc.o_x << ','
		// 			<< robot_o.desc.o_y << ','
		// 			<< robot_o.desc.o_z << ','
		// 			<< robot_o.desc.o_roll << ','
		// 			<< robot_o.desc.o_pitch << ','
		// 			<< robot_o.desc.o_yaw << ','
		// 			<< robot_o.desc.x_size << ','
		// 			<< robot_o.desc.y_size << ','
		// 			<< robot_o.desc.z_size << ','
		// 			<< robot_o.desc.mass << ','
		// 			<< robot_o.desc.mu << ','
		// 			<< movable << '\n';
		// }

		// write goal states
		DATA << 'G' << '\n';
		DATA << m_objs.size() << '\n';
		for (size_t oidx = 0; oidx < m_objs.size(); ++oidx)
		{
			auto g = m_objs[oidx]->Goal();
			State gs;
			DiscToCont(g, gs);
			DATA << m_obj_idx_to_id[oidx] << ',' << gs.at(0) << ',' << gs.at(1) << '\n';
		}

		// write solution trajs
		DATA << 'T' << '\n';
		o = m_objs.size();
		DATA << o << '\n';

		// auto move = m_ooi->GetMoveTraj();
		// DATA << 999 << '\n';
		// DATA << move->size() << '\n';
		// for (const auto& s: *move) {
		// 	DATA << s.state.at(0) << ',' << s.state.at(1) << '\n';
		// }

		for (size_t oidx = 0; oidx < m_objs.size(); ++oidx)
		{
			auto agent_obs = m_objs[oidx]->GetObject();
			DATA << agent_obs->desc.id << '\n';
			DATA << makespan + 1 << '\n';
			for (int t = 0; t <= makespan; ++t)
			{
				if (node->m_solution[oidx].second.size() <= t) {
					DATA << node->m_solution[oidx].second.back().state.at(0) << ',' << node->m_solution[oidx].second.back().state.at(1) << '\n';
				}
				else {
					DATA << node->m_solution[oidx].second.at(t).state.at(0) << ',' << node->m_solution[oidx].second.at(t).state.at(1) << '\n';
				}
			}
		}

		// DATA << 'R' << '\n';
		// DATA << tidx + 1 << '\n';
		// for (int t = 0; t <= tidx; ++t)
		// {
		// 	if (node->m_solution[0].second.size() <= t) {
		// 		DATA 	<< node->m_solution[0].second.back().state.at(0) << ','
		// 				<< node->m_solution[0].second.back().state.at(1) << ','
		// 				<< node->m_solution[0].second.back().state.at(2) << ','
		// 				<< node->m_solution[0].second.back().state.at(3) << ','
		// 				<< node->m_solution[0].second.back().state.at(4) << ','
		// 				<< node->m_solution[0].second.back().state.at(5) << ','
		// 				<< node->m_solution[0].second.back().state.at(6) << '\n';
		// 	}
		// 	else {
		// 		DATA 	<< node->m_solution[0].second.at(t).state.at(0) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(1) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(2) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(3) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(4) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(5) << ','
		// 				<< node->m_solution[0].second.at(t).state.at(6) << '\n';
		// 	}
		// }

		DATA << 'C' << '\n';
		for (auto& constraint : node->m_constraints) {
			DATA 	<< constraint->m_me << ','
					<< constraint->m_other << ','
					<< constraint->m_time << ','
					<< constraint->m_q.state.at(0) << ','
					<< constraint->m_q.state.at(1) << '\n';
		}

		struct coord_compare
		{
			bool operator()(const Coord& u, const Coord& v) const
			{
				return std::tie(u.at(0), u.at(1)) < std::tie(v.at(0), v.at(1));
			}
		};
		std::set<Coord, coord_compare> ngr;
		auto ngr_voxels = m_robot->TrajVoxels();
		for (auto itr_list = ngr_voxels->begin(); itr_list != ngr_voxels->end(); ++itr_list)
		{
			for (auto itr = itr_list->begin(); itr != itr_list->end(); ++itr)
			{
				State s = { itr->x(), itr->y() };
				Coord c;
				ContToDisc(s, c);
				ngr.insert(c);
			}
		}
		DATA << "NGR" << '\n';
		DATA << ngr.size() << '\n';
		for (const auto& p: ngr) {
			DATA 	<< p.at(0) << ','
					<< p.at(1) << '\n';
		}

		auto push_debug_data = m_robot->PushDebugData();
		DATA << "PUSHES" << '\n';
		DATA << push_debug_data.size() << '\n';
		for (const auto& push: push_debug_data) {
			DATA 	<< push.at(0) << ','
					<< push.at(1) << ','
					<< push.at(2) << ','
					<< push.at(3) << ','
					<< push.at(4) << '\n';
		}

		DATA.close();
		m_robot->ClearPushDebugData();
	}
}

void CBS::SaveStats()
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/?.csv";

	std::string algoname;
	switch (ALGO)
	{
		case MAPFAlgo::ECBS:
		{
			algoname = "ECBS";
			break;
		}
		case MAPFAlgo::CBSWP:
		{
			algoname = "CBSwP";
			break;
		}
		case MAPFAlgo::PBS:
		{
			algoname = "PBS";
			break;
		}
		case MAPFAlgo::VCBS:
		default:
		{
			algoname = "CBS";
			break;
		}
	}
	found = filename.find_last_of("?");
	filename.insert(found, algoname);
	found = filename.find_last_of("?");
	filename.erase(found, 1);

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "Solved?,SolveTime,SolutionCost,"
				<< "HLGenerated,HLDeadends,HLExpanded,"
				<< "LLExpanded,LLTime,ConflictsTime\n";
	}

	STATS << m_scene_id << ','
			<< (int)m_solved << ',' << m_search_time << ',' << m_soln_cost << ','
			<< m_ct_generated << ',' << m_ct_deadends << ',' << m_ct_expanded << ','
			<< m_ll_expanded << ',' << m_ll_time << ',' << m_conflict_time << '\n';
	STATS.close();
}

bool CBS::UpdateStats(std::map<std::string, double>& stats)
{
	if (stats.empty())
	{
		stats["calls"] = 1;
		stats["solved"] = (int)m_solved;
		stats["search_time"] = m_search_time;
		stats["ct_nodes"] = m_ct_generated;
		stats["ct_deadends"] = m_ct_deadends;
		stats["ct_expanded"] = m_ct_expanded;
		stats["ll_time"] = m_ll_time;
		stats["conflict_time"] = m_conflict_time;

		return true;
	}

	stats["calls"] += 1;
	stats["solved"] += (int)m_solved;
	stats["search_time"] += m_search_time;
	stats["ct_nodes"] += m_ct_generated;
	stats["ct_deadends"] += m_ct_deadends;
	stats["ct_expanded"] += m_ct_expanded;
	stats["ll_time"] += m_ll_time;
	stats["conflict_time"] += m_conflict_time;

	return true;
}

} // namespace clutter
