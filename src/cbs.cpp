#include <pushplan/cbs.hpp>
#include <pushplan/helpers.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/constants.hpp>

#include <smpl/console/console.h>

namespace clutter
{

CBS::CBS() :
m_ct_generated(0), m_ct_expanded(0), m_time_limit(3600.0)
{
	m_robot = nullptr;
	m_objs.clear();
	m_num_agents = 0;
	m_paths.clear();
}

CBS::CBS(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs) :
m_ct_generated(0), m_ct_expanded(0), m_time_limit(3600.0)
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
			writeSolution(next);
			return m_solved;
		}

		double expand_time = GetTime();
		++m_ct_expanded;
		next->m_expand = m_ct_expanded;
		// SMPL_ERROR("Expaning (depth, generate, expand) = (%d, %d, %d)! m_replanned = %d", next->m_depth, next->m_generate, next->m_expand, next->m_replanned);
		writeSolution(next);

		// expand CT node
		HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
		addConstraints(next, child[0], child[1]);
		for (int i = 0; i < 2; ++i)
		{
			if (updateChild(next, child[i])) {
				next->m_children.push_back(child[i]);
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

bool CBS::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();

	// Plan for robot
	if (!m_robot->SatisfyPath(root, &m_paths[0])) { // (CT node, path location)
		return false;
	}
	root->m_solution.emplace_back(m_robot->GetID(), m_paths[0]);
	root->m_g += m_paths[0]->size();
	root->m_makespan = std::max(root->m_makespan, (int)m_paths[0]->size());

	// Plan for objects
	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		if (!m_objs[i]->SatisfyPath(root, &m_paths[i+1])) {
			return false;
		}
		root->m_solution.emplace_back(m_objs[i]->GetID(), m_paths[i+1]);
		root->m_g += m_paths[i+1]->size();
		root->m_makespan = std::max(root->m_makespan, (int)m_paths[i+1]->size());
	}

	findConflicts(*root);

	++m_ct_generated;
	root->m_generate = m_ct_generated;
	root->m_OPEN_h = m_OPEN.push(root);

	return true;
}

void CBS::findConflicts(HighLevelNode& node)
{
	if (node.m_parent == nullptr) // root node
	{
		// robot-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i) {
			findConflictsRobot(node, i);
		}

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

		if (node.m_replanned == 0) // robot
		{
			for (size_t i = 0; i < m_objs.size(); ++i) {
				findConflictsRobot(node, i);
			}
		}
		else
		{
			for (size_t i = 0; i < m_objs.size(); ++i)
			{
				if (m_objs[i]->GetID() != node.m_replanned) {
					continue;
				}

				findConflictsRobot(node, i);
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
}

void CBS::findConflictsRobot(HighLevelNode& curr, size_t oid)
{
	auto* r_traj = m_robot->GetLastTraj();
	auto* a_traj = m_objs[oid]->GetLastTraj();
	int tmin = std::min(r_traj->size(), a_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[oid]->UpdatePose(a_traj->at(t));
		// if (m_robot->CheckCollisionWithObject(r_traj->at(t), m_objs[oid].get(), t))
		if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->at(t), r_traj->at(t), t))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_robot->GetID(), m_objs[oid]->GetID(), t, r_traj->at(t), a_traj->at(t), true);
			curr.m_conflicts.push_front(conflict);
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
			std::vector<Object> o;
			o.push_back(m_objs[oid]->GetObject()->back());
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
					conflict->InitConflict(m_robot->GetID(), m_objs[oid]->GetID(), t, shorter->back(), longer->at(t), true);
					curr.m_conflicts.push_front(conflict);
				}
			}
			else
			{
				// if (m_robot->CheckCollision(longer->at(t), t))
				if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->back(), longer->at(t), t, false))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(m_robot->GetID(), m_objs[oid]->GetID(), t, longer->at(t), shorter->back(), true);
					curr.m_conflicts.push_front(conflict);
				}
			}
		}

		if (!robot_shorter)
		{
			// remove object from robot collision space
			std::vector<Object> o;
			o.push_back(m_objs[oid]->GetObject()->back());
			m_robot->ProcessObstacles(o, true);
		}
	}
}

void CBS::findConflictsObjects(HighLevelNode& curr, size_t o1, size_t o2)
{
	auto* a1_traj = m_objs[o1]->GetLastTraj();
	auto* a2_traj = m_objs[o2]->GetLastTraj();
	int tmin = std::min(a1_traj->size(), a2_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[o1]->UpdatePose(a1_traj->at(t));
		m_objs[o2]->UpdatePose(a2_traj->at(t));
		if (m_cc->ObjectObjectCollision(m_objs[o1].get(), m_objs[o2].get()))
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
		Agent* shorter = a1_shorter ? m_objs[o1].get() : m_objs[o2].get();
		Agent* longer = a1_shorter ? m_objs[o2].get() : m_objs[o1].get();
		auto* shorter_traj = a1_shorter ? a1_traj : a2_traj;
		auto* longer_traj = a1_shorter ? a2_traj : a1_traj;

		shorter->UpdatePose(shorter_traj->back());
		for (int t = tmin; t < longer_traj->size(); ++t)
		{
			longer->UpdatePose(longer_traj->at(t));
			if (m_cc->ObjectObjectCollision(shorter, longer))
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				conflict->InitConflict(shorter->GetID(), longer->GetID(), t, shorter_traj->back(), longer_traj->at(t), false);
				curr.m_conflicts.push_back(conflict);
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

		if (!m_robot->SatisfyPath(child, &m_paths[0])) {
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
			child->m_makespan = std::max(child->m_makespan, (int)m_paths[0]->size());
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

			if (!m_objs[i]->SatisfyPath(child, &m_paths[i+1])) {
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
				child->m_makespan = std::max(child->m_makespan, (int)m_paths[0]->size());
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

void CBS::writeSolution(HighLevelNode* node)
{
	int makespan = node->m_makespan;
	for (int tidx = 0; tidx < makespan; tidx += makespan/3)
	{
		std::string filename(__FILE__);
		auto found = filename.find_last_of("/\\");
		filename = filename.substr(0, found + 1) + "../dat/txt/";

		std::stringstream ss;
		ss << std::setw(4) << std::setfill('0') << node->m_expand;
		ss << "_";
		ss << std::setw(4) << std::setfill('0') << node->m_depth;
		ss << "_";
		ss << std::setw(4) << std::setfill('0') << node->m_replanned;
		ss << "_";
		ss << std::setw(4) << std::setfill('0') << tidx;
		// ss << std::setw(4) << std::setfill('0') << node->m_depth;
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
			movable = obs.movable ? "True" : "False";
			DATA << obs.id << ','
					<< obs.Shape() << ','
					<< obs.type << ','
					<< obs.o_x << ','
					<< obs.o_y << ','
					<< obs.o_z << ','
					<< obs.o_roll << ','
					<< obs.o_pitch << ','
					<< obs.o_yaw << ','
					<< obs.x_size << ','
					<< obs.y_size << ','
					<< obs.z_size << ','
					<< obs.mass << ','
					<< obs.mu << ','
					<< movable << '\n';
		}

		// State loc = m_ooi->GetCurrentState()->state;
		// auto agent_obs = m_ooi->GetObject();
		// movable = "False";
		// DATA << 999 << ',' // for visualisation purposes
		// 		<< agent_obs->back().Shape() << ','
		// 		<< agent_obs->back().type << ','
		// 		<< loc.at(0) << ','
		// 		<< loc.at(1) << ','
		// 		<< agent_obs->back().o_z << ','
		// 		<< agent_obs->back().o_roll << ','
		// 		<< agent_obs->back().o_pitch << ','
		// 		<< agent_obs->back().o_yaw << ','
		// 		<< agent_obs->back().x_size << ','
		// 		<< agent_obs->back().y_size << ','
		// 		<< agent_obs->back().z_size << ','
		// 		<< agent_obs->back().mass << ','
		// 		<< agent_obs->back().mu << ','
		// 		<< movable << '\n';

		State loc;
		const std::vector<Object>* agent_obs = nullptr;
		movable = "True";
		for (size_t oidx = 0; oidx < m_objs.size(); ++oidx)
		{
			if (m_paths[oidx+1]->size() <= tidx) {
				loc = m_paths[oidx+1]->back().state;
			}
			else {
				loc = m_paths[oidx+1]->at(tidx).state;
			}

			agent_obs = m_objs[oidx]->GetObject();
			DATA << agent_obs->back().id << ','
				<< agent_obs->back().Shape() << ','
				<< agent_obs->back().type << ','
				<< loc.at(0) << ','
				<< loc.at(1) << ','
				<< agent_obs->back().o_z << ','
				<< agent_obs->back().o_roll << ','
				<< agent_obs->back().o_pitch << ','
				<< agent_obs->back().o_yaw << ','
				<< agent_obs->back().x_size << ','
				<< agent_obs->back().y_size << ','
				<< agent_obs->back().z_size << ','
				<< agent_obs->back().mass << ','
				<< agent_obs->back().mu << ','
				<< movable << '\n';
		}

		if (m_paths[0]->size() <= tidx) {
			agent_obs = m_robot->GetObject(m_paths[0]->back());
		}
		else {
			agent_obs = m_robot->GetObject(m_paths[0]->at(tidx));
		}

		for (const auto& robot_o: *agent_obs)
		{
			DATA << robot_o.id << ','
					<< robot_o.Shape() << ','
					<< robot_o.type << ','
					<< robot_o.o_x << ','
					<< robot_o.o_y << ','
					<< robot_o.o_z << ','
					<< robot_o.o_roll << ','
					<< robot_o.o_pitch << ','
					<< robot_o.o_yaw << ','
					<< robot_o.x_size << ','
					<< robot_o.y_size << ','
					<< robot_o.z_size << ','
					<< robot_o.mass << ','
					<< robot_o.mu << ','
					<< movable << '\n';
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
			agent_obs = m_objs[oidx]->GetObject();
			DATA << agent_obs->back().id << '\n';
			DATA << tidx + 1 << '\n';
			for (int t = 0; t <= tidx; ++t)
			{
				if (m_paths[oidx+1]->size() <= t) {
					DATA << m_paths[oidx+1]->back().state.at(0) << ',' << m_paths[oidx+1]->back().state.at(1) << '\n';
				}
				else {
					DATA << m_paths[oidx+1]->at(t).state.at(0) << ',' << m_paths[oidx+1]->at(t).state.at(1) << '\n';
				}
			}
		}

		DATA << 'R' << '\n';
		DATA << tidx + 1 << '\n';
		for (int t = 0; t <= tidx; ++t)
		{
			if (m_paths[0]->size() <= t) {
				DATA 	<< m_paths[0]->back().state.at(0) << ','
						<< m_paths[0]->back().state.at(1) << ','
						<< m_paths[0]->back().state.at(2) << ','
						<< m_paths[0]->back().state.at(3) << ','
						<< m_paths[0]->back().state.at(4) << ','
						<< m_paths[0]->back().state.at(5) << ','
						<< m_paths[0]->back().state.at(6) << '\n';
			}
			else {
				DATA 	<< m_paths[0]->at(t).state.at(0) << ','
						<< m_paths[0]->at(t).state.at(1) << ','
						<< m_paths[0]->at(t).state.at(2) << ','
						<< m_paths[0]->at(t).state.at(3) << ','
						<< m_paths[0]->at(t).state.at(4) << ','
						<< m_paths[0]->at(t).state.at(5) << ','
						<< m_paths[0]->at(t).state.at(6) << '\n';
			}
		}

		DATA << 'C' << '\n';
		for (auto& constraint : node->m_constraints) {
			DATA 	<< constraint->m_me << ','
					<< constraint->m_other << ','
					<< constraint->m_time << ','
					<< constraint->m_q.state.at(0) << ','
					<< constraint->m_q.state.at(1) << '\n';
		}

		DATA.close();
	}
}

} // namespace clutter
