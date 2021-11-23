#include <pushplan/planner.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/constants.hpp>
#include <pushplan/geometry.hpp>

#include <smpl/console/console.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <cstdlib>

#include <gperftools/profiler.h>

namespace clutter
{

bool Planner::Init(const std::string& scene_file, int scene_id, bool ycb)
{
	m_scene_file = scene_file;
	m_scene_id = scene_id;

	std::string tables;
	int num_immov, num_mov;

	m_ph.getParam("object_filename", tables);
	m_ph.getParam("objects/num_immov", num_immov);
	m_ph.getParam("objects/num_mov", num_mov);
	m_sim = std::make_shared<BulletSim>(
				tables, ycb,
				m_scene_id, std::string(),
				num_immov, num_mov);

	m_robot = std::make_unique<Robot>();

	m_stats["whca_attempts"] = 0;
	m_stats["robot_plan_time"] = 0.0;
	m_stats["mapf_time"] = 0.0;
	m_stats["first_order_interactions"] = 0;
	m_plan_time = 0.0;

	m_ph.getParam("goal/plan_budget", m_plan_budget);
	m_ph.getParam("goal/sim_budget", m_sim_budget);
	m_ph.getParam("goal/total_budget", m_total_budget);

	setupGlobals();

	m_agents.clear();

	std::vector<Object> all_obstacles, pruned_obstacles;
	if (m_scene_id < 0)	{ // init agents from simulator
		init_agents(ycb, all_obstacles);
	}
	else {
		parse_scene(all_obstacles); // TODO: relax ycb = false assumption
	}

	pruned_obstacles = all_obstacles;
	// only keep the base of the fridge shelf
	if (FRIDGE)
	{
		auto fbegin = pruned_obstacles.begin();
		auto fend = pruned_obstacles.end();
		for (auto itr = pruned_obstacles.begin(); itr != pruned_obstacles.end(); ++itr)
		{
			if (itr->id == 2) {
				fbegin = itr;
			}
			if (itr->id > 5) {
				fend = itr;
				break;
			}
		}
		pruned_obstacles.erase(fbegin, fend);
	}

	m_cc = std::make_shared<CollisionChecker>(this, pruned_obstacles);
	pruned_obstacles.clear();

	// Get OOI goal
	m_ooi_gf = m_cc->GetRandomStateOutside(&m_ooi.GetObject()->back());
	ContToDisc(m_ooi_gf, m_ooi_g);

	m_ooi.SetCC(m_cc);
	m_robot->SetCC(m_cc);
	for (auto& a: m_agents) {
		a.SetCC(m_cc);
	}

	m_ooi.Setup();
	if (!m_robot->Setup())
	{
		SMPL_ERROR("Robot setup failed!");
		return false;
	}
	for (auto& a: m_agents) {
		a.Setup();
	}

	if (!m_robot->ProcessObstacles(all_obstacles))
	{
		SMPL_ERROR("Robot collision space setup failed!");
		return false;
	}
	all_obstacles.clear();

	int t = 0, grasp_tries;
	m_ph.getParam("robot/grasp_tries", grasp_tries);
	for (; t < grasp_tries; ++t)
	{
		if (m_robot->ComputeGrasps(m_goal, m_ooi.GetObject()->back())) {
			break;
		}
	}
	if (t == grasp_tries)
	{
		SMPL_ERROR("Robot failed to compute grasp states!");
		return false;
	}

	m_simulate = m_nh.advertiseService("run_sim", &Planner::runSim, this);
	m_animate = m_nh.advertiseService("anim_soln", &Planner::animateSolution, this);
	m_rearrange = m_nh.advertiseService("rearrange", &Planner::rearrange, this);

	setupSim();

	m_robot->SetSim(m_sim);

	return true;
}

bool Planner::Alive()
{
	// double plan_time = m_plan_time + m_robot->PlannerTime();
	// if (plan_time > m_plan_budget) {
	// 	return false;
	// }

	// if (m_robot->SimTime() > m_sim_budget) {
	// 	return false;
	// }

	double total_time = m_plan_time + m_robot->PlannerTime() + m_robot->SimTime();
	if (total_time > m_total_budget) {
		return false;
	}

	if (m_robot->BadAttach()) {
		return false;
	}

	return true;
}

bool Planner::Plan()
{
	++m_stats["whca_attempts"];
	while (!setupProblem()) {
		continue;
	}

	double start_time = GetTime(), time_taken;
	if (whcastar())
	{
		m_exec_interm.clear();

		auto robot_traj = m_robot->GetMoveTraj();
		m_exec_interm.insert(m_exec_interm.begin(), robot_traj->begin(), robot_traj->end());
		SMPL_INFO("Exec trajj size = %d", m_exec_interm.size());
		m_robot->ProfileTraj(m_exec_interm);
	}
	else
	{
		m_exec_interm.clear();

		time_taken = GetTime() - start_time;
		SMPL_WARN("Need to re-run WHCA*! Planning took %f seconds.", time_taken);
		m_stats["mapf_time"] += time_taken;
		m_plan_time += time_taken;
		return false;
	}
	time_taken = GetTime() - start_time;
	m_stats["mapf_time"] += time_taken;
	m_plan_time += time_taken;
	SMPL_INFO("Planning took %f seconds. (%f runs so far)", time_taken, m_stats["whca_attempts"]);

	m_cc->PrintConflicts();
	m_stats["first_order_interactions"] = m_cc->NumConflicts();
	if (SAVE) {
		savePlanData();
	}

	return true;
}

bool Planner::PlanExtract()
{
	// Add rearranged objects as obstacles for extraction planning
	std::vector<Object> extract_obs;
	for (const auto& a: m_agents)
	{
		extract_obs.push_back(a.GetObject()->back());
		// auto search = m_agent_map.find(o.id);
		// if (search != m_agent_map.end()) {
		// }
	}
	if (!m_robot->Plan(m_ooi.GetObject()->back(), extract_obs)) {
		moveit_msgs::RobotTrajectory to_exec;
		m_robot->ConvertTraj(m_exec_interm, to_exec);
		m_exec = to_exec.joint_trajectory;
		return false;
	}
	else {
		m_robot->GetExecTraj(m_exec);
	}
	SMPL_INFO("Exec traj size = %d", m_exec.points.size());

	comms::ObjectsPoses rearranged = m_rearranged;
	bool success = true;
	double start_time = GetTime();
	if (!m_sim->ExecTraj(m_exec, rearranged, m_robot->GraspAt(), m_ooi.GetID()))
	{
		SMPL_ERROR("Failed to exec traj!");
		success = false;
	}

	m_sim->RemoveConstraint();

	return success;
}

bool Planner::Rearrange()
{
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	if (!rearrange(req, resp)) {
		SMPL_WARN("There were no conflicts to rearrange!");
		return false;
	}

	for (auto& a: m_agents) {
		a.Setup(); // updates original object
	}
	return true;
}

std::uint32_t Planner::RunSim()
{
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;

	if (!runSim(req, resp)) {
		SMPL_ERROR("Simulation failed!");
	}
	return m_violation;
}

bool Planner::TryExtract()
{
	if (m_exec_interm.empty() || !setupSim()) {
		return false;
	}

	moveit_msgs::RobotTrajectory to_exec;
	m_robot->ConvertTraj(m_exec_interm, to_exec);
	comms::ObjectsPoses rearranged = m_rearranged;
	bool success = true;
	double start_time = GetTime();
	if (!m_sim->ExecTraj(to_exec.joint_trajectory, rearranged, m_robot->GraspAt(), m_ooi.GetID()))
	{
		SMPL_ERROR("Failed to exec traj!");
		success = false;
	}

	m_sim->RemoveConstraint();

	return success;
}

void Planner::AnimateSolution()
{
	std::vector<Object> final_objects;
	for (auto& a: m_agents) {
		a.ResetObject();
		final_objects.push_back(a.GetObject()->back());
	}
	m_robot->ProcessObstacles(final_objects);

	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	animateSolution(req, resp);
}

bool Planner::whcastar()
{
	// ProfilerStart("/home/dhruv/test3.out");

	double start_time = GetTime(), total_time = 0.0, iter_time = 0.0;

	int iter = 0;
	if (SAVE) {
		writePlanState(iter);
	}

	// // Add rearranged objects as obstacles for extraction planning
	// std::vector<Object> extract_obs;
	// for (const auto& o: m_rearranged.poses)
	// {
	// 	auto search = m_agent_map.find(o.id);
	// 	if (search != m_agent_map.end()) {
	// 		extract_obs.push_back(m_agents.at(m_agent_map[o.id]).GetObject()->back());
	// 	}
	// }
	if (!m_robot->Plan(m_ooi.GetObject()->back())) {
		return false;
	}
	m_stats["robot_plan_time"] = m_robot->TrajPlanTime();
	// m_robot->AnimateSolution();

	reinit();
	while (!m_robot->AtGrasp())
	{
		if (!m_ooi.Search(1)) {
			return false;
		}
		int robin = 2;
		for (const auto& p: m_priorities)
		{
			if (!m_agents.at(p).Search(robin)) {
				return false;
			}
			++robin;
		}

		step_agents(); // take 1 step by default
		reinit();

		++iter;
		if (SAVE) {
			writePlanState(iter);
		}

		iter_time = GetTime() - start_time;
		if (iter_time > WHCA_PLANNING_TIME) {
			SMPL_WARN("WHCA* Phase 1 Timeout!");
			return false;
		}
	}
	double phase_time = GetTime() - start_time;
	total_time += phase_time;
	SMPL_INFO("WHCA* Phase 1 planning took %f seconds.", phase_time);

	// ProfilerStop();

	m_phase = 1;

	start_time = GetTime();
	reinit();
	while (!m_robot->AtEnd())
	{
		if (!m_ooi.Search(1)) {
			return false;
		}
		int robin = 2;
		for (const auto& p: m_priorities)
		{
			if (!m_agents.at(p).Search(robin)) {
				return false;
			}
			++robin;
		}

		step_agents();
		reinit();

		++iter;
		if (SAVE) {
			writePlanState(iter);
		}

		iter_time = GetTime() - start_time;
		if (iter_time > WHCA_PLANNING_TIME) {
			SMPL_WARN("WHCA* Phase 2 Timeout!");
			return false;
		}
	}
	phase_time = GetTime() - start_time;
	total_time += phase_time;
	SMPL_INFO("WHCA* Phase 2 planning took %f seconds. Total time = %f seconds.", phase_time, total_time);

	return true;
}

bool Planner::rearrange(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	for (auto& a: m_agents) {
		a.ResetObject();
	}

	auto robot_conflicts = m_cc->GetConflictsOf(100); // get first order conflicts
	if (robot_conflicts.empty()) {
		return false;
	}

	comms::ObjectsPoses rearranged = m_rearranged;
	std::vector<int> ids_attempted;

	bool push_found = false;
	while (!push_found && !robot_conflicts.empty())
	{
		auto i = robot_conflicts.begin();
		for (auto iter = robot_conflicts.begin(); iter != robot_conflicts.end(); ++iter)
		{
			if (iter == i) {
				continue;
			}

			if (iter->second < i->second) {
				i = iter;
			}
		}

		int oid = i->first.first;
		int oid_t = i->second;
		SMPL_INFO("Rearranging object %d", oid);
		// m_agents.at(m_agent_map[oid]).ResetObject();

		auto oid_conflicts = m_cc->GetConflictsOf(oid);
		std::vector<Object> new_obstacles;
		for (const auto& a: m_agents)
		{
			if (a.GetObject()->back().id == oid) {
				continue; // selected object cannot be obstacle
			}

			bool oid_child = false;
			for (const auto& c: oid_conflicts)
			{
				if (c.first.first == a.GetObject()->back().id) {
					oid_child = true;
					break;
				}
			}
			if (oid_child) {
				continue; // children of selected object may not be obstacle
			}

			new_obstacles.push_back(a.GetObject()->back());
			SMPL_INFO("Adding object %d as obstacle", a.GetObject()->back().id);
		}

		for (const auto& c: robot_conflicts)
		{
			if (c.first.first != oid && c.second < oid_t)
			{
				// first order conflicts earlier than selected object
				// must be obstacles
				new_obstacles.push_back(m_agents.at(m_agent_map[c.first.first]).GetObject()->back());
				SMPL_INFO("Adding object %d as obstacle", c.first.first);
			}
		}

		for (const auto& id: ids_attempted)
		{
			if (id == oid) {
				continue;
			}

			// first order conflicts already selected (for which no push was found)
			// must be obstacles
			new_obstacles.push_back(m_agents.at(m_agent_map[id]).GetObject()->back());
			SMPL_INFO("Adding rearranged object %d as obstacle", id);
		}
		// add new obstacles
		m_robot->ProcessObstacles(new_obstacles);

		// get push location
		std::vector<double> push;
		m_agents.at(m_agent_map[oid]).GetSE2Push(push);
		SMPL_INFO("Object %d push is (%f, %f, %f)", oid, push[0], push[1], push[2]);

		// set push location goal for robot
		m_robot->SetPushGoal(push);

		// plan to push location
		// m_robot->PlanPush creates the planner internally, because it might
		// change KDL chain during the process
		comms::ObjectsPoses result;

		if (m_robot->PlanPush(oid, m_agents.at(m_agent_map[oid]).GetMoveTraj(), m_agents.at(m_agent_map[oid]).GetObject()->back(), rearranged, result))
		{
			push_found = true;
			m_rearrangements.push_back(m_robot->GetLastPlan());

			// update positions of moved objects
			updateAgentPositions(result, rearranged);
		}
		if (SAVE) {
			m_robot->SavePushData(m_scene_id);
		}

		// remove new obstacles
		m_robot->ProcessObstacles(new_obstacles, true);

		robot_conflicts.erase(i);

		auto it = std::find(begin(ids_attempted), end(ids_attempted), oid);
		if (it == end(ids_attempted)) {
			ids_attempted.push_back(oid);
		}
	}
	m_rearranged = rearranged;

	return true;
}

bool Planner::animateSolution(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	m_robot->AnimateSolution();
	return true;
}

bool Planner::setupSim()
{
	if (!m_sim->SetRobotState(m_robot->GetStartState()->joint_state))
	{
		ROS_ERROR("Failed to set start state!");
		return false;
	}

	int planning_arm = armId();
	if (!m_sim->ResetArm(1 - planning_arm))
	{
		ROS_ERROR("Failed to reset other arm!");
		return false;
	}

	int removed;
	if (!m_sim->CheckScene(planning_arm, removed))
	{
		ROS_ERROR("Failed to check for start state collisions with objects!");
		return false;
	}

	if (!m_sim->ResetScene()) {
		ROS_ERROR("Failed to reset scene objects!");
		return false;
	}

	if (!m_sim->SetColours(m_ooi.GetID()))
	{
		ROS_ERROR("Failed to set object colours in scene!");
		return false;
	}
}

bool Planner::runSim(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	setupSim();

	m_violation = 0x00000000;

	comms::ObjectsPoses dummy;
	for (const auto& traj: m_rearrangements)
	{
		if (traj.points.empty()) {
			continue;
		}

		if (!m_sim->ExecTraj(traj, dummy))
		{
			SMPL_ERROR("Failed to exec rearrangement!");
			m_violation |= 0x00000001;
		}
	}
	// if any rearrangement traj execuction failed, m_violation == 1

	if (!m_sim->ExecTraj(m_exec, dummy, m_robot->GraspAt(), m_ooi.GetID()))
	{
		SMPL_ERROR("Failed to exec traj!");
		m_violation |= 0x00000004;
	}
	// if all rearrangements succeeded, but extraction failed, m_violation == 4
	// if any rearrangement traj execuction failed, and extraction failed, m_violation == 5
	// if all executions succeeded, m_violation == 0

	m_sim->RemoveConstraint();

	return m_violation == 0;
}

const std::vector<Object>* Planner::GetObject(const LatticeState& s, int priority)
{
	if (priority == 0) {
		return m_robot->GetObject(s);
	}
	else if (priority == 1) {
		return m_ooi.GetObject(s);
	}
	else {
		return m_agents.at(m_priorities.at(priority-2)).GetObject(s);
	}
}

void Planner::reinit()
{
	// reset searches
	m_ooi.reset(m_phase);
	for (auto& a: m_agents) {
		a.reset(m_phase);
	}

	// Set agent starts - always the current state
	m_ooi.SetStartState(*(m_ooi.GetCurrentState()));
	for (auto& a: m_agents) {
		a.SetStartState(*(a.GetCurrentState()));
	}

	// Set agent goals
	if (m_phase == 0) {
		m_ooi.SetGoalState(m_ooi.GetCurrentState()->coord);
	}
	else if (m_phase == 1) {
		m_ooi.SetGoalState(m_robot->GetEECoord()); // EE position
	}

	for (auto& a: m_agents) {
		a.SetGoalState(a.GetCurrentState()->coord);
	}

	// Set agent priorities
	prioritize();
}

void Planner::prioritize()
{
	m_priorities.clear();
	m_priorities.resize(m_agents.size());
	std::iota(m_priorities.begin(), m_priorities.end(), 0);

	// robot_objs is a std::vector<Object>*
	auto robot_objs = m_robot->GetObject(*(m_robot->GetCurrentState()));

	std::vector<double> dists(m_agents.size(), std::numeric_limits<double>::max());
	State robot_pf(2, 0.0), agent_pf;
	for (size_t i = 0; i < m_agents.size(); ++i)
	{
		DiscToCont(m_agents.at(i).GetCurrentState()->coord, agent_pf);
		for (const auto& o: *robot_objs)
		{
			robot_pf.at(0) = o.o_x;
			robot_pf.at(1) = o.o_y;
			dists.at(i) = std::min(dists.at(i), EuclideanDist(robot_pf, agent_pf));
		}
	}
	std::stable_sort(m_priorities.begin(), m_priorities.end(),
		[&dists](size_t i1, size_t i2) { return dists[i1] < dists[i2]; });
}

void Planner::step_agents(int k)
{
	// TODO: make sure we can handle k > 1
	m_ooi.Step(k);
	m_robot->Step(k);
	for (auto& a: m_agents) {
		a.Step(k);
	}
	m_t += k;
}

void Planner::updateAgentPositions(
	const comms::ObjectsPoses& result,
	comms::ObjectsPoses& rearranged)
{
	for (const auto& o: result.poses)
	{
		auto search = m_agent_map.find(o.id);
		if (search != m_agent_map.end()) {
			// auto orig_obj = m_agents.at(m_agent_map[o.id]).GetObject()->back();
			// if (EuclideanDist(o.xyz, {orig_obj.o_x, orig_obj.o_y, orig_obj.o_z}) < RES)
			// {
			// 	SMPL_DEBUG("Object %d did not move > %f cm.", o.id, RES);
			// 	continue;
			// }

			// SMPL_DEBUG("Object %d did moved > %f cm.", o.id, RES);
			m_agents.at(m_agent_map[o.id]).SetObjectPose(o.xyz, o.rpy);
			bool exist = false;
			for (auto& p: rearranged.poses)
			{
				if (p.id == o.id)
				{
					p = o;
					exist = true;
					break;
				}
			}
			if (!exist) {
				rearranged.poses.push_back(o);
			}
		}
	}
}

bool Planner::setupProblem()
{
	int result = cleanupLogs();
	if (result == -1) {
		SMPL_ERROR("system command errored!");
	}

	m_t = 0;
	m_phase = 0;
	m_cc->ClearConflicts();

	// Set agent current positions and time
	m_ooi.Init();

	m_robot->Init();
	if (m_exec_interm.empty() && !m_robot->RandomiseStart()) {
		return false;
	}

	for (auto& a: m_agents) {
		a.Init();
	}

	return true;
}

int Planner::cleanupLogs()
{
	std::string files(__FILE__), command;
	auto found = files.find_last_of("/\\");
	files = files.substr(0, found + 1) + "../dat/txt/*.txt";

	command = "rm " + files;
	// SMPL_WARN("Execute command: %s", command.c_str());
	return system(command.c_str());
}

void Planner::init_agents(
	bool ycb, std::vector<Object>& obstacles)
{
	auto mov_objs = m_sim->GetMovableObjs();
	auto mov_obj_ids = m_sim->GetMovableObjIDs();
	auto immov_objs = m_sim->GetImmovableObjs();
	auto immov_obj_ids = m_sim->GetImmovableObjIDs();

	m_num_objs = mov_objs->size() + immov_objs->size();
	m_agents.clear();
	m_agent_map.clear();
	obstacles.clear();

	int tables = FRIDGE ? 5 : 1;
	bool ooi_set = false;
	Object o;
	for (size_t i = 0; i < immov_objs->size(); ++i)
	{
		o.id = immov_obj_ids->at(i).first;
		o.type = i < tables ? -1 : 0; // table or immovable
		o.o_x = immov_objs->at(i).at(0);
		o.o_y = immov_objs->at(i).at(1);
		o.o_z = immov_objs->at(i).at(2);
		o.o_roll = immov_objs->at(i).at(3);
		o.o_pitch = immov_objs->at(i).at(4);
		o.o_yaw = immov_objs->at(i).at(5);

		if (ycb && i >= tables)
		{
			auto itr = YCB_OBJECT_DIMS.find(immov_obj_ids->at(i).second);
			if (itr != YCB_OBJECT_DIMS.end())
			{
				o.x_size = itr->second.at(0);
				o.y_size = itr->second.at(1);
				o.z_size = itr->second.at(2);
				o.o_yaw += itr->second.at(3);
			}
		}
		else
		{
			o.x_size = immov_objs->at(i).at(6);
			o.y_size = immov_objs->at(i).at(7);
			o.z_size = immov_objs->at(i).at(8);
		}
		o.movable = false;
		o.shape = immov_obj_ids->at(i).second;
		o.mass = i == 0 ? 0 : -1;
		o.locked = i == 0 ? true : false;
		o.mu = -1;
		o.ycb = i >= tables ? ycb : false;

		if (!ooi_set && i >= tables)
		{
			m_ooi.SetObject(o);
			m_goal.clear();

			double xdisp = std::cos(o.o_yaw) * 0.1;
			double ydisp = std::sin(o.o_yaw) * 0.1;
			m_goal = {o.o_x - xdisp, o.o_y - ydisp, obstacles.at(0).o_z + obstacles.at(0).z_size + 0.05, 0.0, 0.0, -o.o_yaw};

			ooi_set = true;
			continue;
		}

		obstacles.push_back(o);
	}

	for (size_t i = 0; i < mov_objs->size(); ++i)
	{
		o.id = mov_obj_ids->at(i).first;
		o.type = 1; // movable
		o.o_x = mov_objs->at(i).at(0);
		o.o_y = mov_objs->at(i).at(1);
		o.o_z = mov_objs->at(i).at(2);
		o.o_roll = mov_objs->at(i).at(3);
		o.o_pitch = mov_objs->at(i).at(4);
		o.o_yaw = mov_objs->at(i).at(5);

		if (ycb)
		{
			auto itr = YCB_OBJECT_DIMS.find(mov_obj_ids->at(i).second);
			if (itr != YCB_OBJECT_DIMS.end())
			{
				o.x_size = itr->second.at(0);
				o.y_size = itr->second.at(1);
				o.z_size = itr->second.at(2);
				o.o_yaw += itr->second.at(3);
			}
		}
		else
		{
			o.x_size = mov_objs->at(i).at(6);
			o.y_size = mov_objs->at(i).at(7);
			o.z_size = mov_objs->at(i).at(8);
		}
		o.movable = true;
		o.shape = mov_obj_ids->at(i).second;
		o.mass = i == 0 ? 0 : -1;
		o.locked = false;
		o.mu = -1;
		o.ycb = ycb;

		Agent r(o);
		m_agents.push_back(std::move(r));
		m_agent_map[o.id] = m_agents.size() - 1;
	}
}

void Planner::parse_scene(std::vector<Object>& obstacles)
{
	std::ifstream SCENE;
	SCENE.open(m_scene_file);

	if (SCENE.is_open())
	{
		std::string line;
		while (!SCENE.eof())
		{
			getline(SCENE, line);
			if (line.compare("O") == 0)
			{
				getline(SCENE, line);
				m_num_objs = std::stoi(line);
				m_agents.clear();
				m_agent_map.clear();
				obstacles.clear();

				for (int i = 0; i < m_num_objs; ++i)
				{
					Object o;

					getline(SCENE, line);
					std::stringstream ss(line);
					std::string split;

					int count = 0;
					while (ss.good())
					{
						getline(ss, split, ',');
						switch (count) {
							case 0: o.id = std::stoi(split); break;
							case 1: o.shape = std::stoi(split); break;
							case 2: o.type = std::stoi(split); break;
							case 3: o.o_x = std::stof(split); break;
							case 4: o.o_y = std::stof(split); break;
							case 5: o.o_z = std::stof(split); break;
							case 6: o.o_roll = std::stof(split); break;
							case 7: o.o_pitch = std::stof(split); break;
							case 8: o.o_yaw = std::stof(split); break;
							case 9: o.x_size = std::stof(split); break;
							case 10: o.y_size = std::stof(split); break;
							case 11: o.z_size = std::stof(split); break;
							case 12: {
								o.mass = std::stof(split);
								o.locked = o.mass == 0;
								break;
							}
							case 13: o.mu = std::stof(split); break;
							case 14: o.movable = (split.compare("True") == 0); break;
						}
						o.ycb = false;
						count++;
					}
					if (o.movable) {
						Agent r(o);
						m_agents.push_back(std::move(r));
						m_agent_map[o.id] = m_agents.size() - 1;
					}
					else {
						// o.x_size += RES;
						// o.y_size += RES;
						obstacles.push_back(o);
					}
				}
			}

			else if (line.compare("ooi") == 0)
			{
				getline(SCENE, line);
				m_ooi_idx = std::stoi(line); // object of interest ID

				for (auto itr = obstacles.begin(); itr != obstacles.end(); ++itr)
				{
					if (itr->id == m_ooi_idx)
					{
						m_ooi.SetObject(*itr);
						obstacles.erase(itr);
						break;
					}
				}
			}

			else if (line.compare("G") == 0)
			{
				getline(SCENE, line);

				std::stringstream ss(line);
				std::string split;
				while (ss.good())
				{
					getline(ss, split, ',');
					m_goal.push_back(std::stod(split));
				}

				std::swap(m_goal[3], m_goal[5]);
				if (std::fabs(m_goal[3]) >= 1e-4)
				{
					m_goal[3] = 0.0;
					m_goal[4] = 0.0;
					m_goal[5] = smpl::angles::normalize_angle(m_goal[5] + M_PI);
				}
				SMPL_INFO("Goal (x, y, z, yaw): (%f, %f, %f, %f)", m_goal[0], m_goal[1], m_goal[2], m_goal[5]);
			}
		}
	}

	SCENE.close();
}

void Planner::writePlanState(int iter)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/txt/";

	std::stringstream ss;
	ss << std::setw(4) << std::setfill('0') << iter;
	std::string s = ss.str();

	filename += s;
	filename += ".txt";

	std::ofstream DATA;
	DATA.open(filename, std::ofstream::out);

	DATA << 'O' << '\n';
	int o = m_cc->NumObstacles() + 1 + m_agents.size() + 3;
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

	State loc = m_ooi.GetCurrentState()->state;
	auto agent_obs = m_ooi.GetObject();
	movable = "False";
	DATA << 999 << ',' // for visualisation purposes
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

	movable = "True";
	for (const auto& a: m_agents)
	{
		loc = a.GetCurrentState()->state;
		agent_obs = a.GetObject();
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

	agent_obs = m_robot->GetObject(*(m_robot->GetCurrentState()));
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
	o = 1 + m_agents.size();
	DATA << o << '\n';

	auto move = m_ooi.GetMoveTraj();
	DATA << 999 << '\n';
	DATA << move->size() << '\n';
	for (const auto& s: *move) {
		DATA << s.state.at(0) << ',' << s.state.at(1) << '\n';
	}

	for (const auto& a: m_agents)
	{
		agent_obs = a.GetObject();
		move = a.GetMoveTraj();
		DATA << agent_obs->back().id << '\n';
		DATA << move->size() << '\n';
		for (const auto& s: *move) {
			DATA << s.state.at(0) << ',' << s.state.at(1) << '\n';
		}
	}

	DATA.close();
}

void Planner::setupGlobals()
{
	m_ph.getParam("/fridge", FRIDGE);
	m_ph.getParam("whca/planning_time", WHCA_PLANNING_TIME);
	m_ph.getParam("whca/window", WINDOW);
	m_ph.getParam("whca/goal_thresh", GOAL_THRESH);
	m_ph.getParam("whca/res", RES);
	m_ph.getParam("whca/grid", GRID);
	m_ph.getParam("whca/cost_mult", COST_MULT);
	m_ph.getParam("robot/semi_minor", SEMI_MINOR);
	m_ph.getParam("robot/robot_obj_mass", R_MASS);
	m_ph.getParam("robot/speed", R_SPEED);
	m_ph.getParam("goal/save", SAVE);
}

int Planner::armId()
{
	int arm;
	for (const auto& name : m_robot->GetStartState()->joint_state.name)
	{
		if (name.find("r_") == 0)
		{
			arm = 1;
			break;
		}

		else if (name.find("l_") == 0)
		{
			arm = 0;
			break;
		}
	}
	return arm;
}

bool Planner::savePlanData()
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/PLANNER.csv";

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "WHCACalls,RobotPlanTime,"
				<< "MAPFSolveTime,FirstOrderInteractions\n";
	}

	STATS << m_scene_id << ','
			<< m_stats["whca_attempts"] << ',' << m_stats["robot_plan_time"] << ','
			<< m_stats["mapf_time"] << ',' << m_stats["first_order_interactions"] << '\n';
	STATS.close();

	m_stats["whca_attempts"] = 0;
	m_stats["robot_plan_time"] = 0.0;
	m_stats["mapf_time"] = 0.0;
	m_stats["first_order_interactions"] = 0;
}

} // namespace clutter
