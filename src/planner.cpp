#include <pushplan/planner.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/constants.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/cbs.hpp>
#include <pushplan/helpers.hpp>

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

	m_robot = std::make_shared<Robot>();
	m_robot->SetID(0);

	m_stats["whca_attempts"] = 0;
	m_stats["robot_plan_time"] = 0.0;
	m_stats["mapf_time"] = 0.0;
	m_stats["first_order_interactions"] = 0;
	m_plan_time = 0.0;

	m_ph.getParam("goal/plan_budget", m_plan_budget);
	m_ph.getParam("goal/sim_budget", m_sim_budget);
	m_ph.getParam("goal/total_budget", m_total_budget);

	setupGlobals();

	m_agent_map.clear();
	m_agents.clear();
	m_ooi = std::make_shared<Agent>();

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
	m_ooi_gf = m_cc->GetRandomStateOutside(m_ooi->GetFCLObject());
	ContToDisc(m_ooi_gf, m_ooi_g);
	m_cc->AddObstacle(m_ooi->GetObject()->back());

	m_robot->SetCC(m_cc);
	m_ooi->SetCC(m_cc);
	for (auto& a: m_agents) {
		a->SetCC(m_cc);
	}

	if (!m_robot->Setup())
	{
		SMPL_ERROR("Robot setup failed!");
		return false;
	}
	m_ooi->Setup();
	for (auto& a: m_agents) {
		a->Setup();
	}

	if (!m_robot->ProcessObstacles(all_obstacles))
	{
		SMPL_ERROR("Robot collision space setup failed!");
		return false;
	}
	all_obstacles.clear();
	m_robot->SetOOI(m_ooi->GetObject());
	m_robot->SetMovables(m_agents);

	int t = 0, grasp_tries;
	m_ph.getParam("robot/grasp_tries", grasp_tries);
	for (; t < grasp_tries; ++t)
	{
		if (m_robot->ComputeGrasps(m_goal)) {
			break;
		}
	}
	if (t == grasp_tries)
	{
		SMPL_ERROR("Robot failed to compute grasp states!");
		return false;
	}
	m_robot->ProcessObstacles({ m_ooi->GetObject()->back() });

	m_simulate = m_nh.advertiseService("run_sim", &Planner::runSim, this);
	m_animate = m_nh.advertiseService("anim_soln", &Planner::animateSolution, this);
	m_rearrange = m_nh.advertiseService("rearrange", &Planner::rearrange, this);

	setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID());

	m_robot->SetSim(m_sim);

	m_cbs = std::make_shared<CBS>(m_robot, m_agents);
	m_cbs->SetCC(m_cc);

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
	while (!setupProblem()) {
		continue;
	}

	m_cbs->Solve();

	return true;
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
		a->Setup(); // updates original object
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
	if (m_exec_interm.empty() || !setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID())) {
		return false;
	}

	moveit_msgs::RobotTrajectory to_exec;
	m_robot->ConvertTraj(m_exec_interm, to_exec);
	comms::ObjectsPoses rearranged = m_rearranged;
	bool success = true;
	double start_time = GetTime();
	if (!m_sim->ExecTraj(to_exec.joint_trajectory, rearranged, m_robot->GraspAt(), m_ooi->GetID()))
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
		a->ResetObject();
		final_objects.push_back(a->GetObject()->back());
	}
	m_robot->ProcessObstacles(final_objects);

	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	animateSolution(req, resp);
}

bool Planner::rearrange(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	// for (auto& a: m_agents) {
	// 	a->ResetObject();
	// }

	// auto robot_conflicts = m_cc->GetConflictsOf(100); // get first order conflicts
	// if (robot_conflicts.empty()) {
	// 	return false;
	// }

	// comms::ObjectsPoses rearranged = m_rearranged;
	// std::vector<int> ids_attempted;

	// bool push_found = false;
	// while (!push_found && !robot_conflicts.empty())
	// {
	// 	auto i = robot_conflicts.begin();
	// 	for (auto iter = robot_conflicts.begin(); iter != robot_conflicts.end(); ++iter)
	// 	{
	// 		if (iter == i) {
	// 			continue;
	// 		}

	// 		if (iter->second < i->second) {
	// 			i = iter;
	// 		}
	// 	}

	// 	int oid = i->first.first;
	// 	int oid_t = i->second;
	// 	SMPL_INFO("Rearranging object %d", oid);
	// 	// m_agents.at(m_agent_map[oid])->ResetObject();

	// 	auto oid_conflicts = m_cc->GetConflictsOf(oid);
	// 	std::vector<Object> new_obstacles;
	// 	for (const auto& a: m_agents)
	// 	{
	// 		if (a->GetObject()->back().id == oid) {
	// 			continue; // selected object cannot be obstacle
	// 		}

	// 		bool oid_child = false;
	// 		for (const auto& c: oid_conflicts)
	// 		{
	// 			if (c.first.first == a->GetObject()->back().id) {
	// 				oid_child = true;
	// 				break;
	// 			}
	// 		}
	// 		if (oid_child) {
	// 			continue; // children of selected object may not be obstacle
	// 		}

	// 		new_obstacles.push_back(a->GetObject()->back());
	// 		SMPL_INFO("Adding object %d as obstacle", a->GetObject()->back().id);
	// 	}

	// 	for (const auto& c: robot_conflicts)
	// 	{
	// 		if (c.first.first != oid && c.second < oid_t)
	// 		{
	// 			// first order conflicts earlier than selected object
	// 			// must be obstacles
	// 			new_obstacles.push_back(m_agents.at(m_agent_map[c.first.first])->GetObject()->back());
	// 			SMPL_INFO("Adding object %d as obstacle", c.first.first);
	// 		}
	// 	}

	// 	for (const auto& id: ids_attempted)
	// 	{
	// 		if (id == oid) {
	// 			continue;
	// 		}

	// 		// first order conflicts already selected (for which no push was found)
	// 		// must be obstacles
	// 		new_obstacles.push_back(m_agents.at(m_agent_map[id])->GetObject()->back());
	// 		SMPL_INFO("Adding rearranged object %d as obstacle", id);
	// 	}
	// 	// add new obstacles
	// 	m_robot->ProcessObstacles(new_obstacles);

	// 	// get push location
	// 	std::vector<double> push;
	// 	m_agents.at(m_agent_map[oid])->GetSE2Push(push);
	// 	SMPL_INFO("Object %d push is (%f, %f, %f)", oid, push[0], push[1], push[2]);

	// 	// set push location goal for robot
	// 	m_robot->SetPushGoal(push);

	// 	// plan to push location
	// 	// m_robot->PlanPush creates the planner internally, because it might
	// 	// change KDL chain during the process
	// 	comms::ObjectsPoses result;

	// 	if (m_robot->PlanPush(oid, m_agents.at(m_agent_map[oid]).GetLastTraj(), m_agents.at(m_agent_map[oid])->GetObject()->back(), rearranged, result))
	// 	{
	// 		push_found = true;
	// 		m_rearrangements.push_back(m_robot->GetLastPlan());

	// 		// update positions of moved objects
	// 		updateAgentPositions(result, rearranged);
	// 	}
	// 	if (SAVE) {
	// 		m_robot->SavePushData(m_scene_id);
	// 	}

	// 	// remove new obstacles
	// 	m_robot->ProcessObstacles(new_obstacles, true);

	// 	robot_conflicts.erase(i);

	// 	auto it = std::find(begin(ids_attempted), end(ids_attempted), oid);
	// 	if (it == end(ids_attempted)) {
	// 		ids_attempted.push_back(oid);
	// 	}
	// }
	// m_rearranged = rearranged;

	return true;
}

bool Planner::animateSolution(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	m_robot->AnimateSolution();
	return true;
}

bool Planner::runSim(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID());

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

	if (!m_sim->ExecTraj(m_exec, dummy, m_robot->GraspAt(), m_ooi->GetID()))
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

// fcl::CollisionObject* Planner::GetObject(const LatticeState& s, int priority)
// {
// 	fcl::Transform3f pose;
// 	pose.setIdentity();

// 	if (priority == 1)
// 	{
// 		m_ooi->UpdatePose(s);
// 		return m_ooi->GetFCLObject();;
// 	}
// 	else
// 	{
// 		m_agents.at(m_priorities.at(priority-2)).UpdatePose(s);
// 		return m_agents.at(m_priorities.at(priority-2)).GetFCLObject();
// 	}
// }

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
			m_agents.at(m_agent_map[o.id])->SetObjectPose(o.xyz, o.rpy);
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
	// CBS TODO: assign starts and goals to agents

	int result = cleanupLogs();
	if (result == -1) {
		SMPL_ERROR("system command errored!");
	}

	// Set agent current positions and time
	m_ooi->Init();
	m_robot->Init();
	// if (!m_robot->RandomiseStart()) {
	// 	return false;
	// }
	for (auto& a: m_agents) {
		a->Init();
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
			m_ooi->SetObject(o);
			m_goal.clear();

			double xdisp = std::cos(o.o_yaw) * 0.1;
			double ydisp = std::sin(o.o_yaw) * 0.1;
			m_goal = {o.o_x - xdisp, o.o_y - ydisp, obstacles.at(0).o_z + obstacles.at(0).z_size + 0.05, 0.0, 0.0, -o.o_yaw};

			ooi_set = true;
			continue;
		}

		o.CreateCollisionObjects();
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

		o.CreateCollisionObjects();
		std::shared_ptr<Agent> movable(new Agent(o));
		m_agents.push_back(std::move(movable));
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

					o.CreateCollisionObjects();
					if (o.movable) {
						std::shared_ptr<Agent> movable(new Agent(o));
						m_agents.push_back(std::move(movable));
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
				int ooi_idx = std::stoi(line); // object of interest ID

				for (auto itr = obstacles.begin(); itr != obstacles.end(); ++itr)
				{
					if (itr->id == ooi_idx)
					{
						m_ooi->SetObject(*itr);
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
	m_ph.getParam("occupancy_grid/res", DF_RES);
	m_ph.getParam("goal/cc_2d", CC_2D);
	m_ph.getParam("goal/cc_3d", CC_3D);
	m_ph.getParam("whca/eecbs_mult", ECBS_MULT);
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
