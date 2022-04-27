#include <pushplan/search/planner.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/cbs.hpp>
#include <pushplan/search/cbswp.hpp>
#include <pushplan/search/pbs.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/geometry.hpp>
#include <pushplan/utils/helpers.hpp>

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

	std::vector<Object> all_obstacles;
	if (m_scene_id < 0)	{ // init agents from simulator
		init_agents(ycb, all_obstacles);
	}
	else {
		parse_scene(all_obstacles); // TODO: relax ycb = false assumption
	}

	// create collision checker
	m_cc = std::make_shared<CollisionChecker>(this, all_obstacles);

	// Get OOI goal
	m_ooi_gf = m_cc->GetRandomStateOutside(m_ooi->GetFCLObject());
	ContToDisc(m_ooi_gf, m_ooi_g);
	m_cc->AddObstacle(m_ooi->GetObject());

	m_robot->SetSim(m_sim);
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

	if (!m_robot->ProcessObstacles(all_obstacles))
	{
		SMPL_ERROR("Robot collision space setup failed!");
		return false;
	}
	all_obstacles.clear();
	m_robot->SetOOI(m_ooi->GetObject());
	// m_robot->SetMovables(m_agents);

	int t = 0, grasp_tries;
	m_ph.getParam("robot/grasping/tries", grasp_tries);
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
	m_robot->VizCC();

	setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID());

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

bool Planner::SetupNGR()
{
	std::vector<Object*> movable_obstacles;
	for (const auto& a: m_agents) {
		movable_obstacles.push_back(a->GetObject());
	}

	m_robot->ProcessObstacles({ m_ooi->GetObject() }, true);
	// if (!m_robot->PlanApproachOnly(movable_obstacles)) {
	if (!m_robot->PlanRetrieval(movable_obstacles)) {
		return false;
	}
	m_robot->ProcessObstacles({ m_ooi->GetObject() });
	m_robot->UpdateNGR();

	double ox, oy, oz, sx, sy, sz;
	m_sim->GetShelfParams(ox, oy, oz, sx, sy, sz);

	for (auto& a: m_agents) {
		a->SetObstacleGrid(m_robot->ObsGrid());
		a->SetNGRGrid(m_robot->NGRGrid());
		a->ResetSolution();
		if (ALGO == MAPFAlgo::OURS) {
			a->ComputeNGRComplement(ox, oy, oz, sx, sy, sz);
		}
	}

	// m_ooi->SetObstacleGrid(m_robot->Grid());
	// if (ALGO == MAPFAlgo::OURS) {
	// 	m_ooi->ComputeNGRComplement(ox, oy, oz, sx, sy, sz);
	// }

	return true;
}

bool Planner::createCBS()
{
	switch (ALGO)
	{
		case MAPFAlgo::VCBS:
		case MAPFAlgo::ECBS:
		{
			m_cbs = std::make_shared<CBS>(m_robot, m_agents, m_scene_id);
			break;
		}
		case MAPFAlgo::CBSWP:
		{
			m_cbs = std::make_shared<CBSwP>(m_robot, m_agents, m_scene_id);
			break;
		}
		case MAPFAlgo::PBS:
		{
			m_cbs = std::make_shared<PBS>(m_robot, m_agents, m_scene_id);
			break;
		}
		default:
		{
			SMPL_ERROR("MAPF Algo type currently not supported!");
			return false;
		}
	}
	m_cbs->SetCC(m_cc);

	return true;
}

bool Planner::Plan()
{
	double start_time = GetTime();
	while (!SetupNGR()) {
		continue;
	}

	bool backwards = true; // ALGO == MAPFAlgo::OURS;
	while (!setupProblem(backwards)) {
		continue;
	}

	if (!createCBS()) {
		return false;
	}

	bool result = m_cbs->Solve(backwards);
	m_cbs->SaveStats();

	m_plan_time += GetTime() - start_time;

	return result;
}

bool Planner::Rearrange()
{
	if (!rearrange()) {
		SMPL_WARN("There were no conflicts to rearrange!");
		return false;
	}
	return true;
}

std::uint32_t Planner::RunSim()
{
	if (!runSim()) {
		SMPL_ERROR("Simulation failed!");
	}
	return m_violation;
}

bool Planner::TryExtract()
{
	if (!setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID())) {
		return false;
	}

	comms::ObjectsPoses rearranged = m_rearranged;
	bool success = true;
	double start_time = GetTime();
	if (!m_sim->ExecTraj(m_robot->GetLastPlan(), rearranged, m_robot->GraspAt(), m_ooi->GetID()))
	{
		SMPL_ERROR("Failed to exec traj!");
		success = false;
	}

	m_sim->RemoveConstraint();

	return success;
}

void Planner::AnimateSolution()
{
	std::vector<Object*> final_objects;
	for (auto& a: m_agents) {
		// a->ResetObject();
		final_objects.push_back(a->GetObject());
	}
	m_robot->ProcessObstacles(final_objects);

	animateSolution();
}

bool Planner::rearrange()
{
	comms::ObjectsPoses rearranged = m_rearranged;

	auto cbs_soln = m_cbs->GetSolution();
	for (auto& path: cbs_soln->m_solution)
	{
		if (path.first == 0 || path.second.front().coord == path.second.back().coord)
		{
			// either this is robot or the object did not move
			continue;
		}

		// get push location
		std::vector<double> push;
		m_agents.at(m_agent_map[path.first])->GetSE2Push(push);
		SMPL_INFO("Object %d push is (%f, %f, %f)", path.first, push[0], push[1], push[2]);

		// other movables to be considered as obstacles
		std::vector<Object*> movable_obstacles;
		for (const auto& a: m_agents)
		{
			if (a->GetID() == path.first) {
				continue; // selected object cannot be obstacle
			}
			movable_obstacles.push_back(a->GetObject());
		}

		// plan to push location
		// m_robot->PlanPush creates the planner internally, because it might
		// change KDL chain during the process
		comms::ObjectsPoses result;

		if (m_robot->PlanPush(m_agents.at(m_agent_map[path.first]).get(), push, movable_obstacles, rearranged, result))
		{
			m_rearrangements.push_back(m_robot->GetLastPlan());

			// update positions of moved objects
			updateAgentPositions(result, rearranged);
		}
		if (SAVE) {
			m_robot->SavePushData(m_scene_id);
		}
	}
	m_rearranged = rearranged;

	return true;
}

bool Planner::animateSolution()
{
	m_robot->AnimateSolution();
	return true;
}

bool Planner::runSim()
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

void Planner::updateAgentPositions(
	const comms::ObjectsPoses& result,
	comms::ObjectsPoses& rearranged)
{
	for (const auto& o: result.poses)
	{
		auto search = m_agent_map.find(o.id);
		if (search != m_agent_map.end())
		{
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

bool Planner::setupProblem(bool backwards)
{
	// CBS TODO: assign starts and goals to agents

	int result = cleanupLogs();
	if (result == -1) {
		SMPL_ERROR("system command errored!");
	}

	// Set agent current positions and time
	// m_robot->Init();
	// if (!m_robot->RandomiseStart()) {
	// 	return false;
	// }

	for (auto& a: m_agents) {
		a->Init(backwards);
	}

	return true;
}

int Planner::cleanupLogs()
{
	std::string files(__FILE__), command;
	auto found = files.find_last_of("/\\");
	files = files.substr(0, found + 1) + "../../dat/txt/*.txt";

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
		o.desc.id = immov_obj_ids->at(i).first;
		o.desc.type = i < tables ? -1 : 0; // table or immovable
		o.desc.o_x = immov_objs->at(i).at(0);
		o.desc.o_y = immov_objs->at(i).at(1);
		o.desc.o_z = immov_objs->at(i).at(2);
		o.desc.o_roll = immov_objs->at(i).at(3);
		o.desc.o_pitch = immov_objs->at(i).at(4);
		o.desc.o_yaw = immov_objs->at(i).at(5);

		if (ycb && i >= tables)
		{
			auto itr = YCB_OBJECT_DIMS.find(immov_obj_ids->at(i).second);
			if (itr != YCB_OBJECT_DIMS.end())
			{
				o.desc.x_size = itr->second.at(0);
				o.desc.y_size = itr->second.at(1);
				o.desc.z_size = itr->second.at(2);
				o.desc.o_yaw += itr->second.at(3);
			}
		}
		else
		{
			o.desc.x_size = immov_objs->at(i).at(6);
			o.desc.y_size = immov_objs->at(i).at(7);
			o.desc.z_size = immov_objs->at(i).at(8);
		}
		o.desc.movable = false;
		o.desc.shape = immov_obj_ids->at(i).second;
		o.desc.mass = i == 0 ? 0 : -1;
		o.desc.locked = i == 0 ? true : false;
		o.desc.mu = -1;
		o.desc.ycb = i >= tables ? ycb : false;

		if (!ooi_set && i >= tables)
		{
			m_ooi->SetObject(o);
			m_goal.clear();

			double xdisp = std::cos(o.desc.o_yaw) * 0.1;
			double ydisp = std::sin(o.desc.o_yaw) * 0.1;
			m_goal = {o.desc.o_x - xdisp, o.desc.o_y - ydisp, obstacles.at(0).desc.o_z + obstacles.at(0).desc.z_size + 0.05, 0.0, 0.0, -o.desc.o_yaw};

			ooi_set = true;
			continue;
		}

		o.CreateCollisionObjects();
		o.CreateSMPLCollisionObject();
		o.GenerateCollisionModels();

		obstacles.push_back(o);
	}

	for (size_t i = 0; i < mov_objs->size(); ++i)
	{
		o.desc.id = mov_obj_ids->at(i).first;
		o.desc.type = 1; // movable
		o.desc.o_x = mov_objs->at(i).at(0);
		o.desc.o_y = mov_objs->at(i).at(1);
		o.desc.o_z = mov_objs->at(i).at(2);
		o.desc.o_roll = mov_objs->at(i).at(3);
		o.desc.o_pitch = mov_objs->at(i).at(4);
		o.desc.o_yaw = mov_objs->at(i).at(5);

		if (ycb)
		{
			auto itr = YCB_OBJECT_DIMS.find(mov_obj_ids->at(i).second);
			if (itr != YCB_OBJECT_DIMS.end())
			{
				o.desc.x_size = itr->second.at(0);
				o.desc.y_size = itr->second.at(1);
				o.desc.z_size = itr->second.at(2);
				o.desc.o_yaw += itr->second.at(3);
			}
		}
		else
		{
			o.desc.x_size = mov_objs->at(i).at(6);
			o.desc.y_size = mov_objs->at(i).at(7);
			o.desc.z_size = mov_objs->at(i).at(8);
		}
		o.desc.movable = true;
		o.desc.shape = mov_obj_ids->at(i).second;
		o.desc.mass = i == 0 ? 0 : -1;
		o.desc.locked = false;
		o.desc.mu = -1;
		o.desc.ycb = ycb;

		o.CreateCollisionObjects();
		o.CreateSMPLCollisionObject();
		o.GenerateCollisionModels();

		std::shared_ptr<Agent> movable(new Agent(o));
		m_agents.push_back(std::move(movable));
		m_agent_map[o.desc.id] = m_agents.size() - 1;
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
							case 0: o.desc.id = std::stoi(split); break;
							case 1: o.desc.shape = std::stoi(split); break;
							case 2: o.desc.type = std::stoi(split); break;
							case 3: o.desc.o_x = std::stof(split); break;
							case 4: o.desc.o_y = std::stof(split); break;
							case 5: o.desc.o_z = std::stof(split); break;
							case 6: o.desc.o_roll = std::stof(split); break;
							case 7: o.desc.o_pitch = std::stof(split); break;
							case 8: o.desc.o_yaw = std::stof(split); break;
							case 9: o.desc.x_size = std::stof(split); break;
							case 10: o.desc.y_size = std::stof(split); break;
							case 11: o.desc.z_size = std::stof(split); break;
							case 12: {
								o.desc.mass = std::stof(split);
								o.desc.locked = o.desc.mass == 0;
								break;
							}
							case 13: o.desc.mu = std::stof(split); break;
							case 14: o.desc.movable = (split.compare("True") == 0); break;
						}
						o.desc.ycb = false;
						count++;
					}

					o.CreateCollisionObjects();
					o.CreateSMPLCollisionObject();
					o.GenerateCollisionModels();

					if (o.desc.movable) {
						std::shared_ptr<Agent> movable(new Agent(o));
						m_agents.push_back(std::move(movable));
						m_agent_map[o.desc.id] = m_agents.size() - 1;
					}
					else {
						// o.desc.x_size += RES;
						// o.desc.y_size += RES;
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
					if (itr->desc.id == ooi_idx)
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
	m_ph.getParam("mapf/planning_time", MAPF_PLANNING_TIME);
	m_ph.getParam("mapf/res", RES);
	m_ph.getParam("mapf/cost_mult", COST_MULT);
	m_ph.getParam("mapf/goal_thresh", GOAL_THRESH);
	m_ph.getParam("mapf/whca/window", WINDOW);
	m_ph.getParam("mapf/whca/grid", GRID);
	m_ph.getParam("robot/semi_minor", SEMI_MINOR);
	m_ph.getParam("robot/speed", R_SPEED);
	m_ph.getParam("goal/save", SAVE);
	m_ph.getParam("goal/cc_2d", CC_2D);
	m_ph.getParam("goal/cc_3d", CC_3D);
	m_ph.getParam("occupancy_grid/res", DF_RES);

	int llhc, hlhc, algo;
	m_ph.getParam("mapf/cbs/algo", algo);
	m_ph.getParam("mapf/cbs/llhc", llhc);
	m_ph.getParam("mapf/cbs/hlhc", hlhc);
	m_ph.getParam("mapf/cbs/ecbs_mult", ECBS_MULT);
	ALGO = static_cast<MAPFAlgo>(algo);
	LLHC = static_cast<LowLevelConflictHeuristic>(llhc);
	HLHC = static_cast<HighLevelConflictHeuristic>(hlhc);
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
	filename = filename.substr(0, found + 1) + "../../dat/PLANNER.csv";

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
