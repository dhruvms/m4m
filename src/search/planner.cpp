#include <pushplan/search/planner.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/cbs.hpp>
#include <pushplan/search/cbswp.hpp>
#include <pushplan/search/pbs.hpp>
#include <pushplan/sampling/sampling_planner.hpp>
#include <pushplan/sampling/rrt.hpp>
#include <pushplan/sampling/rrtstar.hpp>
#include <pushplan/sampling/tcrrt.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/geometry.hpp>
#include <pushplan/utils/helpers.hpp>
#include <comms/ObjectPose.h>

#include <smpl/console/console.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <boost/math/distributions/beta.hpp>

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
	m_robot->SetSim(m_sim);

	m_stats["robot_planner_time"] = 0.0;
	m_stats["push_planner_time"] = 0.0;
	m_stats["push_input_time"] = 0.0;
	m_stats["mapf_time"] = 0.0;
	m_stats["sim_time"] = 0.0;

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

	m_robot->SetCC(m_cc);
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

	// Ready OOI
	if (m_ooi->Set())
	{
		m_ooi_gf = m_cc->GetRandomStateOutside(m_ooi->GetFCLObject());
		ContToDisc(m_ooi_gf, m_ooi_g);
		m_cc->AddObstacle(m_ooi->GetObject());
		m_ooi->SetCC(m_cc);
		m_robot->SetOOI(m_ooi->GetObject());
		// m_robot->SetMovables(m_agents);

		// Compute robot grasping states and plan path through them
		int t = 0, grasp_tries;
		m_ph.getParam("robot/grasping/tries", grasp_tries);
		m_timer = GetTime();
		for (; t < grasp_tries; ++t)
		{
			if (m_robot->ComputeGrasps(m_goal)) {
				if (SetupNGR()) {
					break;
				}
			}
		}
		m_stats["robot_planner_time"] += GetTime() - m_timer;
		if (t == grasp_tries)
		{
			SMPL_ERROR("Robot failed to compute grasp states!");
			return false;
		}
	}
	m_robot->VizCC();

	setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID());
	m_violation = 0x00000008;
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
	m_C = 25;
	m_ph.getParam("robot/pushing/input", m_push_input);

	return true;
}

bool Planner::Alive()
{
	// double plan_time = m_stats["robot_planner_time"] + m_stats["push_planner_time"] + m_stats["mapf_time"];
	// if (plan_time > m_plan_budget) {
	// 	return false;
	// }

	// if (m_stats["sim_time"] + m_robot->SimTime() > m_sim_budget) {
	// 	return false;
	// }

	// total_time includes:
	// 	1. time taken to compute grasps
	// 	2. time taken to plan robot path and setup ngr
	// 	3. time taken to sample, plan, simulate pushes
	// 	4. time taken to solve mapf problem
	double total_time = m_stats["robot_planner_time"] + m_stats["push_planner_time"] + m_stats["mapf_time"];
	if (total_time > m_total_budget) {
		if (m_cbs)
		{
			m_cbs->WriteRoot();
			m_cbs.reset();
		}
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
	if (!m_robot->PlanRetrieval(movable_obstacles))	{
		return false;
	}
	m_robot->ProcessObstacles({ m_ooi->GetObject() });
	m_robot->UpdateNGR();
	m_exec = m_robot->GetLastPlanProfiled();

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

bool Planner::FinalisePlan()
{
	m_timer = GetTime();

	std::vector<Object*> movable_obstacles;
	for (const auto& a: m_agents) {
		movable_obstacles.push_back(a->GetObject());
	}

	m_robot->ProcessObstacles({ m_ooi->GetObject() }, true);
	// if (!m_robot->PlanApproachOnly(movable_obstacles)) {
	smpl::RobotState start_state = {};
	if (!m_rearrangements.empty()) {
		start_state = m_rearrangements.back().points.back().positions;
	}
	if (!m_robot->PlanRetrieval(movable_obstacles, true, start_state))
	{
		m_stats["robot_planner_time"] += GetTime() - m_timer;
		return false;
	}
	m_robot->ProcessObstacles({ m_ooi->GetObject() });
	m_exec = m_robot->GetLastPlanProfiled();

	m_stats["robot_planner_time"] += GetTime() - m_timer;
	return true;
}

bool Planner::RunRRT()
{
	int samples, steps, planner;
	double gbias, gthresh, timeout;

	m_ph.getParam("sampling/samples", samples);
	m_ph.getParam("sampling/steps", steps);
	m_ph.getParam("sampling/gbias", gbias);
	m_ph.getParam("sampling/gthresh", gthresh);
	m_ph.getParam("sampling/timeout", timeout);
	m_ph.getParam("sampling/planner", planner);

	switch (planner)
	{
		case 0:
		{
			m_sampling_planner = std::make_shared<sampling::RRT>(
				samples, steps, gbias, gthresh, timeout);
			SMPL_INFO("Run RRT");
			break;
		}
		case 1:
		{
			m_sampling_planner = std::make_shared<sampling::RRTStar>(
				samples, steps, gbias, gthresh, timeout);
			SMPL_INFO("Run RRTStar");
			break;
		}
		case 2:
		{
			int mode, I, J;
			m_ph.getParam("sampling/tcrrt/mode", mode);
			m_ph.getParam("sampling/tcrrt/I", I);
			m_ph.getParam("sampling/tcrrt/J", J);

			m_sampling_planner = std::make_shared<sampling::TCRRT>(
				samples, steps, gbias, gthresh, timeout, mode, I, J);
			SMPL_INFO("Run TCRRT");

			smpl::RobotState pregrasp_state;
			Eigen::Affine3d pregrasp_pose;
			m_robot->GetPregraspState(pregrasp_state);
			m_robot->ComputeFK(pregrasp_state, pregrasp_pose);
			m_robot->VizPlane(pregrasp_pose.translation().z());
			m_sampling_planner->SetConstraintHeight(pregrasp_pose.translation().z());
			break;
		}
		default:
		{
			SMPL_ERROR("Unsupported sampling planner!");
			return false;
		}
	}

	m_sampling_planner->SetRobot(m_robot);
	m_sampling_planner->SetRobotGoalCallback(std::bind(&Robot::GetPregraspState, m_robot.get(), std::placeholders::_1));

	smpl::RobotState start_config;
	m_robot->GetHomeState(start_config);
	comms::ObjectsPoses start_objects = GetStartObjects();
	m_sampling_planner->SetStartState(start_config, start_objects);

	bool plan_success = m_sampling_planner->Solve();
	bool exec_success = false;
	// if (plan_success)
	{
		m_sampling_planner->ExtractTraj(m_exec);
		exec_success = m_sim->ExecTraj(m_exec, start_objects);
	}

	auto rrt_stats = m_sampling_planner->GetStats();
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/";
	switch (planner)
	{
		case 0:
		{
			filename += "RRT.csv";
			break;
		}

		case 1:
		{
			filename += "RRTStar.csv";
			break;
		}

		case 2:
		{
			int mode;
			m_ph.getParam("sampling/tcrrt/mode", mode);
			filename += "TCRRT_" + std::to_string(mode) + ".csv";
			break;
		}
	}

	// m_stats["
	// m_stats["goal_samples"] = 0.0;
	// m_stats["random_samples"] = 0.0;

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "PlanSuccess,ExecSuccess,SimCalls,SimTime,"
				<< "TotalVertices,PlanTime,SolnCost,"
				<< "FirstGoalVertex,FirstSolnTime,"
				<< "GoalSamples,RandomSamples\n";
	}

	STATS << m_scene_id << ','
			<< plan_success << ',' << exec_success << ','
			<< rrt_stats["sims"] << ',' << rrt_stats["sim_time"] << ','
			<< rrt_stats["vertices"] << ',' << rrt_stats["plan_time"] << ','
			<< rrt_stats["soln_cost"] << ',' << rrt_stats["first_goal"] << ','
			<< rrt_stats["first_soln_time"] << ',' << rrt_stats["goal_samples"] << ','
			<< rrt_stats["random_samples"] << '\n';
	STATS.close();

	return false;
}

void Planner::RunStudy(int study)
{
	int N;
	if (study == 0)
	{
		m_ph.getParam("robot/yoshikawa_tries", N);
		m_robot->RunManipulabilityStudy(N);
	}
	else if (study == 1)
	{
		m_ph.getParam("robot/push_ik_yaws", N);
		m_robot->RunPushIKStudy(N);
	}
}

bool Planner::RunWHCA()
{
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

	int runs;
	m_ph.getParam("robot/runs", runs);
	for (int power = 0; power < 4; ++power)
	{
		WINDOW = std::pow(3, power);
		bool saved = false;
		int saved_flowtime = 0;
		for (int tries = 0; tries < runs; ++tries)
		{
			m_ooi->InitWHCA();
			for (auto& a: m_agents) {
				a->InitWHCA();
			}

			bool done = false, success = false;
			int iter = 0, makespan = -1, flowtime = 0, failure = 0;
			double plan_time = 0.0;

			m_timer = GetTime();
			prioritise();
			while (!done)
			{
				int priority = 0;
				for (const auto& p: m_priorities)
				{
					double ll_time = GetTime();
					if (!m_agents.at(p)->PlanPrioritised(priority))
					{
						failure = 1;
						plan_time = GetTime() - m_timer;
						break;
					}
					if (GetTime() - ll_time > 4.0)
					{
						failure = 2;
						plan_time = GetTime() - m_timer;
						break;
					}
					++priority;
				}
				if (failure > 0)	{
					break;
				}

				step_agents(); // take 1 step by default
				prioritise();

				++iter;

				done = true;
				for (auto& a: m_agents)
				{
					done = done && a->ReachedGoal();
					if (!done) {
						break;
					}
				}
				if (done)
				{
					plan_time = GetTime() - m_timer;
					success = true;

					for (const auto& a: m_agents)
					{
						int move_length = int(a->MoveTraj()->size());
						flowtime += move_length;
						if (move_length > makespan) {
							makespan = move_length;
						}
					}
					if (!saved || saved_flowtime > flowtime)
					{
						writeWHCAState(iter, ngr);
						saved = true;
						saved_flowtime = flowtime;
					}
				}

				if ((GetTime() - m_timer) > 30.0)
				{
					failure = 3;
					plan_time = GetTime() - m_timer;
				}

				if (failure > 0) {
					break;
				}
			}

			std::string filename(__FILE__);
			auto found = filename.find_last_of("/\\");
			filename = filename.substr(0, found + 1) + "../../dat/WHCA.csv";

			bool exists = FileExists(filename);
			std::ofstream STATS;
			STATS.open(filename, std::ofstream::out | std::ofstream::app);
			if (!exists)
			{
				STATS << "UID,Window,"
						<< "Success,Failure?,Iterations,PlanTime,"
						<< "Makespan,Flowtime\n";
			}

			STATS << m_scene_id << ','
					<< WINDOW << ',' << success << ',' << failure << ','
					<< iter << ',' << plan_time << ','
					<< makespan << ',' << flowtime << '\n';
			STATS.close();
		}
	}

	return true;
}

void Planner::prioritise()
{
	m_priorities.clear();
	m_priorities.resize(m_agents.size());
	std::iota(m_priorities.begin(), m_priorities.end(), 0);

	State ooi_s = m_ooi->CurrentState().state;
	std::vector<double> dists(m_agents.size(), std::numeric_limits<double>::max());
	for (size_t i = 0; i < m_agents.size(); ++i) {
		dists.at(i) = std::min(dists.at(i), EuclideanDist(m_agents.at(i)->CurrentState().state, ooi_s));
	}
	std::stable_sort(m_priorities.begin(), m_priorities.end(),
		[&dists](size_t i1, size_t i2) { return dists[i1] < dists[i2]; });
}

void Planner::step_agents(int k)
{
	// TODO: make sure we can handle k > 1
	for (auto& a: m_agents) {
		a->Step(k);
	}
}

void Planner::writeWHCAState(int iter, std::set<Coord, coord_compare> ngr)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/txt/";

	std::stringstream ss;
	ss << "WHCA_" << m_scene_id << "_W" << WINDOW << "_" << std::setw(4) << std::setfill('0') << iter;
	std::string s = ss.str();

	filename += s;
	filename += ".txt";

	std::ofstream DATA;
	DATA.open(filename, std::ofstream::out);

	DATA << 'O' << '\n';
	int o = m_cc->NumObstacles() + m_agents.size();
	DATA << o << '\n';

	std::string movable = "False";
	auto obstacles = m_cc->GetObstacles();
	for (const auto& obs: *obstacles)
	{
		int id = obs.desc.id == m_ooi->GetID() ? 999 : obs.desc.id;
		DATA << id << ','
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

	movable = "True";
	for (const auto& a: m_agents)
	{
		State loc = a->CurrentState().state;
		auto agent_obj = a->GetObject();
		DATA << agent_obj->desc.id << ','
			<< agent_obj->Shape() << ','
			<< agent_obj->desc.type << ','
			<< loc.at(0) << ','
			<< loc.at(1) << ','
			<< agent_obj->desc.o_z << ','
			<< agent_obj->desc.o_roll << ','
			<< agent_obj->desc.o_pitch << ','
			<< agent_obj->desc.o_yaw << ','
			<< agent_obj->desc.x_size << ','
			<< agent_obj->desc.y_size << ','
			<< agent_obj->desc.z_size << ','
			<< agent_obj->desc.mass << ','
			<< agent_obj->desc.mu << ','
			<< movable << '\n';
	}

	// write solution trajs
	DATA << 'T' << '\n';
	o = m_agents.size();
	DATA << o << '\n';

	for (const auto& a: m_agents)
	{
		auto agent_obj = a->GetObject();
		auto move = a->MoveTraj();
		DATA << agent_obj->desc.id << '\n';
		DATA << move->size() << '\n';
		for (const auto& s: *move) {
			DATA << s.state.at(0) << ',' << s.state.at(1) << '\n';
		}
	}

	if (!ngr.empty())
	{
		DATA << "NGR" << '\n';
		DATA << ngr.size() << '\n';
		for (const auto& p: ngr) {
			DATA 	<< p.at(0) << ','
					<< p.at(1) << '\n';
		}
	}

	DATA.close();
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

bool Planner::Plan(bool& done)
{
	done = false;
	if (!m_replan) {
		return true;
	}

	if (FinalisePlan())
	{
		done = true;
		m_plan_success = true;
		if (m_cbs)
		{
			m_cbs->WriteRoot();
			m_cbs.reset();
		}
		return true;
	}

	bool backwards = true; // ALGO == MAPFAlgo::OURS;
	if (!setupProblem(backwards)) {
		return false;
	}

	// if (m_cbs) {
	// 	m_cbs->WriteRoot();
	// }
	m_timer = GetTime();
	if (!createCBS())
	{
		m_stats["mapf_time"] += GetTime() - m_timer;
		return false;
	}

	SMPL_INFO("Replan MAPF");
	bool result = m_cbs->Solve(backwards);
	m_cbs->UpdateStats(m_cbs_stats);
	m_stats["mapf_time"] += GetTime() - m_timer;

	m_moved = 0;
	m_cbs_soln_map.clear();
	m_alphas.clear();
	m_betas.clear();
	if (result)
	{
		m_cbs_soln = m_cbs->GetSolution();
		for (std::size_t i = 0; i < m_cbs_soln->m_solution.size(); ++i)
		{
			const auto& path = m_cbs_soln->m_solution[i];
			if (path.first == 0 || path.second.front().coord == path.second.back().coord)
			{
				// either this is robot or the object did not move
				continue;
			}
			m_cbs_soln_map[m_moved] = i;
			++m_moved;
		}

		if (m_cbs) {
			m_cbs->WriteRoot();
		}
		if (m_moved == 0)
		{
			m_plan_success = true;
		}
		else
		{
			m_alphas.resize(m_moved, 1.0);
			m_betas.resize(m_moved, 1.0);
		}
	}

	m_replan = !result;
	return result;
}

bool Planner::Rearrange()
{
	if (!rearrange())
	{
		SMPL_INFO("There were no conflicts to rearrange! WE ARE DONE!");

		m_timer = GetTime();

		std::vector<Object*> movable_obstacles;
		for (const auto& a: m_agents) {
			movable_obstacles.push_back(a->GetObject());
		}

		smpl::RobotState start_state = {};
		if (!m_rearrangements.empty()) {
			start_state = m_rearrangements.back().points.back().positions;
		}

		m_robot->ProcessObstacles({ m_ooi->GetObject() }, true);
		// if (!m_robot->PlanApproachOnly(movable_obstacles)) {
		if (!m_robot->PlanRetrieval(movable_obstacles, false, start_state))	{
			return false;
		}
		m_robot->ProcessObstacles({ m_ooi->GetObject() });
		m_exec = m_robot->GetLastPlanProfiled();

		m_stats["robot_planner_time"] += GetTime() - m_timer;

		return false;
	}

	return true;
}

std::uint32_t Planner::RunSim()
{
	m_timer = GetTime();
	if (!runSim()) {
		SMPL_ERROR("Simulation failed!");
	}
	m_stats["sim_time"] += GetTime() - m_timer;
	return m_violation;
}

bool Planner::TryExtract()
{
	m_timer = GetTime();

	if (!setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, armId(), m_ooi->GetID()))
	{
		m_stats["sim_time"] += GetTime() - m_timer;
		return false;
	}

	comms::ObjectsPoses rearranged = m_rearranged;
	bool success = true;
	if (!m_sim->ExecTraj(m_exec, rearranged, m_robot->GraspAt(), m_ooi->GetID()))
	{
		SMPL_ERROR("Failed to exec traj!");
		success = false;
	}

	m_sim->RemoveConstraint();

	m_stats["sim_time"] += GetTime() - m_timer;
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

int Planner::chooseObjDTS()
{
	std::vector<double> r(m_moved, 0);
	for (int i = 0; i < m_moved; ++i)
	{
		boost::math::beta_distribution<> dist(m_alphas.at(i), m_betas.at(i));
		r.at(i) = boost::math::quantile(dist, m_distD(m_rng));
	}

	return (int)std::distance(r.begin(), std::max_element(r.begin(), r.end()));
}

bool Planner::rearrange()
{
	if (m_moved == 0) {
		return false;
	}

	SMPL_INFO("Rearrange?");
	for (auto& a: m_agents) {
		a->ResetObject();
	}

	comms::ObjectsPoses rearranged = m_rearranged;
	bool push_found = false;
	int idx = chooseObjDTS();
	const auto& path = m_cbs_soln->m_solution[m_cbs_soln_map[idx]];

	// get push location
	std::vector<double> push;
	SMPL_INFO("Pushing object %d", path.first);
	m_timer = GetTime();
	m_agents.at(m_agent_map[path.first])->GetSE2Push(push, m_push_input);
	m_stats["push_input_time"] += GetTime() - m_timer;
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

	std::vector<double> push_start_state;
	if (!m_rearrangements.empty()) {
		push_start_state = m_rearrangements.back().points.back().positions;
	}

	double push_reward = 0.0;
	m_timer = GetTime();
	if (m_robot->PlanPush(push_start_state, m_agents.at(m_agent_map[path.first]).get(), push, movable_obstacles, rearranged, result, push_reward, m_push_input))
	{
		m_rearrangements.push_back(m_robot->GetLastPlan());

		// update positions of moved objects
		updateAgentPositions(result, rearranged);
		push_found = true;
		SMPL_INFO("Push found!");
	}
	m_stats["push_planner_time"] += GetTime() - m_timer;

	// if (push_reward > 0) {
	// 	m_alphas[idx] += push_reward;
	// }
	// if (push_reward < 0) {
	// 	m_betas[idx] -= push_reward;
	// }

	// if (m_alphas[idx] + m_betas[idx] > m_C)
	// {
	// 	double factor = m_C/double((m_C + std::abs(push_reward)));
	// 	m_alphas[idx] *= factor;
	// 	m_betas[idx] *= factor;
	// }

	m_rearranged = rearranged;
	m_replan = push_found;

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
	m_sim_success = m_violation == 0;

	return m_sim_success;
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

	// int result = cleanupLogs();
	// if (result == -1) {
	// 	SMPL_ERROR("system command errored!");
	// }

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

bool Planner::SaveData()
{
	auto robot_stats = m_robot->GetStats();

	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/PLANNER.csv";

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "PlanSuccess,SimSuccess,SimResult,SimTime,"
				<< "RobotPlanTime,MAPFTime,PushInputTime,PlanPushTime,"
				<< "PlanPushCalls,PushSamplesFound,PushActionsFound,"
				<< "PushPlanningTime,PushSimTime,PushSimSuccesses,"
				<< "CBSCalls,CBSSuccesses,CBSTime,"
				<< "CTNodes,CTDeadends,CTExpansions,LLTime,ConflictDetectionTime\n";
	}

	STATS << m_scene_id << ','
			<< m_plan_success << ',' << m_sim_success << ','
			<< m_violation << ',' << m_stats["sim_time"] << ','
			<< m_stats["robot_planner_time"] << ',' << m_stats["mapf_time"] << ','
			<< m_stats["push_input_time"] << ',' << m_stats["push_planner_time"] << ','
			<< robot_stats["plan_push_calls"] << ',' << robot_stats["push_samples_found"] << ','
			<< robot_stats["push_actions_found"] << ',' << robot_stats["push_plan_time"] << ','
			<< robot_stats["push_sim_time"] << ',' << robot_stats["push_sim_successes"] << ','
			<< m_cbs_stats["calls"] << ',' << m_cbs_stats["solved"] << ','
			<< m_cbs_stats["search_time"] << ',' << m_cbs_stats["ct_nodes"] << ','
			<< m_cbs_stats["ct_deadends"] << ',' << m_cbs_stats["ct_expanded"] << ','
			<< m_cbs_stats["ll_time"] << ',' << m_cbs_stats["conflict_time"] << '\n';
	STATS.close();
}

} // namespace clutter
