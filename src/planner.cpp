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

Planner::Planner(const std::string& scene_file, int scene_id)
:
m_scene_file(scene_file),
m_num_agents(-1),
m_ooi_idx(-1),
m_t(0),
m_phase(0),
m_scene_id(scene_id),
m_ph("~")
{
	setupGlobals();

	m_agents.clear();
	std::vector<Object> all_obstacles, pruned_obstacles;
	parse_scene(all_obstacles);
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

	m_robot = std::make_unique<Robot>();

	m_ooi.SetCC(m_cc);
	m_robot->SetCC(m_cc);
	for (auto& a: m_agents) {
		a.SetCC(m_cc);
	}

	m_ooi.Setup();
	if (!m_robot->Setup()) {
		SMPL_ERROR("Robot setup failed!");
	}
	for (auto& a: m_agents) {
		a.Setup();
	}

	if (!m_robot->ProcessObstacles(all_obstacles)) {
		SMPL_ERROR("Robot collision space setup failed!");
	}
	all_obstacles.clear();

	while (!setupProblem()) {
		continue;
	}

	m_simsrv = m_nh.advertiseService("run_sim", &Planner::runSim, this);
	m_animsrv = m_nh.advertiseService("anim_soln", &Planner::animateSolution, this);
}

void Planner::Plan()
{
	int runs = 1;

	double start_time = GetTime();
	while (!whcastar())
	{
		// SMPL_WARN("Some agent search failed. Must solve again!");
		m_t = 0;
		m_phase = 0;

		while (!setupProblem(true)) {
			continue;
		}

		int res = cleanupLogs();
		if (res == -1) {
			SMPL_ERROR("system command errored!");
		}

		++runs;
	}
	double time_taken = GetTime() - start_time;
	SMPL_INFO("Planning took %f seconds. (%d runs)", time_taken, runs);

	m_cc->PrintConflicts();

	auto robot_traj = m_robot->GetMoveTraj();
	m_exec.insert(m_exec.begin(), robot_traj->begin(), robot_traj->end());
	m_robot->ProfileTraj(m_exec);
}

bool Planner::whcastar()
{
	// ProfilerStart("/home/dhruv/test3.out");

	double start_time = GetTime(), total_time = 0.0, iter_time = 0.0;

	int iter = 0;
	writePlanState(iter);

	reinit();
	while (!m_robot->AtGoal(*(m_robot->GetCurrentState()), false)) // robot (current) state includes joint config to avoid IK
	{
		if (!m_ooi.Search(0)) {
			return false;
		}
		if (!m_robot->Search(1)) {
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
		writePlanState(iter);

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
	while (!m_robot->AtGoal(*(m_robot->GetCurrentState()), false))
	{
		if (!m_ooi.Search(0)) {
			return false;
		}
		if (!m_robot->Search(1)) {
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
		writePlanState(iter);

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

bool Planner::animateSolution(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	m_robot->AnimateSolution();
	return true;
}

bool Planner::runSim(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	m_sim = std::make_unique<BulletSim>(
				std::string(), false,
				m_scene_id, std::string(),
				-1, -1);

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

	if (!m_sim->SetColours())
	{
		ROS_ERROR("Failed to set object colours in scene!");
		return false;
	}

	moveit_msgs::RobotTrajectory to_exec;
	m_robot->ConvertTraj(m_exec, to_exec);
	if (!m_sim->ExecTraj(to_exec.joint_trajectory))
	{
		ROS_ERROR("Failed to exec traj!");
		return false;
	}

	return true;
}

const std::vector<Object>* Planner::GetObject(const LatticeState& s, int priority)
{
	if (priority == 0) {
		return m_ooi.GetObject(s);
	}
	else if (priority == 1) {
		return m_robot->GetObject(s);
	}
	else {
		return m_agents.at(m_priorities.at(priority-2)).GetObject(s);
	}
}

void Planner::reinit()
{
	// reset searches
	m_ooi.reset(m_phase);
	m_robot->reset(m_phase);
	for (auto& a: m_agents) {
		a.reset(m_phase);
	}

	// Set agent starts - always the current state
	m_ooi.SetStartState(*(m_ooi.GetCurrentState()));
	m_robot->SetStartState(*(m_robot->GetCurrentState()));
	for (auto& a: m_agents) {
		a.SetStartState(*(a.GetCurrentState()));
	}

	// Set agent goals
	if (m_phase == 0)
	{
		m_ooi.SetGoalState(m_ooi.GetCurrentState()->coord);
		m_robot->SetGoalState(m_ooi.GetCurrentState()->coord);
	}
	else if (m_phase == 1)
	{
		m_ooi.SetGoalState(m_robot->GetEECoord()); // EE position
		m_robot->SetGoalState(m_ooi_g);
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

bool Planner::setupProblem(bool random)
{
	// Set agent current positions and time
	m_ooi.Init();
	m_robot->Init();
	if (random) {
		if (!m_robot->RandomiseStart()) {
			return false;
		}
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
				m_num_agents = std::stoi(line);
				m_agents.clear();
				obstacles.clear();

				for (int i = 0; i < m_num_agents; ++i)
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
						count++;
					}
					if (o.movable) {
						Agent r(o);
						m_agents.push_back(std::move(r));
					}
					else {
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

				break; // do not care about reading anything that follows
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
				<< obs.shape << ','
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
			<< agent_obs->back().shape << ','
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
			<< agent_obs->back().shape << ','
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
				<< robot_o.shape << ','
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

} // namespace clutter
