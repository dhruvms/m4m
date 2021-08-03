#include <pushplan/planner.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/constants.hpp>
#include <pushplan/geometry.hpp>

#include <smpl/console/console.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <iomanip>

#include <gperftools/profiler.h>

namespace clutter
{

Planner::Planner(const std::string& scene_file)
:
m_scene_file(scene_file),
m_num_agents(-1),
m_ooi_idx(-1),
m_t(0),
m_phase(0),
m_ph("~")
{
	setupGlobals();

	m_agents.clear();
	m_robot = std::make_unique<Robot>();

	std::vector<Object> obstacles;
	parse_scene(obstacles);

	// only keep the base of the fridge shelf
	if (FRIDGE)
	{
		auto fbegin = obstacles.begin();
		auto fend = obstacles.end();
		for (auto itr = obstacles.begin(); itr != obstacles.end(); ++itr)
		{
			if (itr->id == 2) {
				fbegin = itr;
			}
			if (itr->id > 5) {
				fend = itr;
				break;
			}
		}
		obstacles.erase(fbegin, fend);
	}

	m_cc = std::make_shared<CollisionChecker>(this, obstacles);
	m_cc->SetPhase(m_phase);
	obstacles.clear();

	// Get OOI goal
	m_ooi_gf = m_cc->GetGoalState(&m_ooi.GetObject()->back());
	ContToDisc(m_ooi_gf, m_ooi_g);

	m_ooi.SetCC(m_cc);
	m_robot->SetCC(m_cc);
	for (auto& a: m_agents) {
		a.SetCC(m_cc);
	}

	// Set agent current positions and time
	m_ooi.Init();
	m_robot->Init();
	for (auto& a: m_agents) {
		a.Init();
	}
}

void Planner::WHCAStar()
{
	// ProfilerStart("/home/dhruv/test3.out");

	double start_time = GetTime(), total_time = 0.0;

	int iter = 0;
	writePlanState(iter);

	reinit();
	while (!m_robot->AtGoal(*(m_robot->GetCurrentState()), false)) // robot (current) state includes joint config to avoid IK
	{
		// SMPL_INFO("Plan for OOI (number %d, id %d, priority %d)", 0, m_ooi.GetObject()->id, 0);
		// Search() updates cc with found plan
		m_ooi.Search(0);
		m_robot->Search(1);
		int robin = 2;
		for (const auto& p: m_priorities) {
			// SMPL_INFO("Plan for Object (number %d, id %d, priority %d)", p, m_agents.at(p).GetObject()->id, robin);
			m_agents.at(p).Search(robin);
			++robin;
		}

		step_agents(); // take 1 step by default
		reinit();

		++iter;
		writePlanState(iter);
	}
	double end_time = GetTime();
	double phase_time = end_time - start_time;
	total_time += phase_time;
	SMPL_INFO("WHCA* Phase 1 planning took %f seconds.", phase_time);

	// ProfilerStop();

	m_phase = 1;
	m_cc->SetPhase(m_phase);

	start_time = GetTime();
	reinit();
	while (!m_robot->AtGoal(*(m_robot->GetCurrentState()), false))
	{
		// SMPL_INFO("Plan for OOI (number %d, id %d, priority %d)", 0, m_ooi.GetObject()->id, 0);
		m_ooi.Search(0);
		m_robot->Search(1);
		int robin = 2;
		for (const auto& p: m_priorities) {
			// SMPL_INFO("Plan for Object (number %d, id %d, priority %d)", p, m_agents.at(p).GetObject()->id, robin);
			m_agents.at(p).Search(robin);
			++robin;
		}

		step_agents();
		reinit();

		++iter;
		writePlanState(iter);
	}
	end_time = GetTime();
	phase_time = end_time - start_time;
	total_time += phase_time;
	SMPL_INFO("WHCA* Phase 2 planning took %f seconds. Total time = %f seconds.", phase_time, total_time);
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
		m_ooi.SetGoalState(m_robot->GetCurrentState()->coord); // EE position
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

void Planner::set_ee_obj()
{
	Object o;
	o.id = 99;
	o.shape = 2; // circle
	o.type = 1; // movable
	o.o_x = 0.310248; // from start config FK
	o.o_y = -0.673113; // from start config FK
	// o.o_x = m_ooi_gf.x;
	// o.o_y = m_ooi_gf.y;
	o.o_z = m_ooi.GetObject()->back().o_z; // hand picked for now
	o.o_roll = 0.0;
	o.o_pitch = 0.0;
	o.o_yaw = 0.0;
	o.x_size = 0.04; // hand picked for now
	o.y_size = 0.04; // hand picked for now
	o.z_size = 0.03; // hand picked for now
	o.mass = 0.58007; // r_gripper_palm_joint mass from URDF
	o.locked = o.mass == 0;
	o.mu = m_ooi.GetObject()->back().mu; // hand picked for now
	o.movable = true;

	m_ee.SetObject(o);
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

	DATA.close();
}

void Planner::setupGlobals()
{
	m_ph.getParam("fridge", FRIDGE);
	m_ph.getParam("allowed_planning_time", MAX_PLANNING_TIME);
	m_ph.getParam("whca/window", WINDOW);
	m_ph.getParam("whca/goal_thresh", GOAL_THRESH);
	m_ph.getParam("whca/res", RES);
	m_ph.getParam("whca/grid", GRID);
	m_ph.getParam("whca/cost_mult", COST_MULT);
	m_ph.getParam("robot/semi_minor", SEMI_MINOR);
	m_ph.getParam("robot/robot_obj_mass", R_MASS);
}

} // namespace clutter
