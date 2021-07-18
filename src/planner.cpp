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

namespace clutter
{

Planner::Planner(const std::string& scene_file)
:
m_scene_file(scene_file),
m_num_agents(-1),
m_ooi_idx(-1),
m_t(0)
{
	m_agents.clear();

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
			if (itr->id == 6) {
				fend = itr;
				break;
			}
		}
		obstacles.erase(fbegin, fend);
	}

	m_cc = std::make_shared<CollisionChecker>(this, obstacles);
	obstacles.clear();

	m_ooi.SetCC(m_cc);
	for (auto& a: m_agents) {
		a.SetCC(m_cc);
	}

	// Set agent current positions and time
	m_ooi.Init();
	for (auto& a: m_agents) {
		a.Init();
	}

	// Get OOI goal
	m_ooi_gf = m_cc->GetGoalState(m_ooi.GetObject());
	ContToDisc(m_ooi_gf, m_ooi_g);
}

void Planner::WHCAStar()
{
	double start_time = GetTime();

	int iter = 0;
	writePlanState(iter);

	reinit();
	while (!m_ooi.AtGoal(*(m_ooi.GetCurrentState()), false))
	{
		// SMPL_INFO("Plan for OOI (number %d, id %d, priority %d)", 0, m_ooi.GetObject()->id, 0);
		m_ooi.Search(0); // updates cc with found plan
		int robin = 1;
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
	double plan_time = end_time - start_time;
	SMPL_INFO("WHCA* planning took %f seconds.", plan_time);
}

const Object* Planner::GetObject(int priority)
{
	if (priority == 0) {
		return m_ooi.GetObject();
	}
	else {
		return m_agents.at(m_priorities.at(priority-1)).GetObject();
	}
}

void Planner::reinit()
{
	// reset searches
	m_ooi.reset();
	for (auto& a: m_agents) {
		a.reset();
	}

	// Set agent starts
	m_ooi.SetStartState(*(m_ooi.GetCurrentState()));
	for (auto& a: m_agents) {
		a.SetStartState(*(a.GetCurrentState()));
	}

	// Set agent goals
	m_ooi.SetGoalState(m_ooi_g);
	for (auto& a: m_agents) {
		a.SetGoalState(a.GetCurrentState()->p);
	}

	// Set agent priorities
	prioritize();
}

void Planner::prioritize()
{
	m_priorities.clear();
	m_priorities.resize(m_agents.size());
	std::iota(m_priorities.begin(), m_priorities.end(), 0);

	Pointf extract_pf, agent_pf;
	DiscToCont(m_ooi.GetCurrentState()->p, extract_pf);

	std::vector<float> dists(m_agents.size(), 0.0);
	for (size_t i = 0; i < m_agents.size(); ++i) {
		DiscToCont(m_agents.at(i).GetCurrentState()->p, agent_pf);
		dists.at(i) = EuclideanDist(extract_pf, agent_pf);
	}
	std::stable_sort(m_priorities.begin(), m_priorities.end(),
		[&dists](size_t i1, size_t i2) { return dists[i1] < dists[i2]; });
}

void Planner::step_agents(int k)
{
	// TODO: make sure we can handle k > 1
	m_ooi.Step(k);
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
	int o = m_cc->NumObstacles() + 1 + m_agents.size();
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

	Pointf loc;
	DiscToCont(m_ooi.GetCurrentState()->p, loc);
	auto agent_obs = m_ooi.GetObject();
	movable = "False";
	DATA << agent_obs->id << ','
			<< agent_obs->shape << ','
			<< agent_obs->type << ','
			<< loc.x << ','
			<< loc.y << ','
			<< agent_obs->o_z << ','
			<< agent_obs->o_roll << ','
			<< agent_obs->o_pitch << ','
			<< agent_obs->o_yaw << ','
			<< agent_obs->x_size << ','
			<< agent_obs->y_size << ','
			<< agent_obs->z_size << ','
			<< agent_obs->mass << ','
			<< agent_obs->mu << ','
			<< movable << '\n';

	for (const auto& a: m_agents)
	{
		DiscToCont(a.GetCurrentState()->p, loc);
		agent_obs = a.GetObject();
		movable = "True";
		DATA << agent_obs->id << ','
			<< agent_obs->shape << ','
			<< agent_obs->type << ','
			<< loc.x << ','
			<< loc.y << ','
			<< agent_obs->o_z << ','
			<< agent_obs->o_roll << ','
			<< agent_obs->o_pitch << ','
			<< agent_obs->o_yaw << ','
			<< agent_obs->x_size << ','
			<< agent_obs->y_size << ','
			<< agent_obs->z_size << ','
			<< agent_obs->mass << ','
			<< agent_obs->mu << ','
			<< movable << '\n';
	}

	DATA.close();
}

} // namespace clutter
