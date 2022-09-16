#include <pushplan/utils/bullet_sim.hpp>
#include <pushplan/utils/constants.hpp>
#include <comms/ResetSimulation.h>
#include <comms/AddObject.h>
#include <comms/AddYCBObject.h>
#include <comms/AddRobot.h>
#include <comms/SetRobotState.h>
#include <comms/ResetArm.h>
#include <comms/CheckScene.h>
#include <comms/ResetScene.h>
#include <comms/SetColours.h>
#include <comms/ExecTraj.h>
#include <comms/SimPushes.h>

#include <smpl/angles.h>
#include <smpl/console/console.h>

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace clutter
{

BulletSim::BulletSim(
	const std::string& tables, bool ycb,
	int replay_id, const std::string& suffix,
	int immovable_objs,
	int movable_objs) :
m_num_immov(immovable_objs), m_num_mov(movable_objs),
m_robot_id(-1), m_tables(-1),
m_rng(m_dev())
{
	setupServices();

	if (!resetSimulation())
	{
		ROS_ERROR("Failed to reset the simulation.");
	}

	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
	m_distI = std::uniform_int_distribution<>(0, 2);

	if (replay_id < 0)
	{
		// load objects from tables file into scene
		if (!readTables(tables)) {
			ROS_ERROR("Failed to read tables file properly. Exiting.\n");
		}
		if (!setupTables()) {
			ROS_ERROR("Failed to setup the scene.");
		}
		if(!setupObjects(ycb)) {
			ROS_ERROR("Failed to load %d objects into the scene.", m_num_immov);
		}
	}
	else
	{
		m_num_mov = -1;
		m_num_immov = -1;

		bool fridge;
		m_nh.getParam("/fridge", fridge);
		m_tables = fridge ? 5 : 1;
		setupObjectsFromFile(ycb, replay_id, suffix);
		m_num_immov -= m_tables;
	}

	// load robot into sim
	if (!addRobot(replay_id, suffix)) {
		ROS_ERROR("Failed to add robot.");
	}
}

bool BulletSim::SetRobotState(const sensor_msgs::JointState& msg)
{
	comms::SetRobotState srv;
	srv.request.state = msg;

	if (!m_services.at(m_servicemap["set_robot_state"]).call(srv))
	{
		ROS_ERROR("Failed to set robot start state.");
		return false;
	}

	return true;
}


bool BulletSim::ResetArm(const int& arm)
{
	comms::ResetArm srv;
	srv.request.arm = arm;

	if (!m_services.at(m_servicemap["reset_arm"]).call(srv))
	{
		ROS_ERROR("Failed to reset arm %d.", arm);
		return false;
	}

	return true;
}

bool BulletSim::CheckScene(const int& arm, int& count)
{
	count = 0;

	comms::CheckScene srv;
	srv.request.arm = arm;
	srv.request.sim_id = -1; // add to all simulator instances

	if (!m_services.at(m_servicemap["check_scene"]).call(srv))
	{
		ROS_ERROR("Failed to check scene for arm %d.", arm);
		return false;
	}

	if (srv.response.ids.size() > 0)
	{
		if (srv.response.ids.front() == -1) {
			return false;
		}

		count = srv.response.ids.size();
		std::size_t newsize = m_removed.size() + srv.response.ids.size();
		m_removed.reserve(newsize);
		m_removed_ids.reserve(newsize);

		for (const auto& e : srv.response.ids)
		{
			if (!removeObject(e)) {
				ROS_ERROR("Could not remove object %d.", e);
			}
		}
	}

	// scratch planning if any objects removed
	if (count > 0) {
		return false;
	}

	return true;
}

bool BulletSim::ResetScene()
{
	comms::ResetScene srv;
	srv.request.req = true;
	srv.request.sim_id = -1; // add to all simulator instances

	if (!m_services.at(m_servicemap["reset_scene"]).call(srv))
	{
		ROS_ERROR("Failed to reset objects in the scene.");
		return false;
	}

	return true;
}

bool BulletSim::SetColours(int ooi)
{
	comms::SetColours srv;
	srv.request.sim_id = -1; // add to all simulator instances

	for (auto itr = m_immov_ids.begin();
				itr != m_immov_ids.end(); ++itr)
	{
		if (itr->first == m_robot_id) {
			continue;
		}

		else if (itr->first <= m_tables) // tables
		{
			srv.request.ids.push_back(itr->first);
			srv.request.type.push_back(-1); // tables
		}

		else if (itr->first == ooi) // tables
		{
			srv.request.ids.push_back(itr->first);
			srv.request.type.push_back(999); // tables
		}

		else
		{
			srv.request.ids.push_back(itr->first);
			srv.request.type.push_back(0); // immovable
		}
	}

	for (auto itr = m_mov_ids.begin();
				itr != m_mov_ids.end(); ++itr)
	{
		srv.request.ids.push_back(itr->first);
		srv.request.type.push_back(1); // movable
	}

	if (!m_services.at(m_servicemap["set_colours"]).call(srv))
	{
		ROS_ERROR("Failed to set colours in the scene.");
		return false;
	}

	return true;
}

bool BulletSim::ExecTraj(
	const trajectory_msgs::JointTrajectory& traj,
	const comms::ObjectsPoses& rearranged,
	int grasp_at, int ooi)
{
	comms::ExecTraj srv;
	srv.request.traj = traj;
	srv.request.objects = rearranged;
	srv.request.grasp_at = grasp_at;
	srv.request.ooi = ooi;

	if (!m_services.at(m_servicemap["exec_traj"]).call(srv))
	{
		ROS_ERROR("Failed to execute trajector in sim.");
		return false;
	}

	std::stringstream ss;
	for (const auto& id: srv.response.interactions) {
		ss << std::setw(3) << id << ",";
	}
	std::string s = ss.str();
	SMPL_INFO("Interactions: (%s)", s.c_str());
	if (srv.response.violation)
	{
		auto itr = EXEC_TRAJ_FAIL.find(srv.response.cause);
		if (itr != EXEC_TRAJ_FAIL.end()) {
			SMPL_ERROR("Exec Traj error message: %s", itr->second.c_str());
		}
	}

	return !srv.response.violation;
}

bool BulletSim::SimPushes(
	const std::vector<trajectory_msgs::JointTrajectory>& pushes,
	int oid, float gx, float gy,
	int& pidx, int& successes,
	const comms::ObjectsPoses& rearranged,
	comms::ObjectsPoses& result)
{
	comms::SimPushes srv;
	srv.request.pushes = pushes;
	srv.request.oid = oid;
	srv.request.gx = gx;
	srv.request.gy = gy;
	srv.request.objects = rearranged;

	if (!m_services.at(m_servicemap["sim_pushes"]).call(srv))
	{
		ROS_ERROR("Failed to execute trajector in sim.");
		return false;
	}

	pidx = srv.response.idx;
	successes = srv.response.successes;
	result = srv.response.objects;

	return srv.response.res;
}

bool BulletSim::RemoveConstraint()
{
	comms::ResetSimulation srv;
	srv.request.req = true;

	return m_services.at(m_servicemap["remove_constraint"]).call(srv);
}

bool BulletSim::setupObjects(bool ycb)
{
	if (ycb) {
		return setupYCBObjects();
	}
	else {
		return setupPrimitiveObjects();
	}
}

bool BulletSim::setupObjectsFromFile(
	bool ycb, int replay_id, const std::string& suffix)
{
	if (ycb) {
		return setupYCBObjectsFromFile(replay_id, suffix);
	}
	else {
		return setupPrimitiveObjectsFromFile(replay_id, suffix);
	}
}

bool BulletSim::addRobot(int replay_id, const std::string& suffix)
{
	comms::AddRobot srv;

	if (replay_id < 0)
	{
		m_robot.resize(7);
		m_nh.param("robot/start/x", m_robot[0], 0.0);
		m_nh.param("robot/start/y", m_robot[1], 0.0);
		m_nh.param("robot/start/z", m_robot[2], 0.0);
		m_nh.param("robot/start/qx", m_robot[3], 0.0);
		m_nh.param("robot/start/qy", m_robot[4], 0.0);
		m_nh.param("robot/start/qz", m_robot[5], 0.0);
		m_nh.param("robot/start/qw", m_robot[6], 1.0);
	}
	else
	{
		m_robot.reserve(7);
		readRobotFromFile(replay_id, suffix);
	}

	srv.request.x = m_robot[0];
	srv.request.y = m_robot[1];
	srv.request.z = m_robot[2];
	srv.request.qx = m_robot[3];
	srv.request.qy = m_robot[4];
	srv.request.qz = m_robot[5];
	srv.request.qw = m_robot[6];
	srv.request.sim_id = -1; // add to all simulator instances

	if (m_services.at(m_servicemap["add_robot"]).call(srv))
	{
		m_robot_id = srv.response.id;
		return true;
	}

	return false;
}

bool BulletSim::setupPrimitiveObjectsFromFile(int replay_id, const std::string& suffix)
{
	std::string filename = getPartialFilename(replay_id);
	if (!suffix.empty())
	{
		filename.insert(filename.find_last_of('.'), "_");
		filename.insert(filename.find_last_of('.'), suffix);
	}

	std::ifstream DATA;
	DATA.open(filename);

	if (DATA.is_open())
	{
		std::string line;
		while (!DATA.eof())
		{
			getline(DATA, line);
			if (line.compare("O") == 0)
			{
				getline(DATA, line);
				int num_objects = std::stoi(line);
				for (int i = 0; i < num_objects; ++i)
				{
					getline(DATA, line);
					if ((m_num_mov + m_num_immov) == num_objects)
						continue;

					std::stringstream ss(line);
					std::string split;

					int count = 0;
					comms::AddObject srv;
					while (ss.good())
					{
						getline(ss, split, ',');
						switch (count) {
							case 0: break;
							case 1: srv.request.shape = std::stoi(split); break;
							case 2: break;
							case 3: srv.request.o_x = std::stod(split); break;
							case 4: srv.request.o_y = std::stod(split); break;
							case 5: srv.request.o_z = std::stod(split); break;
							case 6: srv.request.o_r = std::stod(split); break;
							case 7: srv.request.o_p = std::stod(split); break;
							case 8: srv.request.o_yaw = std::stod(split); break;
							case 9: srv.request.x_size = std::stod(split); break;
							case 10: srv.request.y_size = std::stod(split); break;
							case 11: srv.request.z_size = std::stod(split); break;
							case 12: {
								srv.request.mass = std::stod(split);
								srv.request.locked = (std::stod(split) == 0);
								break;
							}
							case 13: srv.request.mu = std::stod(split); break;
							case 14: srv.request.movable = (split.compare("True") == 0); break;
						}
						count++;
					}

					srv.request.sim_id = -1; // add to all simulator instances

					if (m_services.at(m_servicemap["add_object"]).call(srv))
					{
						if (srv.response.id != -1)
						{
							std::vector<double> data{	srv.request.o_x,
														srv.request.o_y,
														srv.request.o_z,
														srv.request.o_r,
														srv.request.o_p,
														srv.request.o_yaw,
														srv.request.x_size,
														srv.request.y_size,
														srv.request.z_size,
														(double)srv.request.locked,
														(double)srv.request.movable,
														0.0 // YCB flag for primitive object = false
													};
							if (srv.request.movable)
							{
								m_mov_ids.push_back(
									std::make_pair(srv.response.id, srv.request.shape));
								m_mov.push_back(data);
							}
							else
							{
								m_immov_ids.push_back(
									std::make_pair(srv.response.id, srv.request.shape));
								m_immov.push_back(data);
							}
						}
					}
				}
				m_num_immov = m_immov.size();
				m_num_mov = m_mov.size();
			}
		}
	}

	DATA.close();
	return true;
}

bool BulletSim::setupYCBObjectsFromFile(int replay_id, const std::string& suffix)
{
	setupTableFromFile(replay_id, suffix);

	std::string filename = getPartialFilename(replay_id);
	if (!suffix.empty())
	{
		filename.insert(filename.find_last_of('.'), "_");
		filename.insert(filename.find_last_of('.'), suffix);
	}

	std::ifstream DATA;
	DATA.open(filename);

	if (DATA.is_open())
	{
		std::string line;
		while (!DATA.eof())
		{
			getline(DATA, line);
			if (line.compare("O") == 0)
			{
				getline(DATA, line);
				int num_objects = std::stoi(line);

				for (int i = 0; i < m_tables; ++i) {
					getline(DATA, line);
				}
				for (int i = 0; i < num_objects - m_tables; ++i)
				{
					getline(DATA, line);

					if ((m_num_mov + m_num_immov) == num_objects)
						continue;

					std::stringstream ss(line);
					std::string split;

					int count = 0;
					comms::AddYCBObject srv;
					while (ss.good())
					{
						getline(ss, split, ',');
						switch (count) {
							case 0: break;
							case 1: srv.request.obj_id = std::stoi(split); break;
							case 2: break;
							case 3: srv.request.o_x = std::stod(split); break;
							case 4: srv.request.o_y = std::stod(split); break;
							case 5: srv.request.o_z = std::stod(split); break;
							case 6: srv.request.o_r = std::stod(split); break;
							case 7: srv.request.o_p = std::stod(split); break;
							case 8: srv.request.o_yaw = std::stod(split); break;
							case 9: srv.request.mu = std::stod(split); break;
							case 10: srv.request.movable = (split.compare("True") == 0); break;
						}
						count++;
					}
					srv.request.sim_id = -1; // add to all simulator instances
					if (m_services.at(m_servicemap["add_ycb_object"]).call(srv))
					{
						if (srv.response.id != -1)
						{
							std::vector<double> data{	srv.request.o_x,
														srv.request.o_y,
														srv.request.o_z,
														srv.request.o_r,
														srv.request.o_p,
														srv.request.o_yaw,
														(double)srv.request.movable,
														1.0 // YCB flag for YCB object = true
													};

							if (srv.request.movable)
							{
								m_mov_ids.push_back(
									std::make_pair(srv.response.id, srv.request.obj_id));
								m_mov.push_back(data);
							}
							else
							{
								m_immov_ids.push_back(
									std::make_pair(srv.response.id, srv.request.obj_id));
								m_immov.push_back(data);
							}
						}
					}
				}
				m_num_immov = m_immov.size();
				m_num_mov = m_mov.size();
				break;
			}
		}
	}

	DATA.close();
	return true;
}

void BulletSim::setupTableFromFile(int replay_id, const std::string& suffix)
{
	std::string filename = getPartialFilename(replay_id);
	if (!suffix.empty())
	{
		filename.insert(filename.find_last_of('.'), "_");
		filename.insert(filename.find_last_of('.'), suffix);
	}

	std::ifstream DATA;
	DATA.open(filename);

	if (DATA.is_open())
	{
		std::string line;
		while (!DATA.eof())
		{
			getline(DATA, line);
			if (line.compare("O") == 0)
			{
				getline(DATA, line);
				int num_objects = std::stoi(line);
				for (int i = 0; i < m_tables; ++i)
				{
					getline(DATA, line);

					if ((m_num_mov + m_num_immov) == num_objects)
						continue;

					std::stringstream ss(line);
					std::string split;

					int count = 0;
					comms::AddObject srv;
					while (ss.good())
					{
						getline(ss, split, ',');
						switch (count) {
							case 0: break;
							case 1: srv.request.shape = std::stoi(split); break;
							case 2: break;
							case 3: srv.request.o_x = std::stod(split); break;
							case 4: srv.request.o_y = std::stod(split); break;
							case 5: srv.request.o_z = std::stod(split); break;
							case 6: srv.request.o_r = std::stod(split); break;
							case 7: srv.request.o_p = std::stod(split); break;
							case 8: srv.request.o_yaw = std::stod(split); break;
							case 9: srv.request.x_size = std::stod(split); break;
							case 10: srv.request.y_size = std::stod(split); break;
							case 11: srv.request.z_size = std::stod(split); break;
							case 12: {
								srv.request.mass = 0.0;
								srv.request.locked = true;
								break;
							}
							case 13: srv.request.mu = std::stod(split); break;
							case 14: srv.request.movable = false; break;
						}
						count++;
					}

					srv.request.sim_id = -1; // add to all simulator instances

					if (m_services.at(m_servicemap["add_object"]).call(srv))
					{
						if (srv.response.id != -1)
						{
							m_immov_ids.push_back(
								std::make_pair(srv.response.id, srv.request.shape));

							std::vector<double> data{	srv.request.o_x,
														srv.request.o_y,
														srv.request.o_z,
														srv.request.o_r,
														srv.request.o_p,
														srv.request.o_yaw,
														srv.request.x_size,
														srv.request.y_size,
														srv.request.z_size,
														(double)srv.request.locked,
														(double)srv.request.movable,
														0.0 // YCB flag for primitive object = false
													};
							m_immov.push_back(data);
						}
					}
				}
				m_num_immov = m_immov.size();
				m_num_mov = m_mov.size();
				break;
			}
		}
	}

	DATA.close();
}

bool BulletSim::setupPrimitiveObjects()
{
	int immov_added = 0, mov_added = 0;
	double min_size, xlim, ylim, zlim;
	m_nh.param("objects/min_size", min_size, 0.0);
	m_nh.param("objects/x_size", xlim, 1.0);
	m_nh.param("objects/y_size", ylim, 1.0);
	m_nh.param("objects/z_size", zlim, 1.0);

	comms::AddObject srv;
	while (immov_added < m_num_immov || mov_added < m_num_mov)
	{
		srv.request.shape = m_distI(m_rng);
		// DO NOT add spheres to the scene
		// TODO: deal with spheres
		if (srv.request.shape == 1) {
			continue;
		}

		srv.request.x_size = std::max(min_size, m_distD(m_rng) * xlim);
		srv.request.y_size = std::max(min_size, m_distD(m_rng) * ylim);
		srv.request.z_size = std::max(min_size * (3.0 + 2 * int(srv.request.shape == 2)), m_distD(m_rng) * zlim);
		if (srv.request.shape == 2) { // cylinder
			srv.request.y_size = srv.request.x_size;
		}

		srv.request.o_x = getObjectCoord(	m_immov.at(0)[0],
											m_immov.at(0)[6],
											srv.request.x_size);
		srv.request.o_y = getObjectCoord(	m_immov.at(0)[1],
											m_immov.at(0)[7],
											srv.request.y_size);
		srv.request.o_z = (m_immov.at(0)[2] + m_immov.at(0)[8])	+ srv.request.z_size;
		if (srv.request.shape == 2)	{
			srv.request.o_z -= srv.request.z_size/2;
		}

		srv.request.o_r = 0.0;
		srv.request.o_p = 0.0;
		srv.request.o_yaw = m_distD(m_rng) * 2*M_PI;

		srv.request.locked = false;
		if (immov_added < m_num_immov) {
			srv.request.movable = false;
		}
		else {
			srv.request.movable = true;
		}
		srv.request.mu = -1; // set in simulator
		srv.request.sim_id = -1; // add to all simulator instances

		if (m_services.at(m_servicemap["add_object"]).call(srv))
		{
			if (srv.response.id != -1)
			{
				std::vector<double> data{	srv.request.o_x,
											srv.request.o_y,
											srv.request.o_z,
											srv.request.o_r,
											srv.request.o_p,
											srv.request.o_yaw,
											srv.request.x_size,
											srv.request.y_size,
											srv.request.z_size,
											(double)srv.request.locked,
											(double)srv.request.movable,
											0.0 // flag for primitive object = false
										};
				if (immov_added < m_num_immov)
				{
					m_immov_ids.push_back(
						std::make_pair(srv.response.id, srv.request.shape));
					m_immov.push_back(data);
					immov_added++;
				}
				else
				{
					m_mov_ids.push_back(
						std::make_pair(srv.response.id, srv.request.shape));
					m_mov.push_back(data);
					mov_added++;
				}
			}

			if (!srv.response.removes.empty())
			{
				std::size_t newsize = m_removed.size() +
												srv.response.removes.size();
				m_removed.reserve(newsize);
				m_removed_ids.reserve(newsize);

				for (const auto& e : srv.response.removes)
				{
					if (!removeObject(e)) {
						ROS_ERROR("Could not remove object %d.", e);
					}
				}
			}
		}
	}

	return true;
}

bool BulletSim::setupYCBObjects()
{
	std::vector<int> ycb_objs(YCB_OBJECTS);
	std::random_shuffle(ycb_objs.begin(), ycb_objs.end());

	int N = ycb_objs.size();
	int immov_added = 0, mov_added = 0, total_added = 0;

	comms::AddYCBObject srv;
	while (immov_added < m_num_immov || mov_added < m_num_mov)
	{
		srv.request.sim_id = -1; // add to all simulator instances
		srv.request.obj_id = ycb_objs.at(total_added % N);

		srv.request.o_x = getObjectCoord(	m_immov.at(0)[0],
											m_immov.at(0)[6],
											0.01);
		srv.request.o_y = getObjectCoord(	m_immov.at(0)[1],
											m_immov.at(0)[7],
											0.01);
		srv.request.o_z = m_immov.at(0)[2] + m_immov.at(0)[8];
		srv.request.o_r = 0.0;
		srv.request.o_p = 0.0;
		srv.request.o_yaw = m_distD(m_rng) * 2*M_PI;

		if (immov_added < m_num_immov) {
			srv.request.movable = false;
		}
		else {
			srv.request.movable = true;
		}
		srv.request.mu = -1; // set in simulator

		if (m_services.at(m_servicemap["add_ycb_object"]).call(srv))
		{
			total_added++;
			if (srv.response.id != -1)
			{
				std::vector<double> data{	srv.request.o_x,
											srv.request.o_y,
											srv.request.o_z,
											srv.request.o_r,
											srv.request.o_p,
											srv.request.o_yaw,
											(double)srv.request.movable,
											1.0 // YCB flag for YCB object = true
										};
				if (immov_added < m_num_immov)
				{
					m_immov_ids.push_back(
						std::make_pair(srv.response.id, srv.request.obj_id));
					m_immov.push_back(data);
					immov_added++;
				}
				else
				{
					m_mov_ids.push_back(
						std::make_pair(srv.response.id, srv.request.obj_id));
					m_mov.push_back(data);
					mov_added++;
				}
			}

			if (!srv.response.removes.empty())
			{
				std::size_t newsize = m_removed.size() +
												srv.response.removes.size();
				m_removed.reserve(newsize);
				m_removed_ids.reserve(newsize);

				for (const auto& e : srv.response.removes)
				{
					if (!removeObject(e)) {
						ROS_ERROR("Could not remove object %d.", e);
					}
				}
			}
		}
	}

	return true;
}

void BulletSim::readRobotFromFile(int replay_id, const std::string& suffix)
{
	std::string filename = getPartialFilename(replay_id);
	if (!suffix.empty())
	{
		filename.insert(filename.find_last_of('.'), "_");
		filename.insert(filename.find_last_of('.'), suffix);
	}

	std::ifstream DATA;
	DATA.open(filename);

	if (DATA.is_open())
	{
		std::string line;
		while (!DATA.eof())
		{
			getline(DATA, line);
			if (line.compare("R") == 0)
			{
				getline(DATA, line);
				std::stringstream ss(line);
				std::string split;
				while (ss.good())
				{
					getline(ss, split, ',');
					m_robot.push_back(std::stod(split));
				}
			}
		}
	}

	DATA.close();
}

bool BulletSim::setupTables()
{
	auto objs = m_immov.size();

	comms::AddObject srv;
	for (int i = 0; i < objs; ++i)
	{
		srv.request.shape = m_immov_ids.at(i).second;

		srv.request.o_x = m_immov.at(i)[0];
		srv.request.o_y = m_immov.at(i)[1];
		srv.request.o_z = m_immov.at(i)[2];

		srv.request.o_r = m_immov.at(i)[3];
		srv.request.o_p = m_immov.at(i)[4];
		srv.request.o_yaw = m_immov.at(i)[5];

		srv.request.x_size = m_immov.at(i)[6];
		srv.request.y_size = m_immov.at(i)[7];
		srv.request.z_size = m_immov.at(i)[8];

		srv.request.locked = true; // (bool)m_immov.at(i)[9];
		srv.request.movable = false; // (bool)m_immov.at(i)[10];
		srv.request.mu = -1;

		srv.request.sim_id = -1; // add to all simulator instances
		// Adding Table
		if (m_services.at(m_servicemap["add_object"]).call(srv))
		{
			ROS_DEBUG("Added object ID: %d", srv.response.id);
			m_immov_ids.at(i).first = srv.response.id;
		}

		else
		{
			return false;
		}
	}

	return true;
}

bool BulletSim::readTables(const std::string& filename)
{
	char sTemp[1024];
	int num_obs = 0;
	m_tables = 0;

	FILE* fCfg = fopen(filename.c_str(), "r");

	if (fCfg == NULL)
	{
		ROS_ERROR("Unable to open scene file. Exiting.\n");
		return false;
	}

	// get number of objects
	if (fscanf(fCfg, "%s", sTemp) < 1)
	{
		ROS_ERROR("Parsed string (# objects) has length < 1. Exiting.\n");
		return false;
	}

	num_obs = atoi(sTemp);
	m_tables = num_obs;

	// get {shape x y z dimx dimy dimz} for each object. dims are half extents.
	m_immov.resize(num_obs);
	m_immov_ids.clear();
	for (int i = 0; i < num_obs; ++i)
	{
		if (fscanf(fCfg, "%s", sTemp) < 1)
		{
			ROS_ERROR("Parsed string (shape) has length < 1. Exiting.\n");
			return false;
		}
		m_immov_ids.push_back(std::make_pair(-1, atoi(sTemp)));

		m_immov[i].resize(12);
		for (int j = 0; j < 11; ++j)
		{
			if (fscanf(fCfg, "%s", sTemp) < 1)
			{
				ROS_ERROR("Parsed string (coord %i) has length < 1. Exiting.", j+1);
				return false;
			}

			if (!feof(fCfg) && strlen(sTemp) != 0) {
				m_immov[i][j] = atof(sTemp);
			}
		}
		m_immov[i][11] = 0.0; // YCB flag for primitive object = false

		if (fscanf(fCfg, "%s", sTemp)) {
			ROS_DEBUG("Finished reading object %s", sTemp);
		}
	}
	fclose(fCfg);

	assert(m_immov.size() == m_immov_ids.size());

	return true;
}

bool BulletSim::resetSimulation()
{
	comms::ResetSimulation srv;
	srv.request.req = true;

	if (!m_services.at(m_servicemap["reset_simulation"]).call(srv))
	{
		ROS_ERROR("Failed to reset simulation.");
		return false;
	}

	return true;
}

void BulletSim::setupServices()
{
	m_services.clear();

	m_servicemap["add_object"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::AddObject>("/add_object"));

	m_servicemap["add_ycb_object"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::AddYCBObject>("/add_ycb_object"));

	m_servicemap["add_robot"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::AddRobot>("/add_robot"));

	m_servicemap["set_robot_state"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::SetRobotState>("/set_robot_state"));

	m_servicemap["reset_arm"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::ResetArm>("/reset_arm"));

	m_servicemap["check_scene"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::CheckScene>("/check_scene"));

	m_servicemap["reset_scene"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::ResetScene>("/reset_scene"));

	m_servicemap["set_colours"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::SetColours>("/set_colours"));

	m_servicemap["reset_simulation"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::ResetSimulation>("/reset_simulation"));

	m_servicemap["exec_traj"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::ExecTraj>("/exec_traj"));

	m_servicemap["sim_pushes"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::SimPushes>("/sim_pushes"));

	m_servicemap["remove_constraint"] = m_services.size();
	m_services.push_back(
			m_nh.serviceClient<comms::ResetSimulation>("/remove_constraint"));
}

bool BulletSim::removeObject(const int& id)
{
	int loc = -1;
	for (auto it = m_immov_ids.begin(); it != m_immov_ids.end(); ++it)
	{
		if (it->first == id)
		{
			loc = std::distance(m_immov_ids.begin(), it);
		}
	}

	if (loc != -1)
	{
		m_removed.push_back(m_immov.at(loc));
		m_removed_ids.push_back(m_immov_ids.at(loc));

		m_immov.erase(m_immov.begin() + loc);
		m_immov_ids.erase(m_immov_ids.begin() + loc);

		if (loc < m_tables) {
			m_tables--;
		}

		return true;
	}
	else
	{
		for (auto it = m_mov_ids.begin(); it != m_mov_ids.end(); ++it)
		{
			if (it->first == id)
			{
				loc = std::distance(m_mov_ids.begin(), it);
			}
		}

		if (loc != -1)
		{
			m_removed.push_back(m_mov.at(loc));
			m_removed_ids.push_back(m_mov_ids.at(loc));

			m_mov.erase(m_mov.begin() + loc);
			m_mov_ids.erase(m_mov_ids.begin() + loc);

			return true;
		}
	}

	return false;
}

double BulletSim::getObjectCoord(double origin, double half_extent, double border)
{
	double val = m_distD(m_rng);
	double llim = origin - half_extent + border;
	double ulim = origin + half_extent - border;
	val = val * (ulim - llim) + llim;

	return val;
}

std::string BulletSim::getPartialFilename(int id)
{
	std::string filename(__FILE__), level;
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../../../../simplan/src/simplan/data/clutter_scenes/";

	if (id < 100000) {
		level = "0";
		// ROS_WARN("Planning for a scene with no movable objects!");
	}
	else if (id < 200000) {
		level = "5";
	}
	else if (id < 300000) {
		level = "10";
	}
	else {
		level = "15";
	}
	filename += level + "/plan_";

	std::stringstream ss;
	ss << std::setw(6) << std::setfill('0') << id;
	std::string s = ss.str();

	filename += s + "_SCENE.txt";

	return filename;
}

void BulletSim::GetShelfParams(
		double& ox, double& oy, double& oz,
		double& sx, double& sy, double& sz)
{
	ox = m_immov.at(0)[0] - m_immov.at(0)[6];
	oy = m_immov.at(0)[1] - m_immov.at(0)[7];
	oz = m_immov.at(0)[2] + m_immov.at(0)[8];
	sx = m_immov.at(0)[6] * 2.0;
	sy = m_immov.at(0)[7] * 2.0;

	if (m_tables == 1)
	{
		double zlim;
		m_nh.param("objects/z_size", zlim, 1.0);
		sz = oz + (zlim * 2.0);
	}
	else
	{
		sz = (m_immov.at(2)[2] - m_immov.at(2)[8]) - oz;
	}
}

} // namespace clutter
