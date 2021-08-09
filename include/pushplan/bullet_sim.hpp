#ifndef BULLET_SIM_HPP
#define BULLET_SIM_HPP

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <string>
#include <vector>
#include <unordered_map>
#include <random>

namespace clutter
{

class BulletSim
{
public:
	BulletSim(
		const std::string& tables, bool ycb=false,
		int replay_id=-1, const std::string& suffix=std::string(),
		int immovable_objs=5, int movable_objs=5);

	bool SetRobotState(const sensor_msgs::JointState& msg);
	bool ResetArm(const int& arm);
	bool CheckScene(const int& arm, int& count);
	bool ResetScene();
	bool SetColours();
	bool ExecTraj(const trajectory_msgs::JointTrajectory& traj);

private:
	int m_num_immov, m_num_mov, m_robot_id, m_tables;

	ros::NodeHandle m_nh;
	std::vector<ros::ServiceClient> m_services;
	std::unordered_map<std::string, int> m_servicemap;

	std::vector<double> m_robot;
	std::vector<std::vector<double>> m_immov, m_mov, m_removed;
	std::vector<std::pair<int, int>> m_immov_ids, m_mov_ids, m_removed_ids;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::uniform_int_distribution<> m_distI;

	bool setupObjects(bool ycb);
	bool setupObjectsFromFile(
		bool ycb, int replay_id, const std::string& suffix);

	bool addRobot(int replay_id, const std::string& suffix);

	bool setupPrimitiveObjectsFromFile(
		int replay_id, const std::string& suffix);
	bool setupYCBObjectsFromFile(
		int replay_id, const std::string& suffix);
	void setupTableFromFile(int replay_id, const std::string& suffix);
	bool setupPrimitiveObjects();
	bool setupYCBObjects();

	void readRobotFromFile(int replay_id, const std::string& suffix);

	bool setupTables();
	bool readTables(const std::string& filename);

	bool resetSimulation();
	void setupServices();

	bool removeObject(const int& id);
	double getObjectCoord(double origin, double half_extent, double border);
	std::string getPartialFilename(int id);
};

} // namespace clutter


#endif // BULLET_SIM_HPP
