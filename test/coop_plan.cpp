#include <pushplan/search/planner.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/debug/visualizer_ros.h>
#include <ros/ros.h>

#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>

using namespace clutter;

void SaveData(
	int scene_id,
	int mapf_calls, int mapf_sucesses, bool lucky, bool rearranged,
	bool dead, bool rearrange, std::uint32_t violation)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/MAIN.csv";

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "MAPFCalls,MAPFSuccesses,"
				<< "NotLucky,NotRearranged,"
				<< "Timeout?,Rearranged?,ExecViolation?\n";
	}

	STATS << scene_id << ','
			<< mapf_calls << ',' << mapf_sucesses << ','
			<< lucky << ',' << rearranged << ','
			<< dead << ',' << rearrange << ','
			<< violation << '\n';
	STATS.close();
}

int main(int argc, char** argv)
{
	std::srand(std::time(0));

	ros::init(argc, argv, "whca");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	smpl::VisualizerROS visualizer(nh, 100);
	smpl::viz::set_visualizer(&visualizer);

	// Let publishers set up
	ros::Duration(1.0).sleep();

	// read from NONE file
	std::string filename(__FILE__), results(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/FIRST.txt";
	results = results.substr(0, found + 1) + "../dat/RESULTS.csv";

	std::ifstream NONE;
	NONE.open(filename);

	if (NONE.is_open())
	{
		std::string line, level;
		while (!NONE.eof())
		{
			getline(NONE, line);
			if (line.length() == 0) {
				break;
			}
			int scene_id = std::stoi(line);
			if (scene_id < 100000)
			{
				level = "0";
				// ROS_WARN("Planning for a scene with no movable objects!");
			}
			else if (scene_id < 200000) {
				level = "5";
			}
			else if (scene_id < 300000) {
				level = "10";
			}
			else {
				level = "15";
			}

			std::string planfile(__FILE__);
			auto found = planfile.find_last_of("/\\");
			planfile = planfile.substr(0, found + 1) + "../../../../simplan/src/simplan/data/clutter_scenes/";
			planfile += level + "/plan_" + line + "_SCENE.txt";
			ROS_WARN("Run planner on: %s", planfile.c_str());

			Planner p;
			bool ycb;
			ph.getParam("objects/ycb", ycb);
			if (ycb) {
				scene_id = -1;
			}
			if (!p.Init(planfile, scene_id, ycb)) {
				continue;
			}
			ROS_INFO("Planner and simulator init-ed!");

			if (!p.SetupNGR())
			{
				ROS_ERROR("Failed to initialise NGRs for movable objects!");
				continue;
			}
			ROS_INFO("All movable object init-ed with NGR grid!");

			int mapf_calls = 0, mapf_sucesses = 0;
			bool dead = false, rearrange = true, lucky = false, rearranged = false;
			std::uint32_t violation;
			do
			{
				++mapf_calls;
				if (p.Plan())
				{
					++mapf_sucesses;

					// ROS_WARN("Try extraction before rearrangement! Did we get lucky?");
					// if (p.Alive() && p.TryExtract()) {
					// 	lucky = true;
					// }

					if (p.Alive()) {
						ROS_WARN("Try rearrangement!");
						rearrange = p.Rearrange();
					}

					// ROS_WARN("Try extraction after rearrangement! Did we successfully rearrange?");
					// if (p.Alive() && p.TryExtract()) {
					// 	rearranged = true;
					// }

					// ROS_WARN("Try planning with all objects as obstacles! Are we done?");
					// if (p.Alive())
					// {
					// 	// ROS_WARN("YAYAYAY! We did it!");
					// 	break;
					// }
				}
			}
			while (rearrange && p.Alive());
			// dead = !p.Alive();

			// if (p.Alive()) {
			// 	violation = p.RunSim();

			// 	if (violation == 0) {
			// 		ROS_WARN("SUCCESS!!!");
			// 	}
			// 	else {
			// 		ROS_ERROR("FAILURE!!!");
			// 	}
			// }
			// else {
			// 	violation |= 0x00000008;
			// 	ROS_ERROR("Planner terminated!!!");
			// }

			// if (SAVE)
			// {
			// 	SaveData(
			// 		scene_id,
			// 		mapf_calls, mapf_sucesses, lucky, rearranged,
			// 		dead, rearrange, violation);
			// }
		}
	}
	else
	{
		ROS_ERROR("Planner init error");
		return false;
	}

	NONE.close();

	return 0;
}
