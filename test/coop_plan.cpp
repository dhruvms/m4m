#include <pushplan/planner.hpp>

#include <smpl/debug/visualizer_ros.h>
#include <ros/ros.h>

#include <string>

using namespace clutter;

int main(int argc, char** argv)
{
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
			if (!p.Init(planfile, scene_id)) {
				continue;
			}

			bool rearrange = true;
			do
			{
				if (p.Plan())
				{
					// ROS_WARN("Try extraction!");
					// if (p.TryExtract()) {
					// 	break;
					// }
					rearrange = p.Rearrange();
				}
			}
			while (p.Alive() && rearrange);

			std::uint32_t violation = p.RunSim();
			ROS_INFO("Iteration result: rearrangement planning = %d, execution = %d", rearrange, violation);
			// std::ofstream RESULTS;
			// RESULTS.open(results, std::ofstream::out | std::ofstream::app);
			// RESULTS << scene_id << ',' << int(rearrange) << ',' << int(violation) << '\n';
			// RESULTS.close();
			// p.SaveData();

			if (violation == 0) {
				ROS_WARN("SUCCESS!!!");
			}
			else {
				ROS_WARN("FAILURE!!!");
			}
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
