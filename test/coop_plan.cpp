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

	ROS_INFO("Initialize visualizer");
	smpl::VisualizerROS visualizer(nh, 100);
	smpl::viz::set_visualizer(&visualizer);

	// Let publishers set up
	ros::Duration(1.0).sleep();

	// put together scene file name
	std::string scene, level;
	ph.param<std::string>("scene_id", scene, "100000");
	int scene_id = std::stoi(scene);
	if (scene_id < 100000) {
		level = "0";
		ROS_WARN("Planning for a scene with no movable objects!");
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
	planfile += level + "/plan_" + scene + "_SCENE.txt";

	Planner p(planfile);
	p.Plan();

	return 0;
}
