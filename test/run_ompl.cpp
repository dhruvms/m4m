#include <pushplan/sampling/ompl.hpp>
#include <pushplan/search/planner.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/constants.hpp>

#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <smpl/debug/visualizer_ros.h>
#include <ros/ros.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/PathSimplifier.h>

using namespace clutter;

int main(int argc, char* argv[])
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
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/SAMPLING.txt";

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
			found = planfile.find_last_of("/\\");
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

			///////////////////
			// Planner Setup //
			///////////////////

			ROS_INFO("Initialize the planner");

			auto* state_space = new PushplanStateSpace(&p);
			ompl::base::StateSpacePtr ss_ptr(state_space);
			ompl::geometric::SimpleSetup ss(ss_ptr);

			// Use CollisionSpace as the collision checker...
			ss.setStateValidityChecker([&](const ompl::base::State* state)
			{
				ROS_ERROR("call StateSpace StateValidityChecker");
				smpl::RobotState values;
				state_space->copyToReals(values, state);
				return p.StateValidityChecker(values);
			});

			std::shared_ptr<PushplanMotionValidator> motion_validator = std::make_shared<PushplanMotionValidator>(ss.getSpaceInformation());
			motion_validator->SetRobot(p.GetRobot());
			ss.getSpaceInformation()->setMotionValidator(motion_validator);

			// Set up a projection evaluator to provide forward kinematics...
			auto* fk_projection = new ProjectionEvaluatorFK(state_space);
			fk_projection->model = p.GetRobot()->RobotModel();
			// state_space->registerProjection(
			// 		"fk", ompl::base::ProjectionEvaluatorPtr(fk_projection));

			// Finally construct/initialize the planner...
			int ompl_planner;
			ph.getParam("sampling/ompl_planner", ompl_planner);
			ompl::base::PlannerPtr planner;

			switch (ompl_planner)
			{
				case 0:
				{
					auto* rrt = new ompl::geometric::RRT(ss.getSpaceInformation());
					planner = ompl::base::PlannerPtr(rrt);
					break;
				}

				case 1:
				{
					auto* kpiece = new ompl::geometric::KPIECE1(ss.getSpaceInformation());
					kpiece->setProjectionEvaluator(ompl::base::ProjectionEvaluatorPtr(fk_projection));
					planner = ompl::base::PlannerPtr(kpiece);
					break;
				}
			}

			// // Perform extra configuration steps, if needed.
			// // This call will also issue a call to ompl::base::SpaceInformation::setup() if needed.
			// // This must be called before solving.
			// planner->setup();
			ss.setPlanner(planner);

			//////////////
			// Planning //
			//////////////

			ROS_INFO("Setup the query");

			smpl::RobotState start_state;
			p.GetStartState(start_state);

			comms::ObjectsPoses start_objects = p.GetStartObjects();

			state_space->SetStartObjects(start_objects);

			ompl::base::ScopedState<PushplanStateSpace> start(ss_ptr);
			start->setJointState(start_state);
			start->copyObjectState(start_objects);

			// TODO: set start joints and objects here, before setStartState
			if (ss.getSpaceInformation()->SpaceInformation::isValid(start.get())) {
				ss.setStartState(start);
			}

			auto* goal_condition = new PushplanPoseGoal(
					ss.getSpaceInformation(), p.GoalPose());
			goal_condition->SetTolerances(
					Eigen::Vector3d(0.015, 0.015, 0.015),
					Eigen::Vector3d(0.05, 0.05, 0.05));
			goal_condition->SetRobot(p.GetRobot());
			ss.setGoal(ompl::base::GoalPtr(goal_condition));

			// plan
			double start_time = GetTime();
			double allowed_planning_time;
			ph.getParam("goal/total_budget", allowed_planning_time);
			ROS_INFO("Calling solve...");
			auto solved = ss.solve(allowed_planning_time);

			bool plan_success = ompl::base::PlannerStatus::StatusType(solved) == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
			bool exec_success = false;
			double exec_time = 0.0;
			double planning_time = GetTime() - start_time;
			double sim_time = state_space->GetSimTime();
			int num_sims = state_space->GetNumSims();

			///////////////////////////////////
			// Visualizations and Statistics //
			///////////////////////////////////

			// TODO: print statistics

			if (plan_success)
			{
				std::cout << "Original solution length = " << ss.getSolutionPath().getStateCount() << "\n";

				ompl::geometric::PathSimplifier shortcut(ss.getSpaceInformation());
				shortcut.reduceVertices(ss.getSolutionPath());

				std::cout << "Solution length after reduceVertices = " << ss.getSolutionPath().getStateCount() << "\n";
				ROS_INFO("Animate path");

				trajectory_msgs::JointTrajectory traj;
				traj.points.clear();
				trajectory_msgs::JointTrajectoryPoint wp;
				wp.positions.clear();

				smpl::RobotState s;
				s.clear();

				auto* curr_state = ss.getSolutionPath().getState(0);
				auto* prev_state = ss.getSolutionPath().getState(0);
				state_space->copyToReals(s, curr_state);
				wp.positions = s;
				wp.time_from_start = ros::Duration(0.0);

				traj.points.push_back(wp);

				for (size_t pidx = 1; pidx < ss.getSolutionPath().getStateCount(); ++pidx)
				{
					curr_state = ss.getSolutionPath().getState(pidx);
					prev_state = ss.getSolutionPath().getState(pidx-1);

					s.clear();
					state_space->copyToReals(s, curr_state);

					wp.positions.clear();
					wp.positions = s;

					double max_speed_factor = 0.1;
					double max_time = 0.0;
					for (size_t jidx = 0; jidx < state_space->RMJointVariableCount(); ++jidx) {
						auto from_pos = prev_state->as<PushplanStateSpace::StateType>()->getJoint(jidx);
						auto to_pos = curr_state->as<PushplanStateSpace::StateType>()->getJoint(jidx);
						auto vel = state_space->RMVelLimit(jidx) * max_speed_factor;
						if (vel <= 0.0) {
							continue;
						}
						auto t = 0.0;
						if (state_space->RMIsContinuous(jidx)) {
							t = smpl::angles::shortest_angle_dist(from_pos, to_pos) / vel;
						} else {
							t = fabs(to_pos - from_pos) / vel;
						}

						max_time = std::max(max_time, t);
					}
					// if (max_time <= 0.01) {
					// 	continue;
					// }

					wp.time_from_start = traj.points.back().time_from_start + ros::Duration(max_time);

					traj.points.push_back(wp);
				}

				int grasp_at = traj.points.size() + 1;

				trajectory_msgs::JointTrajectory grasp_traj;
				auto pregrasp_state = ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount() - 1)->as<PushplanStateSpace::StateType>()->joint_state();

				if (!p.GetRobot()->ComputeGraspTraj(pregrasp_state, grasp_traj)) {
					ROS_ERROR("Grasp trajectory computation for solution path failed?");
				}

				for (auto itr = grasp_traj.points.begin() + 1; itr != grasp_traj.points.end(); ++itr)
				{
					itr->time_from_start += traj.points.back().time_from_start;
					traj.points.push_back(*itr);
				}

				traj.joint_names = p.GetRobot()->GetPlanningJoints();
				start_time = GetTime();
				exec_success = p.ExecTraj(traj, grasp_at);
				exec_time = GetTime() - start_time;
			}

			std::string filename(__FILE__);
			found = filename.find_last_of("/\\");
			filename = filename.substr(0, found + 1) + "../dat/";

			switch (ompl_planner)
			{
				case 0:
				{
					filename += "RRT.csv";
					break;
				}

				case 1:
				{
					filename += "KPIECE.csv";
					break;
				}
			}

			bool exists = FileExists(filename);
			std::ofstream STATS;
			STATS.open(filename, std::ofstream::out | std::ofstream::app);
			if (!exists) {
				STATS << "UID,PlanSuccess,ExecSuccess,ExecTime,PlanningTime,NumSims,SimTime\n";
			}

			STATS << scene_id << ','
					<< plan_success << ',' << exec_success << ','
					<< exec_time << ',' << planning_time << ','
					<< num_sims << ',' << sim_time << '\n';
			STATS.close();
		}
	}

	NONE.close();
}
