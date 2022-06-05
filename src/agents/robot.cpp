#include <pushplan/agents/robot.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/cbs_nodes.hpp>
#include <pushplan/utils/geometry.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/pr2_allowed_collision_pairs.h>

#include <smpl/stl/memory.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl_collision_checking/types.h>
#include <sbpl_collision_checking/voxel_operations.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <smpl/angles.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/console/console.h>
#include <smpl/ros/factories.h>
#include <smpl_urdf_robot_model/urdf_robot_model.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <thread>

namespace clutter
{

bool Robot::Setup()
{
	///////////
	// Stats //
	///////////

	m_stats["plan_push_calls"] = 0;
	m_stats["push_samples_found"] = 0;
	m_stats["push_actions_found"] = 0;
	m_stats["push_plan_time"] = 0.0;
	m_stats["push_sim_time"] = 0.0;
	m_stats["push_sim_successes"] = 0;

	/////////////////
	// Robot Model //
	/////////////////

	// Robot description required to initialize collision checker and robot
	// model...
	auto robot_description_key = "robot_description";
	std::string robot_description_param;
	if (!m_nh.searchParam(robot_description_key, robot_description_param))
	{
		ROS_ERROR("Failed to find 'robot_description' key on the param server");
		return false;
	}

	if (!m_nh.getParam(robot_description_param, m_robot_description))
	{
		ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
		return false;
	}

	if (!readRobotModelConfig(ros::NodeHandle("~robot_model")))
	{
		ROS_ERROR("Failed to read robot model config from param server");
		return false;
	}

	// Everyone needs to know the name of the planning frame for reasons...
	// ...frame_id for the occupancy grid (for visualization)
	// ...frame_id for collision objects (must be the same as the grid, other than that, useless)
	if (!m_ph.getParam("planning_frame", m_planning_frame))
	{
		ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
		return false;
	}

	if (!setupRobotModel() || !m_rm)
	{
		ROS_ERROR("Failed to set up Robot Model");
		return false;
	}

	//////////////////////////////////
	// Initialize Collision Checker //
	//////////////////////////////////

	initOccupancyGrids();

	if (!initCollisionChecker())
	{
		ROS_ERROR("Failed to initialise collision checker!");
		return false;
	}
	m_cc_i->setWorldToModelTransform(Eigen::Affine3d::Identity());
	m_cc_m->setWorldToModelTransform(Eigen::Affine3d::Identity());

	// Read in start state from file and update the scene...
	// Start state is also required by the planner...
	m_start_state.joint_state.name.clear();
	m_start_state.joint_state.position.clear();
	if (!readStartState())
	{
		ROS_ERROR("Failed to get initial configuration.");
		return false;
	}

	if(!setReferenceStartState())
	{
		ROS_ERROR("Failed to set start state!");
		return false;
	}

	// from smpl::ManipLattice
	m_min_limits.resize(m_rm->jointVariableCount());
	m_max_limits.resize(m_rm->jointVariableCount());
	m_continuous.resize(m_rm->jointVariableCount());
	m_bounded.resize(m_rm->jointVariableCount());
	for (int jidx = 0; jidx < m_rm->jointVariableCount(); ++jidx)
	{
		m_min_limits[jidx] = m_rm->minPosLimit(jidx);
		m_max_limits[jidx] = m_rm->maxPosLimit(jidx);
		m_continuous[jidx] = m_rm->isContinuous(jidx);
		m_bounded[jidx] = m_rm->hasPosLimit(jidx);
	}

	std::vector<int> discretization(m_rm->jointVariableCount());
	std::vector<double> deltas(m_rm->jointVariableCount());
	std::vector<double> resolutions(m_rm->jointVariableCount());
	readResolutions(resolutions);
	for (size_t vidx = 0; vidx < m_rm->jointVariableCount(); ++vidx)
	{
		if (m_continuous[vidx])
		{
			discretization[vidx] = (int)std::round((2.0 * M_PI) / resolutions[vidx]);
			deltas[vidx] = (2.0 * M_PI) / (double)discretization[vidx];
		}
		else if (m_bounded[vidx])
		{
			auto span = std::fabs(m_max_limits[vidx] - m_min_limits[vidx]);
			discretization[vidx] = std::max(1, (int)std::round(span / resolutions[vidx]));
			deltas[vidx] = span / (double)discretization[vidx];
		}
		else
		{
			discretization[vidx] = std::numeric_limits<int>::max();
			deltas[vidx] = resolutions[vidx];
		}
	}

	m_coord_vals = std::move(discretization);
	m_coord_deltas = std::move(deltas);

	// setup planar approx
	m_priority = 0;
	m_grasp_at = -1;
	m_b = SEMI_MINOR;
	m_shoulder = "r_shoulder_pan_link";
	m_elbow = "r_elbow_flex_link";
	m_wrist = "r_wrist_roll_link";
	m_tip = "r_gripper_finger_dummy_planning_link"; // TODO: check
	m_table_z = m_cc->GetTableHeight();
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
	m_distG = std::normal_distribution<>(0.0, 1.0);

	m_link_s = smpl::urdf::GetLink(&m_rm->m_robot_model, m_shoulder.c_str());
	m_link_e = smpl::urdf::GetLink(&m_rm->m_robot_model, m_elbow.c_str());
	m_link_w = smpl::urdf::GetLink(&m_rm->m_robot_model, m_wrist.c_str());
	m_link_t = smpl::urdf::GetLink(&m_rm->m_robot_model, m_tip.c_str());

	initObjects();

	smpl::RobotState dummy;
	dummy.insert(dummy.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	m_rm->computeFK(dummy); // for reasons unknown
	// if (!reinitStartState())
	// {
	// 	ROS_ERROR("Failed to reinit start state!");
	// 	return false;
	// }
	// ROS_WARN("Start state re-init!");

	m_planner_init = false;
	m_pushes_per_object = -1;
	m_ph.getParam("robot/grasping/tries", m_grasp_tries);
	m_ph.getParam("robot/grasping/lift", m_grasp_lift);
	m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 10);

	return true;
}

bool Robot::SavePushData(int scene_id, bool reset)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/ROBOT.csv";

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "PlanPushCalls,PushSamplesFound,PushActionsFound,"
				<< "PushPlanTime,PushSimTime,PushSimSuccesses\n";
	}

	STATS << scene_id << ','
			<< m_stats["plan_push_calls"] << ',' << m_stats["push_samples_found"] << ',' << m_stats["push_actions_found"] << ','
			<< m_stats["push_plan_time"] << ',' << m_stats["push_sim_time"] << ','
			<< m_stats["push_sim_successes"] << '\n';
	STATS.close();

	if (reset)
	{
		m_stats["plan_push_calls"] = 0;
		m_stats["push_samples_found"] = 0;
		m_stats["push_actions_found"] = 0;
		m_stats["push_plan_time"] = 0.0;
		m_stats["push_sim_time"] = 0.0;
		m_stats["push_sim_successes"] = 0;
	}
}

bool Robot::CheckCollisionWithObject(const LatticeState& robot, Agent* a, int t)
{
	// if (t > m_grasp_at + 1) {
	// 	attachObject(m_ooi);
	// }
	std::vector<Object*> o;
	o.push_back(a->GetObject());
	ProcessObstacles(o, false, true);

	bool collision = !m_cc_m->isStateValid(robot.state);

	// if (t > m_grasp_at + 1) {
	// 	detachObject();
	// }
	ProcessObstacles(o, true, true);

	return collision;
}

bool Robot::CheckCollision(const LatticeState& robot, int t)
{
	// if (t > m_grasp_at + 1) {
	// 	attachObject(m_ooi);
	// }

	bool collision = !m_cc_m->isStateValid(robot.state);
	// if (collision) {
	// 	auto* vis_name = "conflict";
	// 	auto markers = m_cc_m->getCollisionModelVisualization(robot.state);
	// 	for (auto& marker : markers) {
	// 		marker.ns = vis_name;
	// 	}
	// 	SV_SHOW_INFO_NAMED(vis_name, markers);
	// 	SMPL_ERROR("Conflict at time %d", t);
	// }

	// if (t > m_grasp_at + 1) {
	// 	detachObject();
	// }
	return collision;
}

void Robot::SetMovables(const std::vector<std::shared_ptr<Agent> >& agents)
{
	moveit_msgs::CollisionObject mov_obj;
	for (const auto& a: agents)
	{
		a->GetMoveitObj(mov_obj);
		m_movables.push_back(mov_obj);
	}
}

bool Robot::ProcessObstacles(const std::vector<Object>& obstacles,
	bool remove, bool movable)
{
	for (const auto& obs: obstacles)
	{
		if (!obs.desc.ycb)
		{
			moveit_msgs::CollisionObject obj_msg;
			if (!getCollisionObjectMsg(obs, obj_msg, remove)) {
				return false;
			}
			if (!processCollisionObjectMsg(obj_msg, movable)) {
				return false;
			}
		}
		else
		{
			if (!processSTLMesh(obs, remove, movable)) {
				return false;
			}
		}
	}

	auto cc = movable ? m_cc_m.get() : m_cc_i.get();
	SV_SHOW_INFO(cc->getCollisionWorldVisualization());
	return true;
}

bool Robot::ProcessObstacles(const std::vector<Object*>& obstacles,
	bool remove, bool movable)
{
	for (const auto& obs: obstacles)
	{
		if (!obs->desc.ycb)
		{
			moveit_msgs::CollisionObject obj_msg;
			if (!getCollisionObjectMsg(*obs, obj_msg, remove)) {
				return false;
			}
			if (!processCollisionObjectMsg(obj_msg, movable)) {
				return false;
			}
		}
		else
		{
			if (!processSTLMesh(*obs, remove, movable)) {
				return false;
			}
		}
	}

	auto cc = movable ? m_cc_m.get() : m_cc_i.get();
	SV_SHOW_INFO(cc->getCollisionWorldVisualization());
	return true;
}

bool Robot::SetScene(const comms::ObjectsPoses& objects)
{
	for (const auto& object : objects.poses)
	{
		// find the collision object with this name
		auto* cc_object = findCollisionObject(std::to_string(object.id), true);
		if (!cc_object) {
			return false;
		}

		Eigen::Affine3d pose = Eigen::Translation3d(object.xyz[0], object.xyz[1], object.xyz[2]) *
			Eigen::AngleAxisd(object.rpy[2], Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(object.rpy[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(object.rpy[0], Eigen::Vector3d::UnitX());;

		if (cc_object->shape_poses[0].translation() == pose.translation()
			&& cc_object->shape_poses[0].rotation() == pose.rotation()) {
			continue;
		}
		else
		{
			cc_object->shape_poses[0] = pose;
			if (!m_cc_m->moveShapes(cc_object)) {
				return false;
			}
		}
	}

	SV_SHOW_INFO(m_cc_m->getCollisionWorldVisualization());
	return true;
}

bool Robot::SteerAction(
	const smpl::RobotState& to, int steps,
	const smpl::RobotState& from, const comms::ObjectsPoses& start_objs,
	smpl::RobotState& action_end, comms::ObjectsPoses& end_objs,
	std::uint32_t& result)
{
	result = 0x00000000; // error in action computation

	const double res = DEG5 / 5; // 1 degree
	smpl::collision::MotionInterpolation interp(m_cc_i->robotCollisionModel().get());

	auto planning_variables = m_cc_i->planningVariables();
	m_cc_i->robotMotionCollisionModel()->fillMotionInterpolation(
			from,
			to,
			planning_variables,
			res,
			interp);

	smpl::RobotState interm;
	int collides_immov_idx = -1, collides_mov_idx = -1;
	std::size_t max_idx = interp.waypointCount() > steps ? steps : interp.waypointCount();
	for (std::size_t i = 0; i < max_idx; i++)
	{
		interp.interpolate(i, interm, planning_variables);
		if (!m_rm->checkJointLimits(interm)) {
			return false;
		}

		bool mov_collision = false, immov_collision = false;

		// check if interpolated waypoint collides with movable obstacles
		if (collides_mov_idx < 0)
		{
			mov_collision = !m_cc_m->isStateValid(interm);
			if (mov_collision) {
				collides_mov_idx = (int)i;
			}
		}
		// check if interpolated waypoint collides with immovable obstacles
		if (collides_immov_idx < 0)
		{
			immov_collision = !m_cc_i->isStateValid(interm);
			if (immov_collision) {
				collides_immov_idx = (int)i;
			}
		}

		if (mov_collision && immov_collision) {
			SMPL_DEBUG("Action must be in collision with robot body!");
			return false;
		}

		if (collides_mov_idx < 0 && collides_immov_idx >= 0) {
			// we know that its valid up to waypoint collides_immov_idx - 1
			break;
		}
		else if (collides_mov_idx >= 0 && collides_immov_idx >= 0) {
			// we know that it must be simulated up to waypoint collides_immov_idx - 1
			break;
		}
	}

	if (collides_immov_idx == 0) {
		SMPL_ERROR("Start state of SteerAction collides with immovable obstacles!!");
		return false;
	}
	result |= collides_immov_idx < 0 ? 0x00000001 : 0x00000002; // we have an action to validate, full or partial

	std::size_t action_size = collides_immov_idx < 0 ? max_idx : collides_immov_idx;
	if (collides_mov_idx < 0)
	{
		// no collision with movable objects along action, no need to simulate
		// whole or partial action is valid
		interp.interpolate(action_size - 1, action_end, planning_variables);
		end_objs = start_objs;
		result |= 0x00000004;
		// there was no need to simulate
		// if result is 5: full action, 6: partial action
		return true;
	}
	else
	{
		// simulate whole or partial action
		trajectory_msgs::JointTrajectory steer_action;
		trajectory_msgs::JointTrajectoryPoint action_pt;
		// starting waypoint
		action_pt.positions = from;
		action_pt.time_from_start = ros::Duration(0.0);
		steer_action.points.push_back(action_pt);

		for (std::size_t i = 1; i < action_size; i++)
		{
			// get intermediate waypoint
			interp.interpolate(i, action_pt.positions, planning_variables);
			// get time to intermediate waypoint
			auto prev_state = steer_action.points.at(i - 1).positions;
			double duration = profileAction(prev_state, action_pt.positions);
			action_pt.time_from_start = steer_action.points.at(i - 1).time_from_start + ros::Duration(duration);
			// add intermediate waypoint
			steer_action.points.push_back(action_pt);
		}
		// set final waypoint
		interp.interpolate(action_size - 1, action_end, planning_variables);

		int dummy, success;
		// simulate for result
		bool sim_result = m_sim->SimPushes(	{ steer_action }, -1, -1.0, -1.0,
										dummy, success,
										start_objs, end_objs);
		result |= sim_result ? 0x00000008 : 0x00000063;
		// simulated an action
		// success if result is < 100, failure otherwise
		// full action if result = 9, partial if result = 10
		return sim_result;
	}

	SMPL_ERROR("Why are we returning from the very end of Robot::SteerAction?");
	return false;
}

bool Robot::Init()
{
	m_solve.clear();
	m_grasp_at = -1;

	LatticeState s;
	s.state.insert(s.state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	reinitObjects(s.state);

	return true;
}

bool Robot::RandomiseStart()
{
	if (!reinitStartState())
	{
		ROS_ERROR("Failed to randomise start!");
		return false;
	}

	LatticeState s;
	s.state.insert(s.state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	reinitObjects(s.state);

	return true;
}

void Robot::ConvertTraj(
	const Trajectory& traj_in,
	trajectory_msgs::JointTrajectory& traj_out)
{
	traj_out.header.frame_id = m_planning_frame;
	traj_out.joint_names.clear();
	traj_out.points.clear();

	// fill joint names header
	auto& variable_names = m_rm->getPlanningJoints();
	for (auto& var_name : variable_names) {
		traj_out.joint_names.push_back(var_name);
	}

	// empty or number of points in the path
	if (!traj_out.joint_names.empty()) {
		traj_out.points.resize(traj_in.size());
	}

	for (size_t pidx = 0; pidx < traj_in.size(); ++pidx)
	{
		auto& point = traj_in[pidx].state;

		auto& p = traj_out.points[pidx];
		p.positions.resize(traj_out.joint_names.size());
		if (point.size() == traj_out.joint_names.size())
		{
			p.positions = point;
			if (pidx == 0) {
				p.time_from_start = ros::Duration(0.0);
			}
			else
			{
				double step_time = profileAction(traj_out.points[pidx-1].positions, p.positions);
				p.time_from_start = traj_out.points[pidx-1].time_from_start + ros::Duration(step_time);
			}
		}
		else
		{
			p.positions.insert(p.positions.begin(), point.begin(), point.begin() + traj_out.joint_names.size());
			p.time_from_start = ros::Duration(point.back());
		}
	}
	traj_out.header.stamp = ros::Time::now();
}

void Robot::ProfileTraj(Trajectory& traj)
{
	double prev_time = 0.0, wp_time = 0.0;
	for (size_t i = 1; i < traj.size(); ++i)
	{
		wp_time += profileAction(traj.at(i-1).state, traj.at(i).state);
		traj.at(i-1).state.push_back(prev_time);
		prev_time = wp_time;
	}
	traj.back().state.push_back(prev_time);
}

bool Robot::detachObject()
{
	const std::string attached_body_id = "att_obj";
	bool detach_result = m_cc_i->detachObject(attached_body_id)
							|| m_cc_m->detachObject(attached_body_id);
	return detach_result;
}

bool Robot::attachObject(const Object& obj)
{
	smpl::collision::CollisionObject* obj_co = obj.smpl_co;

	std::vector<shapes::ShapeConstPtr> shapes;
	smpl::collision::Affine3dVector transforms;
	if (obj.smpl_co)
	{
		for (size_t sidx = 0; sidx < obj.smpl_co->shapes.size(); ++sidx)
		{
			auto transform = Eigen::Affine3d::Identity();
			transform.translation().x() += 0.2;
			switch (obj.smpl_co->shapes.at(sidx)->type)
			{
				case smpl::collision::ShapeType::Box:
				{
					auto box = static_cast<smpl::collision::BoxShape*>(obj.smpl_co->shapes.at(sidx));
					shapes::ShapeConstPtr ao_shape(new shapes::Box(box->size[0], box->size[1], box->size[2]));
					shapes.push_back(std::move(ao_shape));
					break;
				}
				case smpl::collision::ShapeType::Cylinder:
				{
					auto cylinder = static_cast<smpl::collision::CylinderShape*>(obj.smpl_co->shapes.at(sidx));
					shapes::ShapeConstPtr ao_shape(new shapes::Cylinder(cylinder->radius, cylinder->height));
					shapes.push_back(std::move(ao_shape));
					break;
				}
				case smpl::collision::ShapeType::Mesh:
				{
					shapes::ShapeConstPtr ao_shape = MakeROSShape(obj.smpl_co->shapes.at(sidx));
					shapes.push_back(std::move(ao_shape));

					auto itr = YCB_OBJECT_DIMS.find(obj.desc.shape);
					if (itr != YCB_OBJECT_DIMS.end()) {
						transform.translation().z() -= (m_grasp_z - m_table_z);
					}

					break;
				}
				case smpl::collision::ShapeType::Sphere:
				case smpl::collision::ShapeType::Cone:
				case smpl::collision::ShapeType::Plane:
				case smpl::collision::ShapeType::OcTree:
				default:
				{
					ROS_ERROR("Incompatible shape type!");
					return false;
				}
			}
			transforms.push_back(transform);
		}
	}
	else
	{
		ROS_ERROR("Collision object not found!");
		return false;
	}


	const std::string attach_link = "r_gripper_palm_link";
	const std::string attached_body_id = "att_obj";
	bool attach_result = m_cc_i->attachObject(attached_body_id, shapes, transforms, attach_link)
							|| m_cc_m->attachObject(attached_body_id, shapes, transforms, attach_link);
	if (!attach_result)	{
		ROS_ERROR("Failed to attach body to '%s'", attach_link.c_str());
	}

	// auto markers = m_cc_i->getCollisionRobotVisualization(m_postgrasp_state);
	// SV_SHOW_INFO(markers);

	return attach_result;
}

bool Robot::attachAndCheckObject(const Object& object, const smpl::RobotState& state)
{
	if (!attachObject(object))
	{
		ROS_ERROR("Failed to attach object.");
		return false;
	}
	else
	{
		// object attached, but are we collision free with it grasped?
		if (!m_cc_i->isStateValid(state))
		{
			ROS_ERROR("Robot state is in collision with attached object.");
			return false;
		}
	}

	return true;
}

bool Robot::planApproach(
	const std::vector<std::vector<double> >& approach_cvecs,
	moveit_msgs::MotionPlanResponse& res,
	const std::vector<Object*>& movable_obstacles,
	bool finalise,
	smpl::RobotState start_state)
{
	bool have_obs = !movable_obstacles.empty();

	///////////////////
	// Plan approach //
	///////////////////

	detachObject();
	UpdateKDLRobot(0);
	// setGripper(false); // closed
	InitArmPlanner(false);
	bool goal_collides = !m_cc_i->isStateValid(m_pregrasp_state);

	moveit_msgs::MotionPlanRequest req;

	m_ph.param("allowed_planning_time", req.allowed_planning_time, 10.0);
	createJointSpaceGoal(m_pregrasp_state, req);
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;
	req.num_planning_attempts = 1;
	// req.path_constraints;
	if (goal_collides) {
		req.planner_id = "arastar.manip_cbs.joint_distance";
	}
	else {
		req.planner_id = "mhastar.manip_cbs.bfs.joint_distance";
	}

	// set appropriate start state for approach plan to pregrasp state
	moveit_msgs::RobotState orig_start = m_start_state;
	if (!start_state.empty())
	{
		m_start_state.joint_state.position.erase(
			m_start_state.joint_state.position.begin() + 1,
			m_start_state.joint_state.position.begin() + 1 + start_state.size());
		m_start_state.joint_state.position.insert(
			m_start_state.joint_state.position.begin() + 1,
			start_state.begin(), start_state.end());
	}
	req.start_state = m_start_state;
	// req.trajectory_constraints;
	// req.workspace_parameters;

	// completely unnecessary variable
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.robot_state = m_start_state;

	// SMPL_INFO("Planning to pregrasp state.");
	if (!m_planner->init_planner(planning_scene, req, res))
	{
		// ROS_ERROR("Failed to init planner!");

		if (!start_state.empty()) {
			m_start_state = orig_start;
		}
		return false;
	}

	if (have_obs) {
		// add to immovable collision checker for final plan, and
		// movable collision checker otherwise
		ProcessObstacles(movable_obstacles, false, !finalise);
		if (finalise) {
			// remove from movable collision checker if they were added before
			ProcessObstacles(movable_obstacles, true, true);
		}
	}

	if (!m_planner->solve_with_constraints(req, res, m_movables, approach_cvecs))
	{
		// ROS_ERROR("Failed to plan to pregrasp state.");
		if (have_obs && finalise) {
			// remove from immovable collision checker
			ProcessObstacles(movable_obstacles, true, false);
		}

		if (!start_state.empty()) {
			m_start_state = orig_start;
		}
		return false;
	}
	// SMPL_INFO("Robot found approach plan! # wps = %d", res.trajectory.joint_trajectory.points.size());

	if (have_obs && finalise) {
		// remove from immovable collision checker
		ProcessObstacles(movable_obstacles, true, false);
	}

	if (!start_state.empty()) {
		m_start_state = orig_start;
	}

	return true;
}

bool Robot::planRetract(
	const std::vector<std::vector<double> >& retract_cvecs,
	moveit_msgs::MotionPlanResponse& res,
	const std::vector<Object*>& movable_obstacles,
	bool finalise)
{
	bool have_obs = !movable_obstacles.empty();

	// setGripper(true); // open
	if (!attachAndCheckObject(m_ooi, m_postgrasp_state)) {
		detachObject();
		return false;
	}
	InitArmPlanner(false);

	moveit_msgs::MotionPlanRequest req;

	m_ph.param("allowed_planning_time", req.allowed_planning_time, 10.0);
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;
	req.num_planning_attempts = 1;

	// set start state for retract plan to postgrasp state
	moveit_msgs::RobotState orig_start = m_start_state;
	m_start_state.joint_state.position.erase(
		m_start_state.joint_state.position.begin() + 1,
		m_start_state.joint_state.position.begin() + 1 + m_postgrasp_state.size());
	m_start_state.joint_state.position.insert(
		m_start_state.joint_state.position.begin() + 1,
		m_postgrasp_state.begin(), m_postgrasp_state.end());
	req.start_state = m_start_state;

	// set goal state for retract plan to home state
	smpl::RobotState home;
	home.insert(home.begin(),
		orig_start.joint_state.position.begin() + 1, orig_start.joint_state.position.end());
	createJointSpaceGoal(home, req);
	bool goal_collides = !m_cc_i->isStateValid(home);
	if (goal_collides) {
		req.planner_id = "arastar.manip_cbs.joint_distance";
	}
	else {
		req.planner_id = "mhastar.manip_cbs.bfs.joint_distance";
	}

	// add EE path constraint - cannot roll or pitch more than 5 degrees
	addPathConstraint(req.path_constraints);

	// completely unnecessary variable
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.robot_state = m_start_state;
	// SMPL_INFO("Planning to home state with attached body.");
	if (!m_planner->init_planner(planning_scene, req, res))
	{
		// ROS_ERROR("Failed to init planner!");
		m_start_state = orig_start;
		return false;
	}

	if (have_obs) {
		// add to immovable collision checker for final plan, and
		// movable collision checker otherwise
		ProcessObstacles(movable_obstacles, false, !finalise);
		if (finalise) {
			// remove from movable collision checker if they were added before
			ProcessObstacles(movable_obstacles, true, true);
		}
	}

	if (!m_planner->solve_with_constraints(req, res, m_movables, retract_cvecs))
	{
		// ROS_ERROR("Failed to plan to home state with attached body.");
		detachObject();
		m_start_state = orig_start;

		if (have_obs && finalise) {
			// remove from immovable collision checker
			ProcessObstacles(movable_obstacles, true, false);
		}
		return false;
	}
	// SMPL_INFO("Robot found extraction plan! # wps = %d", res.trajectory.joint_trajectory.points.size());
	if (have_obs && finalise) {
		// remove from immovable collision checker
		ProcessObstacles(movable_obstacles, true, false);
	}

	detachObject();
	m_start_state = orig_start;

	return true;
}

void Robot::voxeliseTrajectory()
{
	m_traj_voxels.clear();
	// double start_time = GetTime();

	int sphere_count = 0;
	for (const auto& wp: m_traj.points)
	{
		auto markers = m_cc_i->getCollisionModelVisualization(wp.positions);
		for (auto& marker : markers)
		{
			std::unique_ptr<smpl::collision::CollisionShape> shape;
			shape = smpl::make_unique<smpl::collision::SphereShape>(boost::get<smpl::visual::Ellipse>(marker.shape).axis_x - RES);

			std::vector<smpl::collision::CollisionShape*> shapes;
			shapes.push_back(shape.get());

			geometry_msgs::Pose pose;
			pose.position.x = marker.pose.position[0];
			pose.position.y = marker.pose.position[1];
			pose.position.z = marker.pose.position[2];
			geometry_msgs::Quaternion orientation;
			tf::quaternionEigenToMsg(marker.pose.orientation, orientation);
			pose.orientation = orientation;

			smpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;
			Eigen::Affine3d transform;
			tf::poseMsgToEigen(pose, transform);
			shape_poses.push_back(transform);

			// create the collision object
			smpl::collision::CollisionObject co;
			co.id = std::to_string(sphere_count++);
			co.shapes = std::move(shapes);
			co.shape_poses = std::move(shape_poses);

			const double res = m_grid_ngr->resolution();
			const Eigen::Vector3d origin(
					m_grid_ngr->originX(), m_grid_ngr->originY(), m_grid_ngr->originZ());

			const Eigen::Vector3d gmin(
					m_grid_ngr->originX(), m_grid_ngr->originY(), m_grid_ngr->originZ());

			const Eigen::Vector3d gmax(
					m_grid_ngr->originX() + m_grid_ngr->sizeX(),
					m_grid_ngr->originY() + m_grid_ngr->sizeY(),
					m_grid_ngr->originZ() + m_grid_ngr->sizeZ());

			if (!smpl::collision::VoxelizeObject(co, res, origin, gmin, gmax, m_traj_voxels)) {
				continue;
			}
		}
	}

	// ROS_INFO("Robot trajectory voxelisation took %f seconds", GetTime() - start_time);

	// TODO: make this faster by looping over cells in a sphere octant
	// and computing the rest via symmetry

	// int sphere_count = 0;
	// for (const auto& wp: m_traj.points)
	// {
	// 	auto markers = m_cc_i->getCollisionModelVisualization(wp.positions);
	// 	for (auto& marker : markers)
	// 	{
	// 	}
	// }
}

void Robot::UpdateNGR(bool vis)
{
	voxeliseTrajectory();
	m_grid_ngr->reset();
	for (auto& voxel_list : m_traj_voxels) {
		m_grid_ngr->addPointsToField(voxel_list);
	}

	if (vis) {
		// SV_SHOW_INFO(m_grid_ngr->getBoundingBoxVisualization());
		// SV_SHOW_INFO(m_grid_ngr->getDistanceFieldVisualization());
		SV_SHOW_INFO(m_grid_ngr->getOccupiedVoxelsVisualization());
	}
}

bool Robot::PlanApproachOnly(const std::vector<Object*>& movable_obstacles)
{
	std::vector<std::vector<double> > dummy;
	moveit_msgs::MotionPlanResponse res;
	if (!planApproach(dummy, res, movable_obstacles)) {
		return false;
	}

	m_traj = res.trajectory.joint_trajectory;
	return true;
}

bool Robot::PlanRetrieval(const std::vector<Object*>& movable_obstacles, bool finalise, smpl::RobotState start_state)
{
	///////////////////
	// Plan approach //
	///////////////////

	std::vector<std::vector<double> > dummy;
	moveit_msgs::MotionPlanResponse res_a;
	if (!planApproach(dummy, res_a, movable_obstacles, finalise, start_state)) {
		return false;
	}

	m_traj = res_a.trajectory.joint_trajectory;

	//////////////////
	// Append grasp //
	//////////////////

	int grasp_at = m_traj.points.size();

	double grasp_time = profileAction(m_pregrasp_state, m_grasp_state);
	double postgrasp_time = profileAction(m_grasp_state, m_postgrasp_state);
	// double retreat_time = profileAction(m_postgrasp_state, m_pregrasp_state);

	trajectory_msgs::JointTrajectoryPoint p;
	p.positions = m_grasp_state;
	p.time_from_start = ros::Duration(grasp_time) + m_traj.points.back().time_from_start;
	m_traj.points.push_back(p);

	p.positions = m_postgrasp_state;
	p.time_from_start = ros::Duration(postgrasp_time) + m_traj.points.back().time_from_start;
	m_traj.points.push_back(p);

	/////////////////////
	// Plan extraction //
	/////////////////////

	moveit_msgs::MotionPlanResponse res_r;
	if (!planRetract(dummy, res_r, movable_obstacles, finalise)) {
		return false;
	}

	m_grasp_at = grasp_at;
	auto extract_start_time = m_traj.points.back().time_from_start;
	for (size_t i = 0; i < res_r.trajectory.joint_trajectory.points.size(); ++i)
	{
		auto& wp = res_r.trajectory.joint_trajectory.points[i];
		wp.time_from_start += extract_start_time;
		m_traj.points.push_back(wp);
	}

	return true;
}

bool Robot::SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f)
{
	expands = 0;
	min_f = 0;
	// CBS TODO: must pick out constraints wrt planning phase:
	// (i) for planning to pregrasp, all constraints with time <= m_grasp_at
	// are active
	// (ii) for planning to home, all constraints with times >= m_grasp_at + 2
	// are active

	// Get relevant constraints - check logic per above
	std::list<std::shared_ptr<Constraint> > approach_constraints, retract_constraints;
	for (auto& constraint : ct_node->m_constraints)
	{
		if (constraint->m_me == ct_node->m_replanned)
		{
			if (constraint->m_time <= m_grasp_at) {
				approach_constraints.push_back(constraint);
			}
			else if (constraint->m_time >= m_grasp_at + 2)
			{
				std::shared_ptr<Constraint> new_constraint(new Constraint());
				new_constraint->m_me = constraint->m_me;
				new_constraint->m_other = constraint->m_other;
				new_constraint->m_time = constraint->m_time - (m_grasp_at + 2);
				new_constraint->m_q = constraint->m_q;
				retract_constraints.push_back(new_constraint);
			}
		}
	}

	std::vector<std::vector<double> > approach_cvecs, retract_cvecs;
	VecConstraints(approach_constraints, approach_cvecs);
	VecConstraints(retract_constraints, retract_cvecs);
	approach_constraints.clear();
	retract_constraints.clear();

	///////////////////
	// Plan approach //
	///////////////////

	moveit_msgs::MotionPlanResponse res_a;
	std::vector<Object*> dummy;
	if (!planApproach(approach_cvecs, res_a, dummy)) {
		return false;
	}

	auto planner_stats = m_planner->getPlannerStats();
	m_stats["approach_plan_time"] = planner_stats["initial solution planning time"];
	expands = planner_stats["expansions"];
	min_f = planner_stats["min f val"];
	m_traj = res_a.trajectory.joint_trajectory;

	//////////////////
	// Append grasp //
	//////////////////

	int grasp_at = m_traj.points.size();

	double grasp_time = profileAction(m_pregrasp_state, m_grasp_state);
	double postgrasp_time = profileAction(m_grasp_state, m_postgrasp_state);
	// double retreat_time = profileAction(m_postgrasp_state, m_pregrasp_state);

	trajectory_msgs::JointTrajectoryPoint p;
	p.positions = m_grasp_state;
	p.time_from_start = ros::Duration(grasp_time) + m_traj.points.back().time_from_start;
	m_traj.points.push_back(p);

	p.positions = m_postgrasp_state;
	p.time_from_start = ros::Duration(postgrasp_time) + m_traj.points.back().time_from_start;
	m_traj.points.push_back(p);

	/////////////////////
	// Plan extraction //
	/////////////////////

	moveit_msgs::MotionPlanResponse res_r;
	if (!planRetract(retract_cvecs, res_r, dummy)) {
		return false;
	}

	planner_stats = m_planner->getPlannerStats();
	m_stats["extract_plan_time"] = planner_stats["initial solution planning time"];
	expands += planner_stats["expansions"];
	min_f = std::min(min_f, (int)planner_stats["min f val"]);

	m_grasp_at = grasp_at;
	auto extract_start_time = m_traj.points.back().time_from_start;
	for (size_t i = 0; i < res_r.trajectory.joint_trajectory.points.size(); ++i)
	{
		auto& wp = res_r.trajectory.joint_trajectory.points[i];
		wp.time_from_start += extract_start_time;
		m_traj.points.push_back(wp);
	}

	/////////////////////////
	// Update CBS solution //
	/////////////////////////

	m_solve.clear();
	int t = 0;
	for (const auto& wp: m_traj.points)
	{
		LatticeState s;
		s.state = wp.positions;
		s.coord = Coord(m_rm->jointVariableCount());
		stateToCoord(s.state, s.coord);
		s.t = t;

		m_solve.push_back(s);
		++t;
	}
	*sol_path = &(this->m_solve);

	// SMPL_INFO("Robot has complete plan! m_solve.size() = %d", m_solve.size());
	// SV_SHOW_INFO_NAMED("trajectory", makePathVisualization());
	return true;
}

bool Robot::ComputeGrasps(
	const std::vector<double>& pregrasp_goal)
{
	m_home_state.clear();
	m_home_state.insert(m_home_state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());

	m_pregrasp_state.clear();
	m_grasp_state.clear();
	m_postgrasp_state.clear();

	bool success = false;
	int tries = 0;
	UpdateKDLRobot(0);

	smpl::RobotState ee_state;
	Eigen::Affine3d ee_pose;
	do
	{
		m_grasp_z = m_table_z + ((m_distD(m_rng) * 0.05) + 0.025);
		ee_pose = Eigen::Translation3d(m_ooi.desc.o_x + 0.025 * std::cos(pregrasp_goal[5]), m_ooi.desc.o_y + 0.025 * std::sin(pregrasp_goal[5]), m_grasp_z) *
						Eigen::AngleAxisd(pregrasp_goal[5], Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
	}
	while (!getStateNearPose(ee_pose, ee_state, m_grasp_state, 5) && ++tries < m_grasp_tries);

	auto* vis_name = "grasp_pose";
	SV_SHOW_INFO_NAMED(vis_name, smpl::visual::MakePoseMarkers(
		ee_pose, m_grid_i->getReferenceFrame(), vis_name));

	if (tries == m_grasp_tries)	{
		return false;
	}

	// vis_name = "grasp_state";
	// auto markers = m_cc_i->getCollisionModelVisualization(m_grasp_state);
	// for (auto& marker : markers) {
	// 	marker.ns = vis_name;
	// }
	// SV_SHOW_INFO_NAMED(vis_name, markers);

	// SMPL_INFO("Found grasp state!!");

	// compute pregrasp state
	ee_pose = m_rm->computeFK(m_grasp_state);
	ee_pose.translation().x() = pregrasp_goal[0];
	ee_pose.translation().y() = pregrasp_goal[1];

	// vis_name = "pregrasp_pose";
	// SV_SHOW_INFO_NAMED(vis_name, smpl::visual::MakePoseMarkers(
	// 	ee_pose, m_grid_i->getReferenceFrame(), vis_name));

	if (getStateNearPose(ee_pose, m_grasp_state, m_pregrasp_state, 1))
	{
		// vis_name = "pregrasp_state";
		// auto markers = m_cc_i->getCollisionModelVisualization(m_pregrasp_state);
		// for (auto& marker : markers) {
		// 	marker.ns = vis_name;
		// }
		// SV_SHOW_INFO_NAMED(vis_name, markers);

		// SMPL_INFO("Found pregrasp state!");
		ee_pose = m_rm->computeFK(m_grasp_state);
		ee_pose.translation().z() += m_grasp_lift;

		// vis_name = "postgrasp_pose";
		// SV_SHOW_INFO_NAMED(vis_name, smpl::visual::MakePoseMarkers(
		// 	ee_pose, m_grid_i->getReferenceFrame(), vis_name));

		// compute postgrasp state
		if (getStateNearPose(ee_pose, m_grasp_state, m_postgrasp_state, 1))
		{
			// vis_name = "postgrasp_state";
			// auto markers = m_cc_i->getCollisionModelVisualization(m_postgrasp_state);
			// for (auto& marker : markers) {
			// 	marker.ns = vis_name;
			// }
			// SV_SHOW_INFO_NAMED(vis_name, markers);

			// SMPL_INFO("Found postgrasp state!!!");

			if (!attachAndCheckObject(m_ooi, m_postgrasp_state)) {
				detachObject();
				return false;
			}
			detachObject();

			success = true;
		}
	}

	return success;
}

bool Robot::UpdateKDLRobot(int mode)
{
	if (mode == 0) {
		m_robot_config.chain_tip_link = m_chain_tip_link;
	}
	else {
		m_robot_config.chain_tip_link = m_robot_config.push_links.at(mode - 1);
	}

	if (!setupRobotModel() || !m_rm)
	{
		ROS_ERROR("Failed to set up Robot Model");
		return false;
	}

	if(!setReferenceStartState())
	{
		ROS_ERROR("Failed to set start state!");
		return false;
	}

	m_link_s = smpl::urdf::GetLink(&m_rm->m_robot_model, m_shoulder.c_str());
	m_link_e = smpl::urdf::GetLink(&m_rm->m_robot_model, m_elbow.c_str());
	m_link_w = smpl::urdf::GetLink(&m_rm->m_robot_model, m_wrist.c_str());
	m_link_t = smpl::urdf::GetLink(&m_rm->m_robot_model, m_tip.c_str());

	smpl::RobotState dummy;
	dummy.insert(dummy.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	m_rm->computeFK(dummy); // for reasons unknown
}

bool Robot::InitArmPlanner(bool interp)
{
	if (!m_planner_init) {
		m_ph.setParam("robot/interpolate", interp);
		return initPlanner();
	}
	else {
		return createPlanner(interp);
	}
}

bool Robot::planToPoseGoal(
	const moveit_msgs::RobotState& start_state,
	const Eigen::Affine3d& pose_goal,
	trajectory_msgs::JointTrajectory& push_traj)
{
	// create planning problem to push start pose
	InitArmPlanner(false);

	moveit_msgs::MotionPlanRequest req;
	moveit_msgs::MotionPlanResponse res;

	createPoseGoalConstraint(pose_goal, req);
	req.allowed_planning_time = 0.1;
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;
	req.num_planning_attempts = 1;
	req.planner_id = "arastar.manip.bfs";
	req.start_state = start_state;

	moveit_msgs::PlanningScene planning_scene; // completely unnecessary variable
	planning_scene.robot_state = start_state;
	// solve for path to push start pose
	if (!m_planner->init_planner(planning_scene, req, res))
	{
		// ROS_ERROR("Failed to init planner!");
		return false;
	}
	if (!m_planner->solve(req, res))
	{
		// ROS_ERROR("Failed to plan.");
		return false;
	}

	push_traj = res.trajectory.joint_trajectory;
	if (push_traj.points.size() <= 1) {
		return false;
	}
	return true;
}

bool Robot::computePushAction(
	const double time_start,
	const smpl::RobotState& jnt_positions,
	const smpl::RobotState& jnt_velocities,
	const Eigen::Affine3d& end_pose,
	trajectory_msgs::JointTrajectory& action, int& failure)
{
	// Constants
	double xy_thresh = 0.01;

	// Variables
	Eigen::Affine3d x_;
	auto q_ = jnt_positions;
	auto q_dot = jnt_velocities;
	std::vector<double> x_dot(6, 0.0), error(6, 0.0), integral(6, 0.0), derivative(6, 0.0), previous(6, 0.0);
	Eigen::Affine3d xo_ = end_pose;

	bool push_end = false;
	for(int iter = 0; iter < m_invvel_iters; iter++)
	{

		// Get difference between current EE pose and desired EE pose
		x_ = m_rm->computeFK(q_);
		auto diff = xo_ * x_.inverse();
		// auto rot = diff.rotation().eulerAngles(2, 1, 0);

		// Update P error terms
		error[0] = xo_.translation().x() - x_.translation().x();
		error[1] = xo_.translation().y() - x_.translation().y();
		error[2] = xo_.translation().z() - x_.translation().z();
		// error[3] = 0.0; // rot[2];
		// error[4] = 0.0; // rot[1];
		// error[5] = 0.0; // rot[0];

		// Update I error terms
		integral[0] += error[0] * m_dt;
		integral[1] += error[1] * m_dt;
		integral[2] += error[2] * m_dt;
		// integral[3] += error[3] * m_dt;
		// integral[4] += error[4] * m_dt;
		// integral[5] += error[5] * m_dt;

		// Update D error terms
		derivative[0] = (error[0] - previous[0]) / m_dt;
		derivative[1] = (error[1] - previous[1]) / m_dt;
		derivative[2] = (error[2] - previous[2]) / m_dt;
		// derivative[3] = (error[3] - previous[3]) / m_dt;
		// derivative[4] = (error[4] - previous[4]) / m_dt;
		// derivative[5] = (error[5] - previous[5]) / m_dt;

		// Compute Cartesian velocities
		x_dot[0] = m_Kp * error[0] + m_Ki * integral[0] + m_Kd * derivative[0];
		x_dot[1] = m_Kp * error[1] + m_Ki * integral[1] + m_Kd * derivative[1];
		x_dot[2] = m_Kp * error[2] + m_Ki * integral[2] + m_Kd * derivative[2];
		// x_dot[3] = m_Kp * error[3] + m_Ki * integral[3] + m_Kd * derivative[3];
		// x_dot[4] = m_Kp * error[4] + m_Ki * integral[4] + m_Kd * derivative[4];
		// x_dot[5] = m_Kp * error[5] + m_Ki * integral[5] + m_Kd * derivative[5];

		// Update previous error term
		previous = error;

		if (!m_rm->computeInverseVelocity(q_, x_dot, q_dot))
		{
			// SMPL_INFO("Failed to compute inverse velocity");
			failure = 1;
			return false;
		}

		// Add waypoint to action
		trajectory_msgs::JointTrajectoryPoint action_pt;
		action_pt.positions = q_;
		action_pt.time_from_start = ros::Duration(time_start + m_dt * (iter));
		action.points.push_back(std::move(action_pt));

		// has the EE reached the push_end_pose
		double dx = std::fabs(xo_.translation().x() - x_.translation().x());
		double dy = std::fabs(xo_.translation().y() - x_.translation().y());
		double dz = std::fabs(xo_.translation().z() - x_.translation().z());

		if (dx <= xy_thresh && dy <= xy_thresh && dz <= xy_thresh)
		{
			// if so, we are done
			// SMPL_INFO("SUCCESS - reached end point");
			std::vector<smpl::visual::Marker> ma;

			auto cinc = 1.0f / float(action.points.size());
			for (size_t i = 0; i < action.points.size(); ++i) {
				auto markers = m_cc_i->getCollisionModelVisualization(action.points[i].positions);

				for (auto& marker : markers) {
					auto r = 0.1f;
					auto g = cinc * (float)(action.points.size() - (i + 1));
					auto b = cinc * (float)i;
					marker.color = smpl::visual::Color{ r, g, b, 1.0f };
				}

				for (auto& m : markers) {
					ma.push_back(std::move(m));
				}
			}

			for (size_t i = 0; i < ma.size(); ++i) {
				auto& marker = ma[i];
				marker.ns = "push_action";
				marker.id = i;
			}
			SV_SHOW_INFO_NAMED("push_action", ma);

			return true;
		}

		// Move arm joints
		for(size_t i = 0; i < q_.size(); ++i) {
			// q_[i] += (q_dot[i] * m_dt);
			double limit = m_rm->velLimit(i);
			q_[i] += std::min(std::max(-limit, q_dot[i]), limit) * m_dt;
			if (m_rm->isContinuous(i)) {
				q_[i] = smpl::angles::normalize_angle(q_[i]);
			}
		}

		auto markers = m_cc_i->getCollisionRobotVisualization(q_);
		for (auto& m : markers.markers) {
			m.ns = "push_action_state";
		}
		SV_SHOW_INFO(markers);

		// TODO: check velocity limit
		// Check joint limits
		if (!m_rm->checkJointLimits(q_)) {
			failure = 2;
			return false;
		}
		if (!m_cc_i->isStateValid(q_)) {
			failure = 3;
			return false;
		}
	}

	// SMPL_INFO("Failed to reach end pose");
	failure = 1;
	return false;
}

bool Robot::PlanPush(
	const std::vector<double>& start_state,
	Agent* object, const std::vector<double>& push,
	const std::vector<Object*>& other_movables,	const comms::ObjectsPoses& rearranged,
	comms::ObjectsPoses& result,
	double& push_reward)
{
	m_push_trajs.clear();
	m_push_actions.clear();

	// required info about object being pushed
	const Trajectory* obj_traj = object->SolveTraj();
	std::vector<Object*> pushed_obj = { object->GetObject() };

	// remove objects from movable collision checker if they exist
	ProcessObstacles(pushed_obj, true, true);
	ProcessObstacles(other_movables, true, true);

	if (m_pushes_per_object == -1)
	{
		m_ph.getParam("robot/pushing/num", m_pushes_per_object);
		m_ph.getParam("robot/pushing/plan_time", m_plan_push_time);

		m_ph.param<double>("robot/pushing/control/Kp", m_Kp, 1.0);
		m_ph.param<double>("robot/pushing/control/Ki", m_Ki, 1.0);
		m_ph.param<double>("robot/pushing/control/Kd", m_Kd, 1.0);
		m_ph.param<double>("robot/pushing/control/dt", m_dt, 0.01);
		m_ph.param<int>("robot/pushing/control/iters", m_invvel_iters, 1000);
	}

	int i = 0;
	double start_time = GetTime();
	bool added = false;
	while((i < m_pushes_per_object) && (GetTime() - start_time < m_plan_push_time))
	{
		++m_stats["plan_push_calls"];

		// add all movable objects as obstacles into immovable collision checker
		// they should be at their latest positions
		if (!added)
		{
			ProcessObstacles(pushed_obj, false, false);
			ProcessObstacles(other_movables, false, false);
			added = true;
		}

		// get push start pose
		Eigen::Affine3d push_start_pose, push_end_pose;
		getPushStartPose(push, push_start_pose);
		if (m_grid_i->getDistanceFromPoint(
			push_start_pose.translation().x(),
			push_start_pose.translation().y(),
			push_start_pose.translation().z()) <= m_grid_i->resolution())
		{
			m_push_debug_data.push_back({
				push_start_pose.translation().x(),
				push_start_pose.translation().y(),
				-99.0,
				-99.0,
				-1.0});
			push_reward = -1;
			continue;
		}

		// plan path to push start pose with all other objects as obstacles
		trajectory_msgs::JointTrajectory push_traj;
		moveit_msgs::RobotState push_start_state = m_start_state;
		if (start_state.size() == m_rm->jointVariableCount())
		{
			push_start_state.joint_state.position.erase(
				push_start_state.joint_state.position.begin() + 1,
				push_start_state.joint_state.position.begin() + 1 + start_state.size());
			push_start_state.joint_state.position.insert(
				push_start_state.joint_state.position.begin() + 1,
				start_state.begin(), start_state.end());
		}
		if (!planToPoseGoal(push_start_state, push_start_pose, push_traj))
		{
			m_push_debug_data.push_back({
				push_start_pose.translation().x(),
				push_start_pose.translation().y(),
				-99.0,
				-99.0,
				0.0});
			push_reward = -1;
			continue;
		}
		++m_stats["push_samples_found"];

		if (added)
		{
			// remove all movable objects from immovable collision space
			ProcessObstacles(pushed_obj, true, false);
			ProcessObstacles(other_movables, true, false);
			added = false;
		}

		// push action parameters
		double push_frac = 1.0;
		double push_dist = EuclideanDist(obj_traj->front().state, obj_traj->back().state);
		push_end_pose = m_rm->computeFK(push_traj.points.back().positions);
		double push_at_angle = std::atan2(
				obj_traj->back().state.at(1) - push_end_pose.translation().y(),
				obj_traj->back().state.at(0) - push_end_pose.translation().x());

		// compute push action end pose
		push_end_pose.translation().x() += std::cos(push_at_angle) * (push_dist * push_frac + push[3]);
		push_end_pose.translation().y() += std::sin(push_at_angle) * (push_dist * push_frac + push[3]);
		// SV_SHOW_INFO_NAMED("push_end_pose", smpl::visual::MakePoseMarkers(
		// 	push_end_pose, m_grid_i->getReferenceFrame(), "push_end_pose"));

		// get push action trajectory via inverse velocity
		trajectory_msgs::JointTrajectory push_action;
		smpl::RobotState joint_vel(m_rm->jointVariableCount(), 0.0);
		int failure = 0;
		if (computePushAction(
				push_traj.points.back().time_from_start.toSec(),
				push_traj.points.back().positions,
				joint_vel,
				push_end_pose,
				push_action, failure))
		{
			// // collision check push action against immovable obstacles
			// if (push_action.points.size() <= 1
			// 	|| !m_cc_i->isStateValid(push_action.points[0].positions)
			// 	|| !m_cc_i->isStateValid(push_action.points.back().positions)
			// 	|| !m_cc_i->isStateToStateValid(push_action.points[0].positions, push_action.points[1].positions))
			// {
			// 	m_push_debug_data.push_back({
			// 		push_start_pose.translation().x(),
			// 		push_start_pose.translation().y(),
			// 		push_end_pose.translation().x(),
			// 		push_end_pose.translation().y(),
			// 		2.0});
			// 	push_reward = 0.1;
			// 	continue;
			// }

			// bool collides = false;
			// for (size_t wp = 1; wp < push_action.points.size(); ++wp)
			// {
			// 	auto& prev_istate = push_action.points[wp - 1].positions;
			// 	auto& curr_istate = push_action.points[wp].positions;
			// 	if (!m_cc_i->isStateToStateValid(prev_istate, curr_istate))
			// 	{
			// 		collides = true;
			// 		break;
			// 	}
			// }
			// if (collides)
			// {
			// 	m_push_debug_data.push_back({
			// 		push_start_pose.translation().x(),
			// 		push_start_pose.translation().y(),
			// 		push_end_pose.translation().x(),
			// 		push_end_pose.translation().y(),
			// 		2.0});
			// 	push_reward = 0.1;
			// 	continue;
			// }

			// collision check push action against pushed object
			// ensure that it collides
			bool collides = false;
			ProcessObstacles(pushed_obj, false, false);
			for (size_t wp = 1; wp < push_action.points.size(); ++wp)
			{
				auto& prev_istate = push_action.points[wp - 1].positions;
				auto& curr_istate = push_action.points[wp].positions;
				if (!m_cc_i->isStateToStateValid(prev_istate, curr_istate))
				{
					collides = true;
					break;
				}
			}
			if (!collides)
			{
				ProcessObstacles(pushed_obj, true, false);
				m_push_debug_data.push_back({
					push_start_pose.translation().x(),
					push_start_pose.translation().y(),
					push_end_pose.translation().x(),
					push_end_pose.translation().y(),
					4.0});
				push_reward = 0.1;
				continue;
			}
			ProcessObstacles(pushed_obj, true, false);

			++m_stats["push_actions_found"];
			m_push_debug_data.push_back({
				push_start_pose.translation().x(),
				push_start_pose.translation().y(),
				push_end_pose.translation().x(),
				push_end_pose.translation().y(),
				5.0});
			push_reward = 1;

			// append waypoints to retract to push start pose
			auto push_action_copy = push_action;
			auto t_reverse = push_action_copy.points.rbegin()->time_from_start;
			for (auto itr = push_action_copy.points.rbegin() + 1; itr != push_action_copy.points.rend(); ++itr)
			{
				itr->time_from_start = t_reverse + ros::Duration(0.01);
				push_action.points.push_back(*itr);
				t_reverse = itr->time_from_start;
			}

			m_planner->ProfilePath(m_rm.get(), push_traj);
			for (auto itr = push_action.points.begin() + 1; itr != push_action.points.end(); ++itr) {
				push_traj.points.push_back(*itr);
			}

			m_push_actions.push_back(std::move(push_action));
			m_push_trajs.push_back(std::move(push_traj));
			++i;
		}
		else
		{
			m_push_debug_data.push_back({
				push_start_pose.translation().x(),
				push_start_pose.translation().y(),
				push_end_pose.translation().x(),
				push_end_pose.translation().y(),
				(double)failure});
			push_reward = -0.5;
		}
	}

	if (added)
	{
		// remove all movable objects from immovable collision space
		ProcessObstacles(pushed_obj, true, false);
		ProcessObstacles(other_movables, true, false);
		added = false;
	}

	// reset robot model back to chain tip link
	UpdateKDLRobot(0);

	if (i == 0)
	{
		// SMPL_INFO("No pushes found! Do nothing!");
		return false;
	}
	m_stats["push_plan_time"] += GetTime() - start_time;

	int pidx, successes;
	start_time = GetTime();
	m_sim->SimPushes(m_push_actions, object->GetID(), obj_traj->back().state.at(0), obj_traj->back().state.at(1), pidx, successes, rearranged, result);
	m_stats["push_sim_time"] += GetTime() - start_time;

	if (pidx == -1)
	{
		// SMPL_WARN("Failed to find a good push for the object! Simulating took %f seconds.", time_spent);
		return false;
	}

	++m_stats["push_sim_successes"];
	m_traj = m_push_trajs.at(pidx);

	Eigen::Affine3d pose = m_rm->computeFK(m_traj.points.back().positions);
	m_push_debug_data.push_back({
		pose.translation().x(),
		pose.translation().y(),
		-99.0,
		-99.0,
		6.0});
	// SMPL_INFO("Found good push traj of length %d!", m_traj.points.size());
	return true;
}

void Robot::AnimateSolution()
{
	SV_SHOW_INFO_NAMED("trajectory", makePathVisualization());

	size_t pidx = 0;
	while (ros::ok())
	{
		auto& point = m_solve[pidx];
		auto markers = m_cc_i->getCollisionRobotVisualization(point.state);
		for (auto& m : markers.markers) {
			m.ns = "path_animation";
		}
		SV_SHOW_INFO(markers);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		pidx++;
		pidx %= m_solve.size();
	}
}

const std::vector<Object>* Robot::GetObject(const LatticeState& s)
{
	reinitObjects(s.state);
	return &m_objs;
}

State Robot::GetEEState(const State& state)
{
	Eigen::Affine3d ee_pose = m_rm->computeFK(state);
	State ee = {ee_pose.translation().x(), ee_pose.translation().y(), ee_pose.translation().z(),
				0.0, 0.0, 0.0};
	smpl::angles::get_euler_zyx(ee_pose.rotation(), ee[5], ee[4], ee[3]);
	return ee;
}

void Robot::getPushStartPose(
	const std::vector<double>& push,
	Eigen::Affine3d& push_pose)
{
	// sample robot link
	int link = 1; // std::floor(m_distD(m_rng) * (m_robot_config.push_links.size() + 1));
	UpdateKDLRobot(link);

	// z is between 3 to 8cm above table height
	double z = m_table_z + (m_distD(m_rng) * 0.05) + 0.03;

	// (x, y) is randomly sampled near push start location
	double x = push[0] + std::cos(push[2] + M_PI) * 0.05 + (m_distG(m_rng) * 0.025);
	double y = push[1] + std::sin(push[2] + M_PI) * 0.05 + (m_distG(m_rng) * 0.025);

	push_pose = Eigen::Translation3d(x, y, z) *
				Eigen::AngleAxisd(push[2], Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

	SV_SHOW_INFO_NAMED("sampled_push_pose", smpl::visual::MakePoseMarkers(
		push_pose, m_grid_i->getReferenceFrame(), "sampled_push_pose"));
}

void Robot::GetRandomState(smpl::RobotState& s)
{
	s.clear();
	s.resize(m_rm->jointVariableCount(), 0.0);

	for(int jidx = 0; jidx < m_rm->jointVariableCount(); jidx++)
	{
		if (m_continuous[jidx]) {
			s.at(jidx) = m_distD(m_rng) * 2 * M_PI;
		}
		else
		{
			auto span = std::fabs(m_max_limits[jidx] - m_min_limits[jidx]);
			s.at(jidx) = m_distD(m_rng) * span + m_min_limits[jidx];
		}
	}
}

bool Robot::reinitStartState()
{
	double x, y;
	double xmin = m_cc->OutsideXMin(), xmax = m_cc->OutsideXMax() - 0.05;
	double ymin = m_cc->OutsideYMin(), ymax = m_cc->OutsideYMax();

	Eigen::Affine3d ee_pose;
	smpl::RobotState s, seed;
	seed.insert(seed.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	do
	{
		x = (m_distD(m_rng) * (xmax - xmin)) + xmin;
		y = (m_distD(m_rng) * (ymax - ymin)) + ymin;
		ee_pose = Eigen::Translation3d(x, y, m_table_z + 0.05) *
					Eigen::AngleAxisd((m_distD(m_rng) * M_PI_2) - M_PI_4, Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(((2 * m_distD(m_rng)) - 1) * DEG5, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(((2 * m_distD(m_rng)) - 1) * DEG5, Eigen::Vector3d::UnitX());

		if (m_rm->computeIKSearch(ee_pose, seed, s))
		{
			if (m_rm->checkJointLimits(s)) {
				break;
			}
		}
		GetRandomState(seed);
	}
	while (true);

	m_start_state.joint_state.position.erase(
		m_start_state.joint_state.position.begin() + 1,
		m_start_state.joint_state.position.begin() + 1 + s.size());
	m_start_state.joint_state.position.insert(
		m_start_state.joint_state.position.begin() + 1,
		s.begin(), s.end());

	if(!setReferenceStartState())
	{
		ROS_ERROR("Failed to set reinit-ed start state!");
		return false;
	}

	return true;
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
void Robot::coordToState(
	const Coord& coord,
	State& state) const
{
	assert((int)state.size() == m_rm->jointVariableCount() &&
			(int)coord.size() == m_rm->jointVariableCount());

	for (size_t i = 0; i < coord.size(); ++i)
	{
		if (m_continuous[i]) {
			state[i] = coord[i] * m_coord_deltas[i];
		}
		else if (!m_bounded[i]) {
			state[i] = (double)coord[i] * m_coord_deltas[i];
		}
		else {
			state[i] = m_min_limits[i] + coord[i] * m_coord_deltas[i];
		}
	}
}

void Robot::stateToCoord(
	const State& state,
	Coord& coord) const
{
	assert((int)state.size() == m_rm->jointVariableCount() &&
			(int)coord.size() == m_rm->jointVariableCount());

	for (size_t i = 0; i < state.size(); ++i)
	{
		if (m_continuous[i])
		{
			auto pos_angle = smpl::angles::normalize_angle_positive(state[i]);

			coord[i] = (int)((pos_angle + m_coord_deltas[i] * 0.5) / m_coord_deltas[i]);

			if (coord[i] == m_coord_vals[i]) {
				coord[i] = 0;
			}
		}
		else if (!m_bounded[i])
		{
			if (state[i] >= 0.0) {
				coord[i] = (int)(state[i] / m_coord_deltas[i] + 0.5);
			}
			else {
				coord[i] = (int)(state[i] / m_coord_deltas[i] - 0.5);
			}
		}
		else {
			coord[i] = (int)(((state[i] - m_min_limits[i]) / m_coord_deltas[i]) + 0.5);
		}
	}
}

bool Robot::readRobotModelConfig(const ros::NodeHandle &nh)
{
	if (!nh.getParam("group_name", m_robot_config.group_name)) {
		ROS_ERROR("Failed to read 'group_name' from the param server");
		return false;
	}

	std::string config_field;
	if (!nh.getParam("planning_joints", config_field)) {
		ROS_ERROR("Failed to read 'planning_joints' from the param server");
		return false;
	}

	std::stringstream joint_name_stream(config_field);
	while (joint_name_stream.good() && !joint_name_stream.eof()) {
		std::string jname;
		joint_name_stream >> jname;
		if (jname.empty()) {
			continue;
		}
		m_robot_config.planning_joints.push_back(jname);
	}

	config_field.clear();
	if (!nh.getParam("push_links", config_field)) {
		ROS_ERROR("Failed to read 'push_links' from the param server");
		return false;
	}

	joint_name_stream.str("");
	joint_name_stream.clear();
	joint_name_stream.str(config_field);
	while (joint_name_stream.good() && !joint_name_stream.eof()) {
		std::string jname;
		joint_name_stream >> jname;
		if (jname.empty()) {
			continue;
		}
		m_robot_config.push_links.push_back(jname);
	}

	config_field.clear();
	if (!nh.getParam("gripper_joints", config_field)) {
		ROS_ERROR("Failed to read 'gripper_joints' from the param server");
		return false;
	}

	joint_name_stream.str("");
	joint_name_stream.clear();
	joint_name_stream.str(config_field);
	while (joint_name_stream.good() && !joint_name_stream.eof()) {
		std::string jname;
		joint_name_stream >> jname;
		if (jname.empty()) {
			continue;
		}
		m_robot_config.gripper_joints.push_back(jname);
	}

	// only required for generic kdl robot model?
	nh.getParam("kinematics_frame", m_robot_config.kinematics_frame);
	nh.getParam("chain_tip_link", m_robot_config.chain_tip_link);
	m_chain_tip_link = m_robot_config.chain_tip_link;
	return true;
}

bool Robot::setupRobotModel()
{
	if (m_robot_config.kinematics_frame.empty() || m_robot_config.chain_tip_link.empty()) {
		ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
		m_rm = NULL;
		return false;
	}

	m_rm = std::make_unique<smpl::PushingKDLRobotModel>();

	if (!m_rm->init(m_robot_description, m_robot_config.kinematics_frame, m_robot_config.chain_tip_link)) {
		ROS_ERROR("Failed to initialize robot model.");
		m_rm = NULL;
		return false;
	}

	return true;
}

bool Robot::readStartState()
{
	XmlRpc::XmlRpcValue xlist;

	// joint_state
	if (m_ph.hasParam("initial_configuration/joint_state")) {
		m_ph.getParam("initial_configuration/joint_state", xlist);

		if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
			ROS_WARN("initial_configuration/joint_state is not an array.");
		}

		if (xlist.size() > 0) {
			for (int i = 0; i < xlist.size(); ++i) {
				m_start_state.joint_state.name.push_back(std::string(xlist[i]["name"]));

				if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
					m_start_state.joint_state.position.push_back(double(xlist[i]["position"]));
				}
				else {
					ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
					if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
						int pos = xlist[i]["position"];
						m_start_state.joint_state.position.push_back(double(pos));
					}
				}
			}
		}
	}
	else {
		ROS_WARN("initial_configuration/joint_state is not on the param server.");
	}

	// multi_dof_joint_state
	if (m_ph.hasParam("initial_configuration/multi_dof_joint_state")) {
		m_ph.getParam("initial_configuration/multi_dof_joint_state", xlist);

		if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
			if (xlist.size() != 0) {
				auto &multi_dof_joint_state = m_start_state.multi_dof_joint_state;
				multi_dof_joint_state.joint_names.resize(xlist.size());
				multi_dof_joint_state.transforms.resize(xlist.size());
				for (int i = 0; i < xlist.size(); ++i) {
					multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

					Eigen::Quaterniond q;
					smpl::angles::from_euler_zyx(
							(double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

					geometry_msgs::Quaternion orientation;
					tf::quaternionEigenToMsg(q, orientation);

					multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
					multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
					multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
					multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
					multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
					multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
					multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
				}
			} else {
				ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
			}
		} else {
			ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
		}
	}

	return true;
}

bool Robot::setReferenceStartState()
{
	// Set reference state in the robot planning model...
	// <sbpl_kdl_robot_model/kdl_robot_model.h> <-
	// 	<smpl_urdf_robot_model/smpl_urdf_robot_model.h> <-
	// 		<smpl_urdf_robot_model/robot_state.h>
	smpl::urdf::RobotState reference_state;
	InitRobotState(&reference_state, &m_rm->m_robot_model);
	for (auto i = 0; i < m_start_state.joint_state.name.size(); ++i) {
		auto* var = GetVariable(&m_rm->m_robot_model, &m_start_state.joint_state.name[i]);
		if (var == NULL) {
			ROS_WARN("Failed to do the thing");
			continue;
		}
		// ROS_INFO("Set joint %s to %f", m_start_state.joint_state.name[i].c_str(), m_start_state.joint_state.position[i]);
		SetVariablePosition(&reference_state, var, m_start_state.joint_state.position[i]);
	}
	SetReferenceState(m_rm.get(), GetVariablePositions(&reference_state));

	// Set reference state in the collision scene here...
	setCollisionRobotState();

	return true;
}

bool Robot::setGripper(bool open)
{
	smpl::urdf::RobotState reference_state;
	InitRobotState(&reference_state, &m_rm->m_robot_model);
	for (auto i = 0; i < m_robot_config.gripper_joints.size(); ++i) {
		auto* var = GetVariable(&m_rm->m_robot_model, &m_robot_config.gripper_joints[i]);
		if (var == NULL) {
			ROS_WARN("Failed to do the thing");
			continue;
		}
		// ROS_INFO("Set joint %s to %f", m_start_state.joint_state.name[i].c_str(), m_start_state.joint_state.position[i]);
		double pos = open ? 0.5 : 0.0; // from gripper.urdf.xacro, limit is actually 0.548
		SetVariablePosition(&reference_state, var, pos);

		// Set state in the collision scene here...
		auto& joint_name = m_robot_config.gripper_joints[i];
		if (!m_cc_i->setJointPosition(joint_name, pos) || !m_cc_m->setJointPosition(joint_name, pos)) {
			ROS_ERROR("Failed to set position of joint '%s' to %f", joint_name.c_str(), pos);
			return false;
		}
	}
	SetReferenceState(m_rm.get(), GetVariablePositions(&reference_state));
}

bool Robot::readResolutions(std::vector<double>& resolutions)
{
	std::string disc_string;
	if (!m_ph.getParam("planning/discretization", disc_string))
	{
		SMPL_ERROR("Parameter 'discretization' not found in planning params");
		return false;
	}

	auto disc = ParseMapFromString<double>(disc_string);
	for (size_t vidx = 0; vidx < m_rm->jointVariableCount(); ++vidx)
	{
		auto& vname = m_rm->getPlanningJoints()[vidx];
		std::string joint_name, local_name;
		if (smpl::IsMultiDOFJointVariable(vname, &joint_name, &local_name))
		{
			// adjust variable name if a variable of a multi-dof joint
			auto mdof_vname = joint_name + "_" + local_name;
			auto dit = disc.find(mdof_vname);
			if (dit == end(disc))
			{
				SMPL_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
				return false;
			}
			resolutions[vidx] = dit->second;
		}
		else
		{
			auto dit = disc.find(vname);
			if (dit == end(disc))
			{
				SMPL_ERROR("Discretization for variable '%s' not found in planning parameters", vname.c_str());
				return false;
			}
			resolutions[vidx] = dit->second;
		}
	}

	return true;
}

void Robot::initObjects()
{
	Object o;
	o.desc.shape = 0; // rectangle
	o.desc.type = 1; // movable
	o.desc.o_x = 0.0; // TBD
	o.desc.o_y = 0.0; // TBD
	o.desc.o_z = 0.0; // NA
	o.desc.o_roll = 0.0; // NA
	o.desc.o_pitch = 0.0; // NA
	o.desc.o_yaw = 0.0; // TBD
	o.desc.x_size = 0.0; // TBD
	o.desc.y_size = 0.0; // TBD
	o.desc.z_size = 0.0; // NA
	o.desc.mass = 1.0;
	o.desc.locked = o.desc.mass == 0;
	o.desc.mu = 0.8;
	o.desc.movable = true;

	// shoulder-elbow rectangle
	o.desc.id = 100;
	m_objs.push_back(o);

	// elbow-wrist rectangle
	o.desc.id = 101;
	m_objs.push_back(o);

	// wrist-tip rectangle
	o.desc.id = 102;
	m_objs.push_back(o);
}

void Robot::reinitObjects(const State& s)
{
	// shoulder-elbow rectangle
	auto link_pose_s = m_rm->computeFKLink(s, m_link_s);
	auto link_pose_e = m_rm->computeFKLink(s, m_link_e);
	State F1 = {link_pose_s.translation().x(), link_pose_s.translation().y()};
	State F2 = {link_pose_e.translation().x(), link_pose_e.translation().y()};
	ArmRectObj(F1, F2, m_b, m_objs.at(0).desc);

	// elbow-wrist rectangle
	auto link_pose_w = m_rm->computeFKLink(s, m_link_w);
	F1 = F2;
	F2.at(0) = link_pose_w.translation().x();
	F2.at(1) = link_pose_w.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(1).desc);

	// wrist-tip rectangle
	auto link_pose_t = m_rm->computeFKLink(s, m_link_t);
	F1 = F2;
	F2.at(0) = link_pose_t.translation().x();
	F2.at(1) = link_pose_t.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(2).desc);
}

double Robot::profileAction(
	const smpl::RobotState& parent, const smpl::RobotState& succ)
{
	double max_time = 0;
	for(int jidx = 0; jidx < m_rm->jointVariableCount(); jidx++)
	{
		auto from_pos = parent[jidx];
		auto to_pos = succ[jidx];
		auto vel = m_rm->velLimit(jidx) * R_SPEED;
		if (vel <= 0.0) {
			continue;
		}
		auto t = 0.0;
		if (m_rm->isContinuous(jidx)) {
			t = smpl::angles::shortest_angle_dist(from_pos, to_pos) / vel;
		} else {
			t = std::fabs(to_pos - from_pos) / vel;
		}

		max_time = std::max(max_time, t);
	}
	return max_time;
}

void Robot::initOccupancyGrids()
{
	double sx, sy, sz, ox, oy, oz, max_distance;

	m_ph.getParam("occupancy_grid/size_x", sx);
	m_ph.getParam("occupancy_grid/size_y", sy);
	m_ph.getParam("occupancy_grid/size_z", sz);
	m_ph.getParam("occupancy_grid/origin_x", ox);
	m_ph.getParam("occupancy_grid/origin_y", oy);
	m_ph.getParam("occupancy_grid/origin_z", oz);
	m_ph.getParam("occupancy_grid/max_dist", max_distance);
	m_ph.getParam("planning_frame", m_planning_frame);

	m_df_i = std::make_shared<smpl::EuclidDistanceMap>(
			ox, oy, oz,
			sx, sy, sz,
			DF_RES,
			max_distance);
	m_df_m = std::make_shared<smpl::EuclidDistanceMap>(
			ox, oy, oz,
			sx, sy, sz,
			DF_RES,
			max_distance);

	bool ref_counted = false;
	m_grid_i = std::make_unique<smpl::OccupancyGrid>(m_df_i, ref_counted);
	m_grid_i->setReferenceFrame(m_planning_frame);
	m_grid_m = std::make_unique<smpl::OccupancyGrid>(m_df_m, ref_counted);
	m_grid_m->setReferenceFrame(m_planning_frame);
	// SV_SHOW_INFO(m_grid_i->getBoundingBoxVisualization());

	bool propagate_negative_distances = true;
	m_df_ngr = std::make_shared<smpl::PropagationDistanceField>(
			ox, oy, oz,
			sx, sy, sz,
			DF_RES,
			max_distance, propagate_negative_distances);

	m_grid_ngr = std::make_unique<smpl::OccupancyGrid>(m_df_ngr, ref_counted);
	m_grid_ngr->setReferenceFrame(m_planning_frame);
}

bool Robot::initCollisionChecker()
{
	smpl::collision::CollisionModelConfig cc_conf;
	if (!smpl::collision::CollisionModelConfig::Load(m_ph, cc_conf)) {
		ROS_ERROR("Failed to load Collision Model Config");
		return false;
	}

	// immovable obstacle collision checker
	m_cc_i = std::make_unique<smpl::collision::CollisionSpace>();
	if (!m_cc_i->init(
			m_grid_i.get(),
			m_robot_description,
			cc_conf,
			m_robot_config.group_name,
			m_robot_config.planning_joints))
	{
		ROS_ERROR("Failed to initialize immovable Collision Space");
		return false;
	}

	if (m_cc_i->robotCollisionModel()->name() == "pr2") {
		// sbpl_collision_checking/types.h
		smpl::collision::AllowedCollisionMatrix acm;
		for (auto& pair : PR2AllowedCollisionPairs) {
			acm.setEntry(pair.first, pair.second, true);
		}
		m_cc_i->setAllowedCollisionMatrix(acm);
	}

	// movable objecet collision checker
	m_cc_m = std::make_unique<smpl::collision::CollisionSpace>();
	if (!m_cc_m->init(
			m_grid_m.get(),
			m_robot_description,
			cc_conf,
			m_robot_config.group_name,
			m_robot_config.planning_joints))
	{
		ROS_ERROR("Failed to initialize movable Collision Space");
		return false;
	}

	if (m_cc_m->robotCollisionModel()->name() == "pr2") {
		// sbpl_collision_checking/types.h
		smpl::collision::AllowedCollisionMatrix acm;
		for (auto& pair : PR2AllowedCollisionPairs) {
			acm.setEntry(pair.first, pair.second, true);
		}
		m_cc_m->setAllowedCollisionMatrix(acm);
	}

	return true;
}

bool Robot::getCollisionObjectMsg(
			const Object& object,
			moveit_msgs::CollisionObject& obj_msg,
			bool remove)
{
	object.GetMoveitObj(obj_msg);
	obj_msg.operation = remove ? moveit_msgs::CollisionObject::REMOVE :
										moveit_msgs::CollisionObject::ADD;

	if (remove) {
		return true;
	}

	obj_msg.header.frame_id = m_planning_frame;
	obj_msg.header.stamp = ros::Time::now();

	return true;
}

/// \brief Process a collision object
/// \param object The collision object to be processed
/// \return true if the object was processed successfully; false otherwise
bool Robot::processCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object,
	bool movable)
{
	if (object.operation == moveit_msgs::CollisionObject::ADD) {
		return addCollisionObjectMsg(object, movable);
	}
	else if (object.operation == moveit_msgs::CollisionObject::REMOVE) {
		return removeCollisionObjectMsg(object, movable);
	}
	// else if (object.operation == moveit_msgs::CollisionObject::APPEND) {
	// 	return AppendCollisionObjectMsg(object);
	// }
	// else if (object.operation == moveit_msgs::CollisionObject::MOVE) {
	// 	return MoveCollisionObjectMsg(object);
	// }
	else {
		return false;
	}
}

bool Robot::processSTLMesh(
	const Object& object, bool remove, bool movable)
{
	auto cc = movable ? m_cc_m.get() : m_cc_i.get();
	if (remove)
	{
		// find the collision object with this name
		auto* _object = findCollisionObject(std::to_string(object.desc.id), movable);
		if (!_object) {
			return false;
		}

		// remove from collision space
		if (!cc->removeObject(_object)) {
			return false;
		}

		// remove all collision shapes belonging to this object
		auto belongs_to_object = [_object](const std::unique_ptr<smpl::collision::CollisionShape>& shape) {
			auto is_shape = [&shape](const smpl::collision::CollisionShape* s) {
				return s == shape.get();
			};
			auto it = std::find_if(
					begin(_object->shapes), end(_object->shapes), is_shape);
			return it != end(_object->shapes);
		};

		auto rit = std::remove_if(
				begin(cc->m_collision_shapes), end(cc->m_collision_shapes),
				belongs_to_object);
		cc->m_collision_shapes.erase(rit, end(cc->m_collision_shapes));

		// remove the object itself
		auto is_object = [_object](const std::unique_ptr<smpl::collision::CollisionObject>& obj) {
			return obj.get() == _object;
		};
		auto rrit = std::remove_if(
				begin(cc->m_collision_objects), end(cc->m_collision_objects), is_object);
		cc->m_collision_objects.erase(rrit, end(cc->m_collision_objects));

		return true;
	}

	moveit_msgs::CollisionObject obj_msg;
	object.GetMoveitObj(obj_msg);
	auto object_mesh = obj_msg.meshes[0];

	auto vertices = new double[3 * object_mesh.vertices.size()];
	auto triangles = new std::uint32_t[3 * object_mesh.triangles.size()];

	for (size_t i = 0; i < object_mesh.vertices.size(); ++i) {
		vertices[3 * i + 0] = object_mesh.vertices[i].x;
		vertices[3 * i + 1] = object_mesh.vertices[i].y;
		vertices[3 * i + 2] = object_mesh.vertices[i].z;
	}

	for (size_t i = 0; i < object_mesh.triangles.size(); ++i) {
		triangles[3 * i + 0] = object_mesh.triangles[i].vertex_indices[0];
		triangles[3 * i + 1] = object_mesh.triangles[i].vertex_indices[1];
		triangles[3 * i + 2] = object_mesh.triangles[i].vertex_indices[2];
	}

	smpl::collision::MeshShape* shape = new smpl::collision::MeshShape;
	shape->vertices = vertices;
	shape->triangles = triangles;
	shape->vertex_count = object_mesh.vertices.size();
	shape->triangle_count = object_mesh.triangles.size();

	Eigen::Affine3d transform;
	tf::poseMsgToEigen(obj_msg.mesh_poses[0], transform);

	std::vector<smpl::collision::CollisionShape*> shapes;
	smpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;

	shapes.push_back(shape);
	shape_poses.push_back(transform);

	auto co = smpl::make_unique<smpl::collision::CollisionObject>();
	co->id = obj_msg.id;
	co->shapes = std::move(shapes);
	co->shape_poses = std::move(shape_poses);

	cc->m_collision_objects.push_back(std::move(co));
	return cc->insertObject(cc->m_collision_objects.back().get());
}

bool Robot::addCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object,
	bool movable)
{
	auto cc = movable ? m_cc_m.get() : m_cc_i.get();

	if (cc->worldCollisionModel()->hasObjectWithName(object.id))
	{
		// find the collision object with this name
		auto* cc_object = findCollisionObject(object.id, movable);
		if (!cc_object) {
			return false;
		}

		Eigen::Affine3d pose;
		if (!object.primitives.empty()) {
			tf::poseMsgToEigen(object.primitive_poses[0], pose);
		}
		else if (!object.meshes.empty()) {
			tf::poseMsgToEigen(object.mesh_poses[0], pose);
		}

		if (cc_object->shape_poses[0].translation() == pose.translation()
			&& cc_object->shape_poses[0].rotation() == pose.rotation()) {
			return true;
		}
		else
		{
			cc_object->shape_poses[0] = pose;
			return cc->moveShapes(cc_object);
		}
	}

	if (object.header.frame_id != cc->getReferenceFrame()) {
		ROS_ERROR("Collision object must be specified in the Collision Space's reference frame (%s)", cc->getReferenceFrame().c_str());
		return false;
	}

	if (!checkCollisionObjectSanity(object)) {
		return false;
	}

	std::vector<smpl::collision::CollisionShape*> shapes;
	smpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;

	for (size_t i = 0; i < object.primitives.size(); ++i) {
		auto& prim = object.primitives[i];

		std::unique_ptr<smpl::collision::CollisionShape> shape;
		switch (prim.type) {
		case shape_msgs::SolidPrimitive::BOX:
			shape = smpl::make_unique<smpl::collision::BoxShape>(
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_X],
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
			break;
		case shape_msgs::SolidPrimitive::SPHERE:
			shape = smpl::make_unique<smpl::collision::SphereShape>(
					prim.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]);
			break;
		case shape_msgs::SolidPrimitive::CYLINDER:
			shape = smpl::make_unique<smpl::collision::CylinderShape>(
					prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
					prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
			break;
		case shape_msgs::SolidPrimitive::CONE:
			shape = smpl::make_unique<smpl::collision::ConeShape>(
					prim.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
					prim.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]);
			break;
		default:
			assert(0);
		}

		cc->m_collision_shapes.push_back(std::move(shape));
		shapes.push_back(cc->m_collision_shapes.back().get());

		auto& prim_pose = object.primitive_poses[i];
		Eigen::Affine3d transform;
		tf::poseMsgToEigen(prim_pose, transform);
		shape_poses.push_back(transform);
	}

	for (size_t i = 0; i < object.meshes.size(); ++i) {
		auto& mesh = object.meshes[i];

		assert(0); // TODO: implement

		auto& mesh_pose = object.mesh_poses[i];
		Eigen::Affine3d transform;
		tf::poseMsgToEigen(mesh_pose, transform);
		shape_poses.push_back(transform);
	}

	// create the collision object
	auto co = smpl::make_unique<smpl::collision::CollisionObject>();
	co->id = object.id;
	co->shapes = std::move(shapes);
	co->shape_poses = std::move(shape_poses);

	cc->m_collision_objects.push_back(std::move(co));
	return cc->insertObject(cc->m_collision_objects.back().get());
}

bool Robot::removeCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object,
	bool movable)
{
	auto cc = movable ? m_cc_m.get() : m_cc_i.get();

	// find the collision object with this name
	auto* _object = findCollisionObject(object.id, movable);
	if (!_object) {
		return false;
	}

	// remove from collision space
	if (!cc->removeObject(_object)) {
		return false;
	}

	// remove all collision shapes belonging to this object
	auto belongs_to_object = [_object](const std::unique_ptr<smpl::collision::CollisionShape>& shape) {
		auto is_shape = [&shape](const smpl::collision::CollisionShape* s) {
			return s == shape.get();
		};
		auto it = std::find_if(
				begin(_object->shapes), end(_object->shapes), is_shape);
		return it != end(_object->shapes);
	};

	auto rit = std::remove_if(
			begin(cc->m_collision_shapes), end(cc->m_collision_shapes),
			belongs_to_object);
	cc->m_collision_shapes.erase(rit, end(cc->m_collision_shapes));

	// remove the object itself
	auto is_object = [_object](const std::unique_ptr<smpl::collision::CollisionObject>& object) {
		return object.get() == _object;
	};
	auto rrit = std::remove_if(
			begin(cc->m_collision_objects), end(cc->m_collision_objects), is_object);
	cc->m_collision_objects.erase(rrit, end(cc->m_collision_objects));

	return true;
}

bool Robot::checkCollisionObjectSanity(
	const moveit_msgs::CollisionObject& object) const
{
	if (object.primitives.size() != object.primitive_poses.size()) {
		ROS_ERROR("Mismatched sizes of primitives and primitive poses");
		return false;
	}

	if (object.meshes.size() != object.mesh_poses.size()) {
		ROS_ERROR("Mismatches sizes of meshes and mesh poses");
		return false;
	}

	// check solid primitive for correct format
	for (auto& prim : object.primitives) {
		switch (prim.type) {
		case shape_msgs::SolidPrimitive::BOX:
		{
			if (prim.dimensions.size() != 3) {
				ROS_ERROR("Invalid number of dimensions for box of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 3, prim.dimensions.size());
				return false;
			}
		}   break;
		case shape_msgs::SolidPrimitive::SPHERE:
		{
			if (prim.dimensions.size() != 1) {
				ROS_ERROR("Invalid number of dimensions for sphere of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 1, prim.dimensions.size());
				return false;
			}
		}   break;
		case shape_msgs::SolidPrimitive::CYLINDER:
		{
			if (prim.dimensions.size() != 2) {
				ROS_ERROR("Invalid number of dimensions for cylinder of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 2, prim.dimensions.size());
				return false;
			}
		}   break;
		case shape_msgs::SolidPrimitive::CONE:
		{
			if (prim.dimensions.size() != 2) {
				ROS_ERROR("Invalid number of dimensions for cone of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 2, prim.dimensions.size());
				return false;
			}
		}   break;
		default:
			ROS_ERROR("Unrecognized SolidPrimitive type");
			return false;
		}
	}

	return true;
}

auto Robot::findCollisionObject(const std::string& id, bool movable) const
	-> smpl::collision::CollisionObject*
{
	auto cc = movable ? m_cc_m.get() : m_cc_i.get();

	for (auto& object : cc->m_collision_objects) {
		if (object->id == id) {
			return object.get();
		}
	}
	return nullptr;
}

bool Robot::setCollisionRobotState()
{
	if (m_start_state.joint_state.name.size() != m_start_state.joint_state.position.size()) {
		ROS_ERROR("Joint state contains mismatched number of joint names and joint positions");
		return false;
	}

	for (size_t i = 0; i < m_start_state.joint_state.name.size(); ++i)
	{
		auto& joint_name = m_start_state.joint_state.name[i];
		double joint_position = m_start_state.joint_state.position[i];
		if (!m_cc_i->setJointPosition(joint_name, joint_position)) {
			ROS_ERROR("Failed to set position of joint '%s' to %f", joint_name.c_str(), joint_position);
			return false;
		}
		if (!m_cc_m->setJointPosition(joint_name, joint_position)) {
			ROS_ERROR("Failed to set position of joint '%s' to %f", joint_name.c_str(), joint_position);
			return false;
		}
	}

	auto& multi_dof_joint_state = m_start_state.multi_dof_joint_state;
	for (size_t i = 0; i < multi_dof_joint_state.joint_names.size(); ++i) {
		auto& joint_name = multi_dof_joint_state.joint_names[i];
		auto& joint_transform = multi_dof_joint_state.transforms[i];
		// TODO: Need a way to identify what type of joint this is to extract
		// its condensed set of variables. Might be worth adding a function to
		// CollisionSpace to handle setting any joint from its local transform.
	}

	// TODO: world -> model transform should be handled by multi_dof joint
	// state.

	// // TODO: ProcessAttachedCollisionObject will need to be copied from CollisionSpaceScene
	// auto& attached_collision_objects = m_start_state.attached_collision_objects;
	// for (auto& aco : attached_collision_objects) {
	// 	if (!ProcessAttachedCollisionObject(aco)) {
	// 		ROS_WARN_NAMED(LOG, "Failed to process attached collision object");
	// 		return false;
	// 	}
	// }

	return true;
}

auto Robot::makePathVisualization() const
	-> std::vector<smpl::visual::Marker>
{
	std::vector<smpl::visual::Marker> ma;

	if (m_solve.empty()) {
		return ma;
	}

	auto cinc = 1.0f / float(m_solve.size());
	for (size_t i = 0; i < m_solve.size(); ++i) {
		auto markers = m_cc_i->getCollisionModelVisualization(m_solve[i].state);

		for (auto& marker : markers) {
			auto r = 0.1f;
			auto g = cinc * (float)(m_solve.size() - (i + 1));
			auto b = cinc * (float)i;
			marker.color = smpl::visual::Color{ r, g, b, 1.0f };
		}

		for (auto& m : markers) {
			ma.push_back(std::move(m));
		}
	}

	for (size_t i = 0; i < ma.size(); ++i) {
		auto& marker = ma[i];
		marker.ns = "trajectory";
		marker.id = i;
	}

	return ma;
}

bool Robot::initPlanner()
{
	if (!readPlannerConfig(ros::NodeHandle("~planning"))) {
		ROS_ERROR("Failed to read planner config");
		return false;
	}

	m_planning_params.addParam("discretization", m_planning_config.discretization);
	m_planning_params.addParam("mprim_filename", m_planning_config.mprim_filename);
	m_planning_params.addParam("use_xyz_snap_mprim", m_planning_config.use_xyz_snap_mprim);
	m_planning_params.addParam("use_rpy_snap_mprim", m_planning_config.use_rpy_snap_mprim);
	m_planning_params.addParam("use_xyzrpy_snap_mprim", m_planning_config.use_xyzrpy_snap_mprim);
	m_planning_params.addParam("use_short_dist_mprims", m_planning_config.use_short_dist_mprims);
	m_planning_params.addParam("xyz_snap_dist_thresh", m_planning_config.xyz_snap_dist_thresh);
	m_planning_params.addParam("rpy_snap_dist_thresh", m_planning_config.rpy_snap_dist_thresh);
	m_planning_params.addParam("xyzrpy_snap_dist_thresh", m_planning_config.xyzrpy_snap_dist_thresh);
	m_planning_params.addParam("short_dist_mprims_thresh", m_planning_config.short_dist_mprims_thresh);
	// m_planning_params.addParam("epsilon", 200.0); Epsilon used for ARAStar
	m_planning_params.addParam("return_first_solution", false);
	m_planning_params.addParam("epsilon", 100.0);
	m_planning_params.addParam("search_mode", false);
	m_planning_params.addParam("allow_partial_solutions", false);
	m_planning_params.addParam("target_epsilon", 1.0);
	m_planning_params.addParam("delta_epsilon", 1.0);
	m_planning_params.addParam("improve_solution", false);
	m_planning_params.addParam("bound_expansions", true);
	m_planning_params.addParam("repair_time", 1.0);
	m_planning_params.addParam("bfs_inflation_radius", 0.02);
	m_planning_params.addParam("bfs_cost_per_cell", 100);
	m_ph.getParam("robot/interpolate", m_planning_params.interpolate_path);

	if (!createPlanner(m_planning_params.interpolate_path))
	{
		ROS_ERROR("Failed to create planner.");
		return false;
	}

	m_planner_init = true;
}

bool Robot::readPlannerConfig(const ros::NodeHandle &nh)
{
	if (!nh.getParam("discretization", m_planning_config.discretization)) {
		ROS_ERROR("Failed to read 'discretization' from the param server");
		return false;
	}

	if (!nh.getParam("mprim_filename", m_planning_config.mprim_filename)) {
		ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
		return false;
	}

	if (!nh.getParam("use_xyz_snap_mprim", m_planning_config.use_xyz_snap_mprim)) {
		ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
		return false;
	}

	if (!nh.getParam("use_rpy_snap_mprim", m_planning_config.use_rpy_snap_mprim)) {
		ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
		return false;
	}

	if (!nh.getParam("use_xyzrpy_snap_mprim", m_planning_config.use_xyzrpy_snap_mprim)) {
		ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
		return false;
	}

	if (!nh.getParam("use_short_dist_mprims", m_planning_config.use_short_dist_mprims)) {
		ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
		return false;
	}

	if (!nh.getParam("xyz_snap_dist_thresh", m_planning_config.xyz_snap_dist_thresh)) {
		ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
		return false;
	}

	if (!nh.getParam("rpy_snap_dist_thresh", m_planning_config.rpy_snap_dist_thresh)) {
		ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
		return false;
	}

	if (!nh.getParam("xyzrpy_snap_dist_thresh", m_planning_config.xyzrpy_snap_dist_thresh)) {
		ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
		return false;
	}

	if (!nh.getParam("short_dist_mprims_thresh", m_planning_config.short_dist_mprims_thresh)) {
		ROS_ERROR("Failed to read param 'short_dist_mprims_thresh' from the param server");
		return false;
	}

	return true;
}

bool Robot::createPlanner(bool interp)
{
	m_planner = std::make_unique<smpl::PlannerInterface>(
										m_rm.get(), m_cc_i.get(), m_grid_i.get(), m_cc_m.get());
	m_planning_params.interpolate_path = interp;

	if (!m_planner->init(m_planning_params)) {
		ROS_ERROR("Failed to initialize Planner Interface");
		return false;
	}

	return true;
}

void Robot::fillGoalConstraint()
{
	m_goal.position_constraints.resize(1);
	m_goal.orientation_constraints.resize(1);
	m_goal.position_constraints[0].header.frame_id = m_planning_frame;

	m_goal.position_constraints[0].constraint_region.primitives.resize(1);
	m_goal.position_constraints[0].constraint_region.primitive_poses.resize(1);
	m_goal.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	m_goal.position_constraints[0].constraint_region.primitive_poses[0].position.x = m_goal_vec[0];
	m_goal.position_constraints[0].constraint_region.primitive_poses[0].position.y = m_goal_vec[1];
	m_goal.position_constraints[0].constraint_region.primitive_poses[0].position.z = m_goal_vec[2];

	Eigen::Quaterniond q;
	smpl::angles::from_euler_zyx(m_goal_vec[5], m_goal_vec[4], m_goal_vec[3], q);
	tf::quaternionEigenToMsg(q, m_goal.orientation_constraints[0].orientation);

	// set tolerances
	m_goal.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
	m_goal.position_constraints[0].constraint_region.primitives[0].dimensions[2] = 0.025;
	m_goal.orientation_constraints[0].absolute_x_axis_tolerance = M_PI;
	m_goal.orientation_constraints[0].absolute_y_axis_tolerance = M_PI;
	m_goal.orientation_constraints[0].absolute_z_axis_tolerance = M_PI;
}

void Robot::createMultiPoseGoalConstraint(moveit_msgs::MotionPlanRequest& req)
{
	req.goal_constraints.clear();
	req.goal_constraints.resize(3);
	req.goal_constraints[0] = m_goal;

	Eigen::Quaterniond q;
	req.goal_constraints[1] = m_goal;
	smpl::angles::from_euler_zyx(m_goal_vec[5] + M_PI_2, m_goal_vec[4], m_goal_vec[3], q);
	tf::quaternionEigenToMsg(q, req.goal_constraints[1].orientation_constraints[0].orientation);

	req.goal_constraints[2] = m_goal;
	smpl::angles::from_euler_zyx(m_goal_vec[5] - M_PI_2, m_goal_vec[4], m_goal_vec[3], q);
	tf::quaternionEigenToMsg(q, req.goal_constraints[2].orientation_constraints[0].orientation);
}

void Robot::createPoseGoalConstraint(
	const Eigen::Affine3d& pose, moveit_msgs::MotionPlanRequest& req)
{
	m_goal_vec.clear();
	m_goal_vec.resize(6, 0.0);

	m_goal_vec[0] = pose.translation().x(); // x
	m_goal_vec[1] = pose.translation().y(); // y
	m_goal_vec[2] = pose.translation().z(); // z
	smpl::angles::get_euler_zyx(pose.rotation(), m_goal_vec[5], m_goal_vec[4], m_goal_vec[3]);

	fillGoalConstraint();
	// set tight tolerances
	m_goal.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.05);
	m_goal.orientation_constraints[0].absolute_x_axis_tolerance = M_PI;
	m_goal.orientation_constraints[0].absolute_y_axis_tolerance = M_PI;
	m_goal.orientation_constraints[0].absolute_z_axis_tolerance = M_PI;

	req.goal_constraints.clear();
	req.goal_constraints.resize(1);
	req.goal_constraints[0] = m_goal;
}

void Robot::createJointSpaceGoal(
	const smpl::RobotState& pose, moveit_msgs::MotionPlanRequest& req)
{
	req.goal_constraints.clear();
	req.goal_constraints.resize(1);

	req.goal_constraints[0].joint_constraints.resize(m_rm->jointVariableCount());
	for (int jidx = 0; jidx < m_rm->jointVariableCount(); ++jidx) {
		req.goal_constraints[0].joint_constraints[jidx].joint_name = m_robot_config.planning_joints.at(jidx);
		req.goal_constraints[0].joint_constraints[jidx].position = pose.at(jidx);
		req.goal_constraints[0].joint_constraints[jidx].tolerance_above = 0.0174533; // 1 degree
		req.goal_constraints[0].joint_constraints[jidx].tolerance_below = 0.0174533; // 1 degree
		req.goal_constraints[0].joint_constraints[jidx].weight = 1.0;
	}
}

void Robot::addPathConstraint(moveit_msgs::Constraints& path_constraints)
{
	path_constraints.position_constraints.resize(1);
	path_constraints.orientation_constraints.resize(1);
	path_constraints.position_constraints[0].header.frame_id = m_planning_frame;

	path_constraints.position_constraints[0].constraint_region.primitives.resize(1);
	path_constraints.position_constraints[0].constraint_region.primitive_poses.resize(1);
	path_constraints.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.0;
	path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.0;
	path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.z = 0.0;

	Eigen::Quaterniond q;
	smpl::angles::from_euler_zyx(0.0, 0.0, 0.0, q);
	tf::quaternionEigenToMsg(q, path_constraints.orientation_constraints[0].orientation);

	// set tolerances
	path_constraints.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.0);
	path_constraints.orientation_constraints[0].absolute_x_axis_tolerance = DEG5;
	path_constraints.orientation_constraints[0].absolute_y_axis_tolerance = DEG5;
	path_constraints.orientation_constraints[0].absolute_z_axis_tolerance = M_PI;
}

bool Robot::getStateNearPose(
	const Eigen::Affine3d& pose,
	const smpl::RobotState& seed_state,
	smpl::RobotState& state,
	int N,
	const std::string& ns)
{
	state.clear();
	smpl::RobotState seed;

	if (!seed_state.empty()) {
		seed = seed_state;
	}
	else {
		GetRandomState(seed);
	}

	int tries = 0;
	do
	{
		// if (!ns.empty())
		// {
		// 	auto markers = m_cc_i->getCollisionModelVisualization(seed);
		// 	for (auto& marker : markers) {
		// 		marker.ns = ns + "_seed";
		// 	}
		// 	SV_SHOW_INFO_NAMED(ns + "_seed", markers);

		// 	SV_SHOW_INFO_NAMED(ns + "_pose", smpl::visual::MakePoseMarkers(
		// 		pose, m_grid_i->getReferenceFrame(), ns + "_pose"));
		// }

		if (m_rm->computeIKSearch(pose, seed, state))
		{
			if (m_rm->checkJointLimits(state) && m_cc_i->isStateValid(state)) {
				break;
			}
		}
		GetRandomState(seed);
		state.clear();
		++tries;
	}
	while (tries < N);

	return tries < N;
}


void Robot::displayObjectMarker(const Object& object)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_footprint";
	marker.header.stamp = ros::Time();
	marker.ns = "object";
	marker.id = object.desc.id;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = object.Shape() == 0 ? visualization_msgs::Marker::CUBE : visualization_msgs::Marker::CYLINDER;

	geometry_msgs::Pose pose;
	pose.position.x = object.desc.o_x;
	pose.position.y = object.desc.o_y;
	pose.position.z = object.desc.o_z;

	Eigen::Quaterniond q;
	smpl::angles::from_euler_zyx(
			object.desc.o_yaw, object.desc.o_pitch, object.desc.o_roll, q);

	geometry_msgs::Quaternion orientation;
	tf::quaternionEigenToMsg(q, orientation);
	pose.orientation = orientation;

	marker.pose = pose;

	marker.scale.x = 2 * object.desc.x_size;
	marker.scale.y = 2 * object.desc.y_size;
	marker.scale.z = 2 * object.desc.z_size;
	marker.scale.z /= object.Shape() == 2 ? 2.0 : 1.0;

	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.86;
	marker.color.g = 0.34;
	marker.color.b = 0.16;

	m_vis_pub.publish(marker);
}

} // namespace clutter
