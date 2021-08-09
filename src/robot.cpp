#include <pushplan/robot.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/constants.hpp>
#include <pushplan/pr2_allowed_collision_pairs.h>

#include <smpl/stl/memory.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl_collision_checking/types.h>
#include <smpl/angles.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/console/console.h>
#include <smpl/ros/factories.h>
#include <smpl_urdf_robot_model/urdf_robot_model.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <eigen_conversions/eigen_msg.h>

namespace clutter
{

bool Robot::Setup()
{
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
	{		ROS_ERROR("Failed to set up Robot Model");
		return false;
	}

	//////////////////////////////////
	// Initialize Collision Checker //
	//////////////////////////////////

	initOccupancyGrid();

	if (!initCollisionChecker())
	{
		ROS_ERROR("Failed to initialise collision checker!");
		return false;
	}
	m_cc_i->setWorldToModelTransform(Eigen::Affine3d::Identity());

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
	m_mass = R_MASS;
	m_b = SEMI_MINOR;
	m_shoulder = "r_shoulder_pan_link";
	m_elbow = "r_elbow_flex_link";
	m_wrist = "r_wrist_roll_link";
	m_tip = "r_gripper_motor_screw_link"; // TODO: check
	m_table_z = m_cc->GetTableHeight();
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);

	m_link_s = smpl::urdf::GetLink(&m_rm->m_robot_model, m_shoulder.c_str());
	m_link_e = smpl::urdf::GetLink(&m_rm->m_robot_model, m_elbow.c_str());
	m_link_w = smpl::urdf::GetLink(&m_rm->m_robot_model, m_wrist.c_str());
	m_link_t = smpl::urdf::GetLink(&m_rm->m_robot_model, m_tip.c_str());

	initObjects();

	smpl::RobotState dummy;
	dummy.insert(dummy.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	m_rm->computeFK(dummy); // for reasons unknown
	if (!reinitStartState())
	{
		ROS_ERROR("Failed to reinit start state!");
		return false;
	}

	if (!m_wastar) {
		m_wastar = std::make_unique<WAStar>(this, 1.0); // make A* search object
	}

	return true;
}

bool Robot::AddObstacles(const std::vector<Object>& obstacles)
{
	for (const auto& obs: obstacles)
	{
		moveit_msgs::CollisionObject obj_msg;
		if (!getCollisionObjectMsg(obs, obj_msg)) {
			return false;
		}
		if (!processCollisionObjectMsg(obj_msg)) {
			return false;
		}
	}

	SV_SHOW_INFO(m_cc_i->getCollisionWorldVisualization());
	SV_SHOW_INFO(m_cc_i->getOccupiedVoxelsVisualization());
	return true;
}

bool Robot::Init()
{
	m_solve.clear();
	m_retrieve.clear();
	m_move.clear();

	m_t = 0;
	m_retrieved = 0;

	m_init.t = m_t;
	m_init.state.clear();
	m_init.state.insert(m_init.state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());

	m_init.coord = Coord(m_rm->jointVariableCount());
	stateToCoord(m_init.state, m_init.coord);
	Eigen::Affine3d ee_pose = m_rm->computeFK(m_init.state);

	reinitObjects(m_init.state);
	m_current = m_init;

	return true;
}

bool Robot::RandomiseStart()
{
	if (!reinitStartState())
	{
		ROS_ERROR("Failed to randomise start!");
		return false;
	}

	m_init.state.clear();
	m_init.state.insert(m_init.state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	m_init.coord = Coord(m_rm->jointVariableCount());
	stateToCoord(m_init.state, m_init.coord);

	reinitObjects(m_init.state);
	m_current = m_init;

	return true;
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

void Robot::ConvertTraj(
	const Trajectory& traj_in,
	moveit_msgs::RobotTrajectory& traj_out)
{
	traj_out.joint_trajectory.header.frame_id = m_planning_frame;
	traj_out.multi_dof_joint_trajectory.header.frame_id = m_planning_frame;

	traj_out.joint_trajectory.joint_names.clear();
	traj_out.joint_trajectory.points.clear();
	traj_out.multi_dof_joint_trajectory.joint_names.clear();
	traj_out.multi_dof_joint_trajectory.points.clear();

	// fill joint names header for both single- and multi-dof joint trajectories
	auto& variable_names = m_rm->getPlanningJoints();
	for (auto& var_name : variable_names) {
		std::string joint_name;
		if (smpl::IsMultiDOFJointVariable(var_name, &joint_name)) {
			auto it = std::find(
					begin(traj_out.multi_dof_joint_trajectory.joint_names),
					end(traj_out.multi_dof_joint_trajectory.joint_names),
					joint_name);
			if (it == end(traj_out.multi_dof_joint_trajectory.joint_names)) {
				// avoid duplicates
				traj_out.multi_dof_joint_trajectory.joint_names.push_back(joint_name);
			}
		} else {
			traj_out.joint_trajectory.joint_names.push_back(var_name);
		}
	}

	// empty or number of points in the path
	if (!traj_out.joint_trajectory.joint_names.empty()) {
		traj_out.joint_trajectory.points.resize(traj_in.size());
	}
	// empty or number of points in the path
	if (!traj_out.multi_dof_joint_trajectory.joint_names.empty()) {
		traj_out.multi_dof_joint_trajectory.points.resize(traj_in.size());
	}

	for (size_t pidx = 0; pidx < traj_in.size(); ++pidx) {
		auto& point = traj_in[pidx].state;

		for (size_t vidx = 0; vidx < variable_names.size(); ++vidx) {
			auto& var_name = variable_names[vidx];

			std::string joint_name, local_name;
			if (smpl::IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
				auto& p = traj_out.multi_dof_joint_trajectory.points[pidx];
				p.transforms.resize(traj_out.multi_dof_joint_trajectory.joint_names.size());

				auto it = std::find(
						begin(traj_out.multi_dof_joint_trajectory.joint_names),
						end(traj_out.multi_dof_joint_trajectory.joint_names),
						joint_name);
				if (it == end(traj_out.multi_dof_joint_trajectory.joint_names)) continue;

				auto tidx = std::distance(begin(traj_out.multi_dof_joint_trajectory.joint_names), it);

				if (local_name == "x" ||
					local_name == "trans_x")
				{
					p.transforms[tidx].translation.x = point[vidx];
				} else if (local_name == "y" ||
					local_name == "trans_y")
				{
					p.transforms[tidx].translation.y = point[vidx];
				} else if (local_name == "trans_z") {
					p.transforms[tidx].translation.z = point[vidx];
				} else if (local_name == "theta") {
					Eigen::Quaterniond q(Eigen::AngleAxisd(point[vidx], Eigen::Vector3d::UnitZ()));
					tf::quaternionEigenToMsg(q, p.transforms[tidx].rotation);
				} else if (local_name == "rot_w") {
					p.transforms[tidx].rotation.w = point[vidx];
				} else if (local_name == "rot_x") {
					p.transforms[tidx].rotation.x = point[vidx];
				} else if (local_name == "rot_y") {
					p.transforms[tidx].rotation.y = point[vidx];
				} else if (local_name == "rot_z") {
					p.transforms[tidx].rotation.z = point[vidx];
				} else {
					SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
					continue;
				}
			} else {
				auto& p = traj_out.joint_trajectory.points[pidx];
				p.positions.resize(traj_out.joint_trajectory.joint_names.size());

				auto it = std::find(
						begin(traj_out.joint_trajectory.joint_names),
						end(traj_out.joint_trajectory.joint_names),
						var_name);
				if (it == end(traj_out.joint_trajectory.joint_names)) continue;

				auto posidx = std::distance(begin(traj_out.joint_trajectory.joint_names), it);

				p.positions[posidx] = point[vidx];
				p.time_from_start = ros::Duration(point.back());
			}
		}
	}
	traj_out.joint_trajectory.header.stamp = ros::Time::now();
}

bool Robot::AtGoal(const LatticeState& s, bool verbose)
{
	if (m_phase == 0)
	{
		reinitObjects(s.state);
		return m_cc->OOICollision(m_objs.back());
	}

	Eigen::Affine3d ee_pose = m_rm->computeFK(s.state);
	State ee = {ee_pose.translation().x(), ee_pose.translation().y()};
	double dist = EuclideanDist(ee, m_goalf);

	if (verbose) {
		SMPL_INFO("Robot EE at: (%f, %f), Goal: (%f, %f), dist = %f (thresh = %f)", ee.at(0), ee.at(1), m_goalf.at(0), m_goalf.at(1), dist, GOAL_THRESH);
	}

	return dist < GOAL_THRESH;
}

void Robot::Step(int k)
{
	// TODO: account for k > 1
	m_t += k;
	for (const auto& s: m_solve)
	{
		if (s.t == m_t)
		{
			m_current = s;
			reinitObjects(m_current.state);

			m_move.push_back(m_current);
		}
	}

	if (m_phase == 1 && m_priority == 1 && !m_retrieve.empty())	{
		m_retrieve.erase(m_retrieve.begin());
		if (m_retrieve.size() == 1) {
			m_retrieved = 1;
		}
	}
}

void Robot::GetSuccs(
	int state_id,
	std::vector<int>* succ_ids,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succ_ids->clear();
	costs->clear();

	LatticeState* parent = getHashEntry(state_id);
	assert(parent);
	m_closed.push_back(parent);

	if (IsGoal(state_id)) {
		SMPL_WARN("We are expanding the goal state (???)");
		return;
	}

	for(int jidx = 0; jidx < m_rm->jointVariableCount(); jidx++)
	{
		generateSuccessor(parent, jidx, 1, succ_ids, costs);
		generateSuccessor(parent, jidx, -1, succ_ids, costs);
	}
	// // r_shoulder_pan_joint
	// generateSuccessor(parent, 0, 1, succ_ids, costs);
	// generateSuccessor(parent, 0, -1, succ_ids, costs);

	// // r_elbow_flex_joint
	// generateSuccessor(parent, 3, 1, succ_ids, costs);
	// generateSuccessor(parent, 3, -1, succ_ids, costs);

	// // r_wrist_flex_joint
	// generateSuccessor(parent, 5, 1, succ_ids, costs);
	// generateSuccessor(parent, 5, -1, succ_ids, costs);
}

unsigned int Robot::GetGoalHeuristic(int state_id)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	if (s->state.empty()) {
		if (s->coord.size() == 2) {
			DiscToCont(s->coord, s->state);
		}
		else {
			coordToState(s->coord, s->state);
		}
	}

	State ee;
	if (s->state.size() == 2) {
		// this should only be true for the goal state?
		ee = s->state;
	}
	else {
		Eigen::Affine3d ee_pose = m_rm->computeFK(s->state);
		ee = {ee_pose.translation().x(), ee_pose.translation().y()};
	}

	double dist = EuclideanDist(ee, m_goalf);
	return (dist * COST_MULT);
}

unsigned int Robot::GetGoalHeuristic(const LatticeState& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	Eigen::Affine3d ee_pose = m_rm->computeFK(s.state);
	State ee = {ee_pose.translation().x(), ee_pose.translation().y()};
	double dist = EuclideanDist(ee, m_goalf);
	return (dist * COST_MULT);
}

const std::vector<Object>* Robot::GetObject(const LatticeState& s)
{
	reinitObjects(s.state);
	return &m_objs;
}

Coord Robot::GetEECoord()
{
	Eigen::Affine3d ee_pose = m_rm->computeFK(m_current.state);
	State ee = {ee_pose.translation().x(), ee_pose.translation().y()};
	Coord ee_coord;
	ContToDisc(ee, ee_coord);
	return ee_coord;
}

void Robot::getRandomState(smpl::RobotState& s)
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
	double x, y, deg5 = 5.0 * (M_PI/180.0);
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
					Eigen::AngleAxisd(((2 * m_distD(m_rng)) - 1) * deg5, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(((2 * m_distD(m_rng)) - 1) * deg5, Eigen::Vector3d::UnitX());

		if (m_rm->computeIKSearch(ee_pose, seed, s))
		{
			if (m_rm->checkJointLimits(s)) {
				break;
			}
		}
		getRandomState(seed);
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

int Robot::generateSuccessor(
	const LatticeState* parent,
	int jidx, int delta,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	LatticeState child;
	child.t = parent->t + 1;
	child.coord = parent->coord;
	child.coord.at(jidx) += delta;
	child.state = State(m_rm->jointVariableCount());
	coordToState(child.coord, child.state);

	// reject invalid successors
	if (!m_rm->checkJointLimits(child.state)) {
		return -1;
	}
	if (!m_cc_i->isStateValid(child.state)) {
		std::string ns_vis = "collides";
		auto markers = m_cc_i->getCollisionModelVisualization(child.state);
		for (auto& marker : markers) {
			marker.ns = ns_vis;
		}
		SV_SHOW_INFO_NAMED(ns_vis.c_str(), markers);
		return -1;
	}
	Eigen::Affine3d ee_pose = m_rm->computeFK(child.state);
	if (std::fabs(ee_pose.translation().z() - (m_table_z + 0.05)) > 0.05) {
		return -1;
	}

	reinitObjects(child.state);
	if (m_cc->ImmovableCollision(m_objs, m_priority)) {
		return -1;
	}

	// robot always has 1 priority

	int succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

unsigned int Robot::cost(
	const LatticeState* s1,
	const LatticeState* s2)
{
	if (s2->t <= m_t + WINDOW)
	{
		if (AtGoal(*s1) && AtGoal(*s2)){
			return 0;
		}

		return COST_MULT;

		// State s1_loc, s2_loc;
		// DiscToCont(s1->coord, s1_loc);
		// DiscToCont(s2->coord, s2_loc);

		// double dist = 1.0f + EuclideanDist(s1_loc, s2_loc);
		// return (dist * COST_MULT);

		// // Works okay for WINDOW = 20, but not for WINDOW <= 10
		// double bdist = m_cc->BoundaryDistance(s2f);
		// double bw = m_cc->GetBaseWidth(), bl = m_cc->GetBaseLength();
		// return ((dist + WINDOW*(1 - bdist/std::min(bw, bl))*(m_priority > 0)) * COST_MULT);
	}
	else if (s2->t == m_t + WINDOW + 1) {
		return GetGoalHeuristic(*s2);
	}
	else {
		SMPL_ERROR("Unknown edge cost condition! Return 0. (s1->t, s2->t, m_t) = (%d, %d, %d)", s1->t, s2->t, m_t);
		return 0;
	}
}

bool Robot::convertPath(
	const std::vector<int>& idpath)
{
	Trajectory opath; // vector of LatticeState

	if (idpath.empty()) {
		return true;
	}

	LatticeState state;

	// attempt to handle paths of length 1...do any of the sbpl planners still
	// return a single-point path in some cases?
	if (idpath.size() == 1)
	{
		auto state_id = idpath[0];

		if (state_id == m_goal_id)
		{
			auto* entry = getHashEntry(m_start_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", m_start_id);
				return false;
			}
			state = *entry;
			opath.push_back(state);
		}
		else
		{
			auto* entry = getHashEntry(state_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", state_id);
				return false;
			}
			state = *entry;
			opath.push_back(state);
		}
	}

	if (idpath[0] == m_goal_id)
	{
		SMPL_ERROR("Cannot extract a non-trivial path starting from the goal state");
		return false;
	}

	// grab the first point
	{
		auto* entry = getHashEntry(idpath[0]);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry for state %d", idpath[0]);
			return false;
		}
		state = *entry;
		opath.push_back(state);
	}

	// grab the rest of the points
	for (size_t i = 1; i < idpath.size(); ++i)
	{
		auto prev_id = idpath[i - 1];
		auto curr_id = idpath[i];

		if (prev_id == m_goal_id)
		{
			SMPL_ERROR("Cannot determine goal state predecessor state during path extraction");
			return false;
		}

		auto* entry = getHashEntry(curr_id);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry state %d", curr_id);
			return false;
		}
		state = *entry;
		opath.push_back(state);
	}
	m_solve = std::move(opath);
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

	std::string planning_joint_list;
	if (!nh.getParam("planning_joints", planning_joint_list)) {
		ROS_ERROR("Failed to read 'planning_joints' from the param server");
		return false;
	}

	std::stringstream joint_name_stream(planning_joint_list);
	while (joint_name_stream.good() && !joint_name_stream.eof()) {
		std::string jname;
		joint_name_stream >> jname;
		if (jname.empty()) {
			continue;
		}
		m_robot_config.planning_joints.push_back(jname);
	}

	// only required for generic kdl robot model?
	nh.getParam("kinematics_frame", m_robot_config.kinematics_frame);
	nh.getParam("chain_tip_link", m_robot_config.chain_tip_link);
	return true;
}

bool Robot::setupRobotModel()
{
	if (m_robot_config.kinematics_frame.empty() || m_robot_config.chain_tip_link.empty()) {
		ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
		m_rm = NULL;
		return false;
	}

	m_rm = std::make_unique<smpl::KDLRobotModel>();

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
	o.shape = 0; // rectangle
	o.type = 1; // movable
	o.o_x = 0.0; // TBD
	o.o_y = 0.0; // TBD
	o.o_z = 0.0; // NA
	o.o_roll = 0.0; // NA
	o.o_pitch = 0.0; // NA
	o.o_yaw = 0.0; // TBD
	o.x_size = 0.0; // TBD
	o.y_size = 0.0; // TBD
	o.z_size = 0.0; // NA
	o.mass = m_mass;
	o.locked = o.mass == 0;
	o.mu = 0.8;
	o.movable = true;

	// shoulder-elbow rectangle
	o.id = 100;
	m_objs.push_back(o);

	// elbow-wrist rectangle
	o.id = 101;
	m_objs.push_back(o);

	// wrist-tip rectangle
	o.id = 102;
	m_objs.push_back(o);
}

void Robot::reinitObjects(const State& s)
{
	// shoulder-elbow rectangle
	auto link_pose_s = m_rm->computeFKLink(s, m_link_s);
	auto link_pose_e = m_rm->computeFKLink(s, m_link_e);
	State F1 = {link_pose_s.translation().x(), link_pose_s.translation().y()};
	State F2 = {link_pose_e.translation().x(), link_pose_e.translation().y()};
	ArmRectObj(F1, F2, m_b, m_objs.at(0));

	// elbow-wrist rectangle
	auto link_pose_w = m_rm->computeFKLink(s, m_link_w);
	F1 = F2;
	F2.at(0) = link_pose_w.translation().x();
	F2.at(1) = link_pose_w.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(1));

	// wrist-tip rectangle
	auto link_pose_t = m_rm->computeFKLink(s, m_link_t);
	F1 = F2;
	F2.at(0) = link_pose_t.translation().x();
	F2.at(1) = link_pose_t.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(2));
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

void Robot::initOccupancyGrid()
{
	double df_size_x, df_size_y, df_size_z;
	double df_origin_x, df_origin_y, df_origin_z;
	double max_distance;

	m_ph.getParam("occupancy_grid/size_x", df_size_x);
	m_ph.getParam("occupancy_grid/size_y", df_size_y);
	m_ph.getParam("occupancy_grid/size_z", df_size_z);
	m_ph.getParam("occupancy_grid/origin_x", df_origin_x);
	m_ph.getParam("occupancy_grid/origin_y", df_origin_y);
	m_ph.getParam("occupancy_grid/origin_z", df_origin_z);
	m_ph.getParam("occupancy_grid/max_dist", max_distance);
	m_ph.getParam("occupancy_grid/res", m_df_res);
	m_ph.getParam("planning_frame", m_planning_frame);

	using DistanceMapType = smpl::EuclidDistanceMap;

	m_df_i = std::make_shared<DistanceMapType>(
			df_origin_x, df_origin_y, df_origin_z,
			df_size_x, df_size_y, df_size_z,
			m_df_res,
			max_distance);

	bool ref_counted = false;
	m_grid_i = std::make_unique<smpl::OccupancyGrid>(m_df_i, ref_counted);
	m_grid_i->setReferenceFrame(m_planning_frame);
	SV_SHOW_INFO(m_grid_i->getBoundingBoxVisualization());
}

bool Robot::initCollisionChecker()
{
	smpl::collision::CollisionModelConfig cc_conf;
	if (!smpl::collision::CollisionModelConfig::Load(m_ph, cc_conf)) {
		ROS_ERROR("Failed to load Collision Model Config");
		return false;
	}

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

	return true;
}

bool Robot::getCollisionObjectMsg(
			const Object& object,
			moveit_msgs::CollisionObject& obj_msg,
			bool remove)
{
	obj_msg.id = std::to_string(object.id);
	obj_msg.operation = remove ? moveit_msgs::CollisionObject::REMOVE :
										moveit_msgs::CollisionObject::ADD;

	if (remove) {
		return true;
	}

	obj_msg.header.frame_id = m_planning_frame;
	obj_msg.header.stamp = ros::Time::now();

	shape_msgs::SolidPrimitive object_prim;
	switch (object.shape)
	{
		case 1: {
			object_prim.type = shape_msgs::SolidPrimitive::SPHERE;
			object_prim.dimensions.resize(1);
			object_prim.dimensions[0] = object.x_size;
			break;
		}

		case 2: {
			object_prim.type = shape_msgs::SolidPrimitive::CYLINDER;
			object_prim.dimensions.resize(2);
			object_prim.dimensions[0] = object.z_size;
			object_prim.dimensions[1] = object.x_size;
			break;
		}

		case 0:
		default: {
			object_prim.type = shape_msgs::SolidPrimitive::BOX;
			object_prim.dimensions.resize(3);
			object_prim.dimensions[0] = object.x_size * 2.0;
			object_prim.dimensions[1] = object.y_size * 2.0;
			object_prim.dimensions[2] = object.z_size * 2.0;
			break;
		}
	}

	geometry_msgs::Pose pose;
	pose.position.x = object.o_x;
	pose.position.y = object.o_y;
	pose.position.z = object.o_z;

	Eigen::Quaterniond q;
	smpl::angles::from_euler_zyx(
			object.o_yaw, object.o_pitch, object.o_roll, q);

	geometry_msgs::Quaternion orientation;
	tf::quaternionEigenToMsg(q, orientation);

	pose.orientation = orientation;

	obj_msg.primitives.push_back(object_prim);
	obj_msg.primitive_poses.push_back(pose);

	return true;
}

/// \brief Process a collision object
/// \param object The collision object to be processed
/// \return true if the object was processed successfully; false otherwise
bool Robot::processCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object, bool movable)
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

bool Robot::addCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object, bool movable)
{
	if (m_cc_i->worldCollisionModel()->hasObjectWithName(object.id)) {
		return false;
	}

	if (object.header.frame_id != m_cc_i->getReferenceFrame()) {
		ROS_ERROR("Collision object must be specified in the Collision Space's reference frame (%s)", m_cc_i->getReferenceFrame().c_str());
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

		m_collision_shapes.push_back(std::move(shape));
		shapes.push_back(m_collision_shapes.back().get());

		auto& prim_pose = object.primitive_poses[i];
		Eigen::Affine3d transform;
		tf::poseMsgToEigen(prim_pose, transform);
		shape_poses.push_back(transform);
	}

	for (size_t i = 0; i < object.planes.size(); ++i) {
		auto& plane = object.planes[i];

		auto shape = smpl::make_unique<smpl::collision::PlaneShape>(
				plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]);
		m_collision_shapes.push_back(std::move(shape));
		shapes.push_back(m_collision_shapes.back().get());

		auto& plane_pose = object.plane_poses[i];
		Eigen::Affine3d transform;
		tf::poseMsgToEigen(plane_pose, transform);
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

	m_collision_objects.push_back(std::move(co));
	return m_cc_i->insertObject(m_collision_objects.back().get());
}

bool Robot::removeCollisionObjectMsg(
	const moveit_msgs::CollisionObject& object, bool movable)
{
	// find the collision object with this name
	auto* _object = findCollisionObject(object.id);
	if (!_object) {
		return false;
	}

	// remove from collision space
	if (!m_cc_i->removeObject(_object)) {
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
			begin(m_collision_shapes), end(m_collision_shapes),
			belongs_to_object);
	m_collision_shapes.erase(rit, end(m_collision_shapes));

	// remove the object itself
	auto is_object = [_object](const std::unique_ptr<smpl::collision::CollisionObject>& object) {
		return object.get() == _object;
	};
	auto rrit = std::remove_if(
			begin(m_collision_objects), end(m_collision_objects), is_object);
	m_collision_objects.erase(rrit, end(m_collision_objects));

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

auto Robot::findCollisionObject(const std::string& id) const
	-> smpl::collision::CollisionObject*
{
	for (auto& object : m_collision_objects) {
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

	if (m_move.empty()) {
		return ma;
	}

	auto cinc = 1.0f / float(m_move.size());
	for (size_t i = 0; i < m_move.size(); ++i) {
		auto markers = m_cc_i->getCollisionModelVisualization(m_move[i].state);

		for (auto& marker : markers) {
			auto r = 0.1f;
			auto g = cinc * (float)(m_move.size() - (i + 1));
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

} // namespace clutter
