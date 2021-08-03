#include <pushplan/robot.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/constants.hpp>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <eigen_conversions/eigen_msg.h>

namespace clutter
{

bool Robot::Init()
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
	{
		ROS_ERROR("Failed to set up Robot Model");
		return false;
	}

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

	m_mass = R_MASS;
	m_b = SEMI_MINOR;
	m_shoulder = "r_shoulder_pan_link";
	m_elbow = "r_elbow_flex_link";
	m_wrist = "r_wrist_roll_link";
	m_tip = "r_gripper_motor_screw_link"; // TODO: check
	initObjects();

	m_t = 0;

	m_init.t = m_t;
	m_init.state.clear();
	m_init.state.insert(m_init.state.begin(),
		m_start_state.joint_state.position.begin() + 1, m_start_state.joint_state.position.end());
	Eigen::Affine3d ee_pose = m_rm->computeFK(m_init.state);
	State ee = {ee_pose.translation().x(), ee_pose.translation().y()};
	ContToDisc(ee, m_init.coord);
	m_z = ee_pose.translation().z();

	reinitObjects(m_init.state);
	m_current = m_init;

	m_wastar = std::make_unique<WAStar>(this, 1.0); // make A* search object

	return true;
}

bool Robot::AtGoal(const LatticeState& s, bool verbose)
{
	reinitObjects(s.state);
	if (m_phase == 0)
	{
		return m_cc->OOICollision(m_objs.back());
	}

	State ee = {m_objs.back().o_x, m_objs.back().o_y};
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

	// r_shoulder_pan_joint
	generateSuccessor(parent, 0, 1, succ_ids, costs);
	generateSuccessor(parent, 0, -1, succ_ids, costs);

	// r_elbow_flex_joint
	generateSuccessor(parent, 3, 1, succ_ids, costs);
	generateSuccessor(parent, 3, -1, succ_ids, costs);

	// r_wrist_flex_joint
	generateSuccessor(parent, 5, 1, succ_ids, costs);
	generateSuccessor(parent, 5, -1, succ_ids, costs);
}

unsigned int Robot::GetGoalHeuristic(int state_id)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	if (s->state.empty()) {
		DiscToCont(s->coord, s->state);
	}

	State ee;
	if (s->state.size() == 2) {
		ee = s->state;
	}
	else {
		reinitObjects(s->state);
		ee = {m_objs.back().o_x, m_objs.back().o_y};
	}

	double dist = EuclideanDist(ee, m_goalf);
	return (dist * COST_MULT);
}

unsigned int Robot::GetGoalHeuristic(const LatticeState& s)
{
	// TODO: RRA* informed backwards Dijkstra's heuristic
	reinitObjects(s.state);
	State ee = {m_objs.back().o_x, m_objs.back().o_y};
	double dist = EuclideanDist(ee, m_goalf);
	return (dist * COST_MULT);
}

const std::vector<Object>* Robot::GetObject(const LatticeState& s)
{
	reinitObjects(s.state);
	return &m_objs;
}

bool Robot::setIKState(
	const Coord& loc, const State& seed, State& state)
{
	State loc_f;
	DiscToCont(loc, loc_f);

	Eigen::Affine3d pose;
	int tries = 0;
	while (tries <= 1) // TODO: make configurable parameter
	{
		pose = Eigen::Translation3d(
				loc_f.at(0), loc_f.at(1), m_z) *
				Eigen::AngleAxisd((m_distD(m_rng) * M_PI_2) - M_PI_4, Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd((m_distD(m_rng) * M_PI_2) - M_PI_4, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd((m_distD(m_rng) * M_PI_2) - M_PI_4, Eigen::Vector3d::UnitX());
		if (m_rm->computeIKSearch(pose, seed, state))
		{
			break;
			// if (m_rm->checkJointLimits(state)) {
			// }
		}
		++tries;
	}

	return tries < 1;
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
	if (!m_rm->checkJointLimits(child.state)) {
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
		ROS_INFO("Set joint %s to %f", m_start_state.joint_state.name[i].c_str(), m_start_state.joint_state.position[i]);
		SetVariablePosition(&reference_state, var, m_start_state.joint_state.position[i]);
	}
	SetReferenceState(m_rm.get(), GetVariablePositions(&reference_state));

	// Set reference state in the collision scene here...
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
	const smpl::urdf::Link* link_s = smpl::urdf::GetLink(&m_rm->m_robot_model, m_shoulder.c_str());
	const smpl::urdf::Link* link_e = smpl::urdf::GetLink(&m_rm->m_robot_model, m_elbow.c_str());
	auto link_pose_s = m_rm->computeFKLink(s, link_s);
	auto link_pose_e = m_rm->computeFKLink(s, link_e);
	State F1 = {link_pose_s.translation().x(), link_pose_s.translation().y()};
	State F2 = {link_pose_e.translation().x(), link_pose_e.translation().y()};
	ArmRectObj(F1, F2, m_b, m_objs.at(0));

	// elbow-wrist rectangle
	const smpl::urdf::Link* link_w = smpl::urdf::GetLink(&m_rm->m_robot_model, m_wrist.c_str());
	auto link_pose_w = m_rm->computeFKLink(s, link_w);
	F1 = F2;
	F2.at(0) = link_pose_w.translation().x();
	F2.at(1) = link_pose_w.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(1));

	// wrist-tip rectangle
	const smpl::urdf::Link* link_t = smpl::urdf::GetLink(&m_rm->m_robot_model, m_tip.c_str());
	auto link_pose_t = m_rm->computeFKLink(s, link_t);
	F1 = F2;
	F2.at(0) = link_pose_t.translation().x();
	F2.at(1) = link_pose_t.translation().y();
	ArmRectObj(F1, F2, m_b, m_objs.at(2));
}

} // namespace clutter
