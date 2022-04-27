#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <pushplan/agents/object.hpp>
#include <pushplan/utils/types.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/bullet_sim.hpp>
#include <comms/ObjectsPoses.h>

#include <smpl/console/console.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/planning_params.h>
#include <smpl/debug/marker.h>
#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/occupancy_grid.h>
#include <smpl/debug/visualizer_ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/pushing_kdl_robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include <string>
#include <memory>
#include <random>

namespace clutter
{

class HighLevelNode;
class Agent;

class Robot
{
public:
	Robot() : m_ph("~"), m_rng(m_dev()) {};
	void SetID(int id) { m_id = id; };
	int GetID() { return m_id; };

	bool Setup();
	bool SavePushData(int scene_id);

	bool CheckCollisionWithObject(const LatticeState& robot, Agent* a, int t);
	bool CheckCollision(const LatticeState& robot, int t);

	bool ProcessObstacles(const std::vector<Object>& obstacles, bool remove=false);
	bool ProcessObstacles(const std::vector<Object*>& obstacles, bool remove=false);

	bool Init();
	void SetMovables(const std::vector<std::shared_ptr<Agent> >& agents);
	bool RandomiseStart();
	bool PlanApproachOnly(const std::vector<Object*>& movable_obstacles);
	bool PlanRetrieval(const std::vector<Object*>& movable_obstacles, bool finalise=false);
	void UpdateNGR(bool vis=false);
	bool SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f);

	void ProfileTraj(Trajectory& traj);
	bool ComputeGrasps(
		const std::vector<double>& pregrasp_goal);
	void ConvertTraj(
		const Trajectory& traj_in,
		moveit_msgs::RobotTrajectory& traj_out);

	bool AtGrasp() {
		return m_t == m_grasp_at - 1;
	}
	bool AtEnd() {
		return m_t == m_solve.back().t;
	}
	int GraspAt() { return m_grasp_at; };

	bool UpdateKDLRobot(int mode);
	bool InitArmPlanner(bool interp=false);
	bool PlanPush(
		Agent* object, const std::vector<double>& push,
		const std::vector<Object*>& other_movables,
		const comms::ObjectsPoses& rearranged, comms::ObjectsPoses& result);
	trajectory_msgs::JointTrajectory GetLastPlan() {
		return m_traj;
	};
	void SetSim(const std::shared_ptr<BulletSim>& sim) {
		m_sim = sim;
	}
	void SetOOI(const Object& ooi) {
		m_ooi = ooi;
	}
	void SetOOI(Object* ooi) {
		SetOOI(*ooi);
	}

	void AnimateSolution();

	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	const std::vector<Object>* GetObject() const { return &m_objs; };
	const std::vector<Object>* GetObject(const LatticeState& s);
	const Trajectory* SolveTraj() const { return &m_solve; };

	State GetEEState(const State& state);
	const moveit_msgs::RobotState* GetStartState() { return &m_start_state; };

	void PrintFK(const trajectory_msgs::JointTrajectory& traj)
	{
		for (const auto& p: traj.points)
		{
			auto pose = m_rm->computeFK(p.positions);
			SMPL_INFO("ee (x, y, z) = (%f, %f, %f)", pose.translation().x(), pose.translation().y(), pose.translation().z());
		}
	}

	double TrajPlanTime() {
		return m_stats["approach_plan_time"] + m_stats["extract_plan_time"];
	}
	double PlannerTime() {
		return m_planner_time;
	}
	double SimTime() {
		return m_sim_time;
	}
	bool BadAttach() {
		return (m_stats["attach_fails"] > 0) || (m_stats["attach_collides"] > 0);
	}

	auto ObsGrid() const -> const std::shared_ptr<smpl::OccupancyGrid>& {
		return m_grid_i;
	}
	auto NGRGrid() const -> const std::shared_ptr<smpl::OccupancyGrid>& {
		return m_grid_ngr;
	}

	void VizCC() {
		SV_SHOW_INFO(m_cc_i->getCollisionWorldVisualization());
	}

	auto TrajVoxels() const -> const std::vector<std::vector<Eigen::Vector3d>>* {
		return &m_traj_voxels;
	}

private:
	int m_id;
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::PushingKDLRobotModel> m_rm;
	moveit_msgs::RobotState m_start_state;
	ros::Publisher m_vis_pub;

	// cached from robot model
	std::vector<double> m_min_limits;
	std::vector<double> m_max_limits;
	std::vector<bool> m_continuous;
	std::vector<bool> m_bounded;

	std::vector<int> m_coord_vals;
	std::vector<double> m_coord_deltas;

	int m_grasp_at;
	double m_b, m_table_z;
	std::string m_shoulder, m_elbow, m_wrist, m_tip;
	const smpl::urdf::Link* m_link_s = nullptr;
	const smpl::urdf::Link* m_link_e = nullptr;	const smpl::urdf::Link* m_link_w = nullptr;
	const smpl::urdf::Link* m_link_t = nullptr;
	smpl::RobotState m_pregrasp_state, m_grasp_state, m_postgrasp_state;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::normal_distribution<> m_distG;

	std::shared_ptr<smpl::DistanceMapInterface> m_df_i, m_df_ngr;
	std::shared_ptr<smpl::OccupancyGrid> m_grid_i, m_grid_ngr;
	std::unique_ptr<smpl::collision::CollisionSpace> m_cc_i;
	std::vector<std::unique_ptr<smpl::collision::CollisionShape>> m_collision_shapes;
	std::vector<std::unique_ptr<smpl::collision::CollisionObject>> m_collision_objects;

	PlannerConfig m_planning_config;
	smpl::PlanningParams m_planning_params;
	std::unique_ptr<smpl::PlannerInterface> m_planner;
	bool m_planner_init;
	std::vector<double> m_goal_vec;
	moveit_msgs::Constraints m_goal;
	std::string m_chain_tip_link;
	trajectory_msgs::JointTrajectory m_traj;
	std::vector<std::vector<Eigen::Vector3d>> m_traj_voxels;

	int m_t, m_priority;
	Trajectory m_solve;
	Object m_ooi;
	std::vector<moveit_msgs::CollisionObject> m_movables;
	std::vector<Object> m_objs;
	std::shared_ptr<CollisionChecker> m_cc;

	std::shared_ptr<BulletSim> m_sim;
	std::vector<trajectory_msgs::JointTrajectory> m_push_trajs, m_push_actions;
	int m_pushes_per_object, m_push_splits, m_grasp_tries, m_invvel_iters;
	double m_plan_push_time, m_grasp_lift, m_grasp_z, m_Kp, m_Ki, m_Kd, m_dt;

	std::map<std::string, double> m_stats;
	double m_planner_time, m_sim_time;

	void getPushStartPose(
		const std::vector<double>& push,
		Eigen::Affine3d& push_pose);
	bool planToPoseGoal(
		const Eigen::Affine3d& pose_goal,
		trajectory_msgs::JointTrajectory& push_traj);
	bool computePushAction(
		const double time_start,
		const smpl::RobotState& jnt_positions,
		const smpl::RobotState& jnt_velocities,
		const Eigen::Affine3d& end_pose,
		trajectory_msgs::JointTrajectory& action);

	void getRandomState(smpl::RobotState& s);
	bool reinitStartState();

	void coordToState(const Coord& coord, State& state) const;
	void stateToCoord(const State& state, Coord& coord) const;

	// Robot Model functions
	bool readRobotModelConfig(const ros::NodeHandle &nh);
	bool setupRobotModel();
	bool readStartState();
	bool setReferenceStartState();
	bool setGripper(bool open);
	bool readResolutions(std::vector<double>& resolutions);

	void initObjects();
	void reinitObjects(const State& s);

	double profileAction(
		const smpl::RobotState& parent,
		const smpl::RobotState& succ);

	void initOccupancyGrids();
	bool initCollisionChecker();
	bool getCollisionObjectMsg(
		const Object& object,
		moveit_msgs::CollisionObject& obj_msg,
		bool remove);
	bool processCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object);
	bool processSTLMesh(
		const Object& object, bool remove);

	bool addCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object);
	bool removeCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object);
	bool checkCollisionObjectSanity(
		const moveit_msgs::CollisionObject& object) const;
	auto findCollisionObject(const std::string& id) const
		-> smpl::collision::CollisionObject*;
	bool setCollisionRobotState();

	auto makePathVisualization() const
	-> std::vector<smpl::visual::Marker>;

	bool initPlanner();
	bool readPlannerConfig(const ros::NodeHandle &nh);
	bool createPlanner(bool interp);
	void fillGoalConstraint();
	void createMultiPoseGoalConstraint(moveit_msgs::MotionPlanRequest& req);
	void createPoseGoalConstraint(
		const Eigen::Affine3d& pose,
		moveit_msgs::MotionPlanRequest& req);
	void createJointSpaceGoal(
		const smpl::RobotState& pose,
		moveit_msgs::MotionPlanRequest& req);
	void addPathConstraint(moveit_msgs::Constraints& path_constraints);
	bool getStateNearPose(
		const Eigen::Affine3d& pose,
		const smpl::RobotState& seed_state,
		smpl::RobotState& state,
		int N=2,
		const std::string& ns="");

	bool attachAndCheckObject(
		const Object& object, const smpl::RobotState& state);
	bool attachObject(const Object& obj);
	bool detachObject();
	void displayObjectMarker(const Object& object);

	bool planApproach(
		const std::vector<std::vector<double> >& approach_cvecs,
		moveit_msgs::MotionPlanResponse& res,
		const std::vector<Object*>& movable_obstacles,
		bool finalise=false);
	bool planRetract(
		const std::vector<std::vector<double> >& retract_cvecs,
		moveit_msgs::MotionPlanResponse& res,
		const std::vector<Object*>& movable_obstacles,
		bool finalise=false);
	void voxeliseTrajectory();
};

} // namespace clutter


#endif // ROBOT_HPP
