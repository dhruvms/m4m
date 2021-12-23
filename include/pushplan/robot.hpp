#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <pushplan/types.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/bullet_sim.hpp>
#include <comms/ObjectsPoses.h>

#include <smpl/console/console.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/planning_params.h>
#include <smpl/debug/marker.h>
#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
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

class Robot
{
public:
	Robot() : m_ph("~"), m_rng(m_dev()) {};


	bool Setup();
	bool SavePushData(int scene_id);
	bool ProcessObstacles(const std::vector<Object>& obstacles, bool remove=false);
	bool Init();
	bool RandomiseStart();
	bool Plan(const Object& ooi, boost::optional<std::vector<Object>> obstacles=boost::none);

	void ProfileTraj(Trajectory& traj);
	bool ComputeGrasps(
		const std::vector<double>& pregrasp_goal,
		const Object& ooi);
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
	void Step(int k);

	bool UpdateKDLRobot(int mode);
	bool InitArmPlanner(bool interp=false);
	void SetPushGoal(const std::vector<double>& push);
	bool PlanPush(
		int oid, const Trajectory* o_traj, const Object& o,
		const comms::ObjectsPoses& rearranged,
		comms::ObjectsPoses& result);
	trajectory_msgs::JointTrajectory GetLastPlan() {
		SMPL_INFO("rearrangmenet traj size = %d", m_traj.points.size());
		return m_traj;
	};
	void SetSim(const std::shared_ptr<BulletSim>& sim) {
		m_sim = sim;
	}

	void AnimateSolution();

	const LatticeState* GetCurrentState() const { return &m_current; };
	const Trajectory* GetMoveTraj() const { return &m_move; };
	void GetExecTraj(trajectory_msgs::JointTrajectory& traj) const { traj = m_exec; };

	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	const std::vector<Object>* GetObject() const { return &m_objs; };
	const std::vector<Object>* GetObject(const LatticeState& s);
	const std::vector<Object>* GetGraspObjs() {
		return &m_grasp_objs;
	};

	Coord GetEECoord();
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

private:
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::KDLRobotModel> m_rm;
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
	double m_mass, m_b, m_table_z;
	std::string m_shoulder, m_elbow, m_wrist, m_tip;
	const smpl::urdf::Link* m_link_s = nullptr;
	const smpl::urdf::Link* m_link_e = nullptr;
	const smpl::urdf::Link* m_link_w = nullptr;
	const smpl::urdf::Link* m_link_t = nullptr;
	smpl::RobotState m_pregrasp_state, m_grasp_state, m_postgrasp_state;
	std::vector<Object> m_grasp_objs;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::normal_distribution<> m_distG;

	std::shared_ptr<smpl::DistanceMapInterface> m_df_i;
	std::unique_ptr<smpl::OccupancyGrid> m_grid_i;
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
	trajectory_msgs::JointTrajectory m_traj, m_exec;

	int m_t, m_priority;
	LatticeState m_current, m_init;
	Trajectory m_solve, m_move;
	std::vector<Object> m_objs;
	std::shared_ptr<CollisionChecker> m_cc;

	std::shared_ptr<BulletSim> m_sim;
	std::vector<trajectory_msgs::JointTrajectory> m_pushes;
	int m_pushes_per_object, m_grasp_tries;
	double m_plan_push_time, m_grasp_lift, m_grasp_z;

	std::map<std::string, double> m_stats;
	double m_planner_time, m_sim_time;

	bool samplePush(
		const Trajectory* object, const std::vector<Object>& obs,
		smpl::RobotState& push_start, smpl::RobotState& push_end);

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

	void initOccupancyGrid();
	bool initCollisionChecker();
	bool getCollisionObjectMsg(
		const Object& object,
		moveit_msgs::CollisionObject& obj_msg,
		bool remove);
	bool processCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable=false);
	bool processSTLMesh(
		const int& id, const geometry_msgs::Pose& pose,
		const std::string& stl_mesh, bool remove, bool movable=false);

	bool addCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable);
	bool removeCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable);
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

	bool attachOOI(const Object& ooi);
	bool detachOOI();
	void displayObjectMarker(const Object& ooi);
};

} // namespace clutter


#endif // ROBOT_HPP
