#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/agents/agent.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/utils/bullet_sim.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/types.hpp>
#include <comms/ObjectsPoses.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class CBS;

namespace sampling
{
class SamplingPlanner;
} // namespace sampling

class Planner
{
public:
	Planner() : m_num_objs(-1), m_scene_id(-1),
				m_ph("~"), m_replan(true),
				m_plan_success(false), m_sim_success(false),
				m_push_input(false), m_rng(m_dev()) {};
	bool Init(const std::string& scene_file, int scene_id, bool ycb);
	bool Alive();
	bool SetupNGR();

	bool Plan(bool& done);
	bool FinalisePlan(bool add_movables=true);
	bool SaveData();
	bool Rearrange();
	std::uint32_t RunSim();
	bool TryExtract();
	void AnimateSolution();
	bool RunRRT();
	void RunStudy(int study);

	Agent* GetAgent(const int& id) {
		assert(id > 0); // 0 is robot
		return m_agents.at(m_agent_map[id]).get();
	}

	const std::vector<Object>* Get2DRobot(const LatticeState& s) {
		return m_robot->GetObject(s);
	}
	const Object* GetObject(int id) {
		return m_agents.at(m_agent_map[id])->GetObject();
	};

	bool CheckRobotCollision(Agent* a, const LatticeState& robot_state, int t, bool process)
	{
		if (!process) {
			return m_robot->CheckCollision(robot_state, t);
		}
		return m_robot->CheckCollisionWithObject(robot_state, a, t);
	}

	std::vector<Agent*> GetAllAgents()
	{
		std::vector<Agent*> all_agents;
		for(int i = 0; i < m_agents.size(); i++) {
			all_agents.push_back(m_agents[i].get());
		}
		return all_agents;
	}

	auto GetStartObjects() -> comms::ObjectsPoses
	{
		comms::ObjectsPoses start_objects;
		for (const auto& a: m_agents) {
			auto object = a->GetObject();

			comms::ObjectPose obj_pose;
			obj_pose.id = object->desc.id;
			obj_pose.xyz = { object->desc.o_x, object->desc.o_y, object->desc.o_z };
			obj_pose.rpy = { object->desc.o_roll, object->desc.o_pitch, object->desc.o_yaw };
			start_objects.poses.push_back(std::move(obj_pose));
		}

		return start_objects;
	}

	bool Replan() { return m_replan; };

	// For KPIECE
	Robot* GetRobot() { return m_robot.get(); };
	bool StateValidityChecker(const smpl::RobotState& state) {
		return m_robot->IsStateValid(state);
	};
	void GetStartState(smpl::RobotState& state) {
		m_robot->GetHomeState(state);
	}
	auto GoalPose() -> Eigen::Affine3d {
		return m_robot->PregraspPose();
	}
	bool ExecTraj(const trajectory_msgs::JointTrajectory& traj, int grasp_at=-1) {
		return m_sim->ExecTraj(traj, GetStartObjects(), grasp_at, m_ooi->GetID());
	}

	// For PP
	bool RunPP();
	fcl::CollisionObject* GetUpdatedObjectFromPriority(const LatticeState& s, int priority)
	{
		m_agents.at(m_priorities.at(priority))->UpdatePose(s);
		return m_agents.at(m_priorities.at(priority))->GetFCLObject();
	}

private:
	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;
	std::shared_ptr<Robot> m_robot;
	std::shared_ptr<BulletSim> m_sim;
	std::shared_ptr<CBS> m_cbs;
	std::shared_ptr<sampling::SamplingPlanner> m_sampling_planner;
	bool m_replan, m_plan_success, m_sim_success, m_push_input;

	int m_num_objs, m_scene_id;
	std::vector<std::shared_ptr<Agent> > m_agents;
	std::shared_ptr<Agent> m_ooi;
	std::unordered_map<int, size_t> m_agent_map;
	Coord m_ooi_g;
	State m_ooi_gf;
	std::vector<double> m_goal;

	trajectory_msgs::JointTrajectory m_exec;
	std::vector<trajectory_msgs::JointTrajectory> m_rearrangements;
	comms::ObjectsPoses m_rearranged;

	HighLevelNode* m_cbs_soln;
	std::unordered_map<int, std::size_t> m_cbs_soln_map;
	int m_moved;
	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::vector<double> m_alphas, m_betas;
	int m_C;

	ros::NodeHandle m_ph, m_nh;
	std::uint32_t m_violation;

	std::map<std::string, double> m_stats, m_cbs_stats;
	double m_plan_budget, m_sim_budget, m_total_budget, m_timer;

	bool createCBS();

	bool runSim();
	bool animateSolution();
	bool rearrange();
	int chooseObjDTS();

	bool setupProblem(bool backwards);
	void updateAgentPositions(
		const comms::ObjectsPoses& result,
		comms::ObjectsPoses& rearranged);
	int cleanupLogs();

	void init_agents(
		bool ycb, std::vector<Object>& obstacles);
	void parse_scene(std::vector<Object>& obstacles);
	void writePlanState(int iter);
	void setupGlobals();
	int armId();

	bool savePlanData();

	// For PP
	std::vector<size_t> m_priorities;
	void prioritise();
	void writePPState(std::set<Coord, coord_compare> ngr={});
};

} // namespace clutter


#endif // PLANNER_HPP
