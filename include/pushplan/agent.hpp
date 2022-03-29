#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/object.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/cbs_nodes.hpp>
#include <pushplan/conflicts.hpp>
#include <pushplan/agent_lattice.hpp>

#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/occupancy_grid.h>
#include <ros/ros.h>

#include <vector>
#include <string>
#include <memory>

namespace clutter
{

class Agent
{
public:
	Agent() : m_ph("~") {};
	Agent(const Object& o) : m_ph("~")
	{
		m_obj = o;
		m_obj_desc = o.desc;
	};
	void SetObject(const Object& o)
	{
		m_obj = o;
		m_obj_desc = o.desc;
	}

	// void ResetObject();
	// bool SetObjectPose(
	// 	const std::vector<double>& xyz,
	// 	const std::vector<double>& rpy);
	bool Init();
	void UpdateNGR(const std::vector<std::vector<Eigen::Vector3d>>& voxels);
	void ComputeNGRComplement();

	bool SatisfyPath(
		HighLevelNode* ct_node,
		Trajectory** sol_path,
		int& expands,
		int& min_f,
		std::unordered_set<int>* to_avoid = nullptr);

	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	void SetObstacleGrid(const std::shared_ptr<smpl::OccupancyGrid>& obs_grid) {
		m_obs_grid = obs_grid;
	}

	void GetSE2Push(std::vector<double>& push);
	const Trajectory* GetLastTraj() const { return &m_solve; };
	int GetID() { return m_obj.id; };

	const std::vector<Object>* GetObject() const { return &m_obj};
	const std::vector<Object>* GetObject(const LatticeState& s);
	fcl::CollisionObject* GetFCLObject() { return m_obj.GetFCLObject(); };
	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		m_obj.GetMoveitObj(msg);
	};

	void UpdatePose(const LatticeState&s);
	bool OutOfBounds(const LatticeState& s);
	bool ImmovableCollision();
	bool ObjectObjectCollision(
		const LatticeState& s, const int& a2_id, const LatticeState& a2_q);
	bool ObjectObjectsCollision(
			const std::vector<int>& other_ids,
			const std::vector<LatticeState>& other_poses);

private:
	ros::NodeHandle m_ph;
	Object m_obj;
	ObjectDesc m_obj_desc;
	LatticeState m_init;
	Coord m_goal;

	std::unique_ptr<AgentLattice> m_lattice;
	std::string m_planning_frame;
	std::shared_ptr<smpl::DistanceMapInterface> m_df;
	std::unique_ptr<smpl::OccupancyGrid> m_ngr;
	std::shared_ptr<smpl::OccupancyGrid> m_obs_grid;
	std::vector<Eigen::Vector3d> m_ngr_complement;
	std::vector<LatticeState> m_ngr_complement_states;

	State m_goalf;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Search> m_search;

	void initNGR();
	bool computeGoal(bool backwards);

	// check collisions with static obstacles
	bool stateObsCollision(const LatticeState$ s);
	// check collisions with NGR
	bool stateOutsideNGR(const LatticeState$ s);
};

} // namespace clutter


#endif // AGENT_HPP
