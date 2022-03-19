#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/types.hpp>
#include <pushplan/object.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/focal.hpp>
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

	bool SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f);

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

	void UpdatePose(const LatticeState&s) { m_obj.UpdatePose(s); };

private:
	ros::NodeHandle m_ph;
	Object m_obj;
	ObjectDesc m_obj_desc;

	std::unique_ptr<AgentLattice> m_lattice;
	std::string m_planning_frame;
	std::shared_ptr<smpl::DistanceMapInterface> m_df;
	std::unique_ptr<smpl::OccupancyGrid> m_ngr;
	std::shared_ptr<smpl::OccupancyGrid> m_obs_grid;
	std::vector<Eigen::Vector3d> m_ngr_complement;

	std::list<std::shared_ptr<Constraint> > m_constraints;
	std::vector<std::pair<int, Trajectory> >* m_cbs_solution; // all agent trajectories
	int m_cbs_id, m_max_time;

	LatticeState m_init;
	Coord m_start, m_goal;
	State m_goalf;
	int m_t, m_priority;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Focal> m_focal;

	void initNGR();
};

} // namespace clutter


#endif // AGENT_HPP
