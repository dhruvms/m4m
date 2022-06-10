#ifndef KPIECE_HPP
#define KPIECE_HPP

#include <pushplan/agents/robot.hpp>
#include <pushplan/search/planner.hpp>

#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/collision_space.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

#include <ros/ros.h>

namespace ob = ompl::base;

namespace clutter
{
class PushplanStateSampler : public ob::StateSampler
{
public:
	PushplanStateSampler(const ob::StateSpace *space, const comms::ObjectsPoses& objects) :
		ob::StateSampler(space)
	{
		m_objects = objects;
	}

	void sampleUniform(ob::State *state) override;
	void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override;
	void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override;
private:
	comms::ObjectsPoses m_objects;
};

class PushplanStateSpace : public ob::StateSpace
{
public:
	class StateType : public ob::State
	{
	public:
		StateType() = default;

		void setJointState(const smpl::RobotState& state);
		void copyObjectState(const comms::ObjectsPoses& objects);
		void setObjectState(const comms::ObjectsPoses& objects);

		void reserveJointState(std::size_t joints)
		{
			m_state.clear();
			m_state.resize(joints, 0.0);
		}

		void setJoint(int index, double value) {
			assert(m_state.size() >= index);
			m_state.at(index) = value;
		}

		double getJoint(int index) const {
			assert(m_state.size() >= index);
			return m_state.at(index);
		}

		auto objects() const -> const comms::ObjectsPoses& { return m_objects; };
		auto joint_state() const -> const smpl::RobotState& { return m_state; };
		int num_joints() const { m_state.size(); };
		int num_objects() const { m_objects.poses.size(); };
	private:
		smpl::RobotState m_state;
		comms::ObjectsPoses m_objects;
	};

	PushplanStateSpace(Planner* p);
	~PushplanStateSpace() = default;

	unsigned int getDimension() const override;
	double getMaximumExtent() const override;
	double getMeasure() const override;

	void enforceBounds(ob::State *state) const override;
	bool satisfiesBounds(const ob::State *state) const override;

	void copyState(ob::State *destination, const ob::State *source) const override;

	double distance(const ob::State *state1, const ob::State *state2) const override;
	bool equalStates(const ob::State *state1, const ob::State *state2) const override;
	void interpolate(const ob::State *from, const ob::State *to, double t, ob::State *state) const override;

	ob::StateSamplerPtr allocDefaultStateSampler() const override;
	ob::State *allocState() const override;
	void freeState(ob::State *state) const override;

	void copyToReals(std::vector<double> &reals, const ob::State *source) const;
	void copyFromReals(ob::State *destination, const std::vector<double> &reals) const;

	void SetStartObjects(const comms::ObjectsPoses& objects) { m_start_objects = objects; };
	auto GetStartObjects() const -> const comms::ObjectsPoses& { return m_start_objects; };

	int GetDimStateSpaceType(int dim) { return m_joint_to_subspace.at(dim)->getType(); };
	const ob::RealVectorBounds& GetStateSpaceBounds(int dim) const { return static_cast<ob::RealVectorStateSpace *>(m_joint_to_subspace.at(dim).get())->getBounds(); };

	auto GetRCM() const -> const smpl::collision::RobotCollisionModelConstPtr& { return m_rcm; };
	auto GetVariables() const -> const std::vector<int>& { return m_planning_joint_to_collision_model_indices; };
	void FillMotionInterpolation(
			const smpl::RobotState& s1,
			const smpl::RobotState& s2,
			smpl::collision::MotionInterpolation& motion) const;

	void RunIK(const Eigen::Affine3d& pose, smpl::RobotState& state);
	Eigen::Affine3d ComputeFK(const smpl::RobotState& state) const {
		return m_rm->computeFK(state);
	};

	int RMJointVariableCount() { return m_rm->jointVariableCount(); };
	double RMVelLimit(int jidx) { return m_rm->velLimit(jidx); };
	bool RMIsContinuous(int jidx) { return m_rm->isContinuous(jidx); };

	double GetSimTime() { return m_sim_time; };
	int GetNumSims() { return m_num_sims; };
	void AddSimTime(double val) { m_sim_time += val; };
	void IncrementSims() { m_num_sims++; };

protected:

private:
	std::vector<ob::StateSpacePtr> m_components;
	unsigned int m_componentCount{0u};
	std::vector<double> m_weights;
	double m_weightSum{0.0};
	bool m_locked{false};
	std::map<int, ob::StateSpacePtr> m_joint_to_subspace;

	Robot* m_robot;
	std::vector<std::string> m_planning_joints;
	smpl::PushingKDLRobotModel* m_rm;
	smpl::collision::RobotCollisionModelConstPtr m_rcm;
	smpl::collision::RobotMotionCollisionModelConstPtr m_rmcm;
	std::vector<int> m_planning_joint_to_collision_model_indices;

	comms::ObjectsPoses m_start_objects;
	smpl::RobotState m_last_ik;
	double m_sim_time = 0.0;
	int m_num_sims = 0.0;

	void addSubspace(const ob::StateSpacePtr &component, double weight);
	void addSubspaceToJointMap(const ob::StateSpacePtr &subspace, int joint);

	bool isLocked() const { return m_locked; };
	void lock() { m_locked = true;} ;
};

class PushplanMotionValidator : public ob::MotionValidator
{
public:
	PushplanMotionValidator(ob::SpaceInformation *si) : ob::MotionValidator(si)
	{
		defaultSettings();
	}
	PushplanMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si)
	{
		defaultSettings();
	}
	~PushplanMotionValidator() override = default;
	bool checkMotion(const ob::State *s1, const ob::State *s2) const override;
	bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;

	void SetRobot(Robot* robot) {
		m_robot = robot;
	};

private:
	PushplanStateSpace *m_state_space;
	Robot* m_robot;

	void defaultSettings();
};

class PushplanPoseGoal : public ob::GoalSampleableRegion
{
public:
	PushplanPoseGoal(
		const ompl::base::SpaceInformationPtr& si,
		const Eigen::Affine3d& pose = Eigen::Affine3d::Identity())
	:
		ob::GoalSampleableRegion(si),
		m_pose(pose)
	{};

	// Pure virtual functions from Goal
	bool isSatisfied(const ob::State *st) const override;

	// Pure virtual functions from GoalRegion
	double distanceGoal(const ob::State *st) const override;

	// Pure virtual functions from GoalSampleableRegion
	void sampleGoal(ob::State *st) const override;
	unsigned int maxSampleCount() const override;

	bool isSatisfied(const ob::State *st, double *distance) const override;

	void SetTolerances(
		const Eigen::Vector3d& position_tolerance,
		const Eigen::Vector3d& orientation_tolerance) {
		m_position_tolerance = position_tolerance;
		m_orientation_tolerance = orientation_tolerance;
	}
	void SetRobot(Robot* robot) {
		m_robot = robot;
	};

private:
	Eigen::Affine3d m_pose;
	Eigen::Vector3d m_position_tolerance;
	Eigen::Vector3d m_orientation_tolerance;
	Robot* m_robot;
};

struct ProjectionEvaluatorFK : public ob::ProjectionEvaluator
{
	smpl::PushingKDLRobotModel* model = NULL;
	const ompl::base::StateSpace* state_space = NULL;

	using Base = ompl::base::ProjectionEvaluator;

	ProjectionEvaluatorFK(const ompl::base::StateSpace* space);
	ProjectionEvaluatorFK(const ompl::base::StateSpacePtr& space);

	~ProjectionEvaluatorFK() = default;

	auto getDimension() const -> unsigned int override;
	void project(
			const ompl::base::State* state,
			ompl::base::EuclideanProjection& projection) const override;
	void setCellSizes(const std::vector<double>& cell_sizes) override;
	void defaultCellSizes() override;
	void setup() override;
	void printSettings(std::ostream& out = std::cout) const override;
	void printProjection(
			const ompl::base::EuclideanProjection& projection,
			std::ostream& out = std::cout) const override;
};

} // namespace clutter

#endif // KPIECE_HPP
