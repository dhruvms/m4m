#include <pushplan/sampling/ompl.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/helpers.hpp>

#include <smpl/time.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Exception.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <limits>
#include <random>

namespace ob = ompl::base;
using namespace boost::math::double_constants;

namespace clutter
{

void PushplanStateSampler::sampleUniform(ob::State *state)
{
	const unsigned int dim = space_->getDimension();

	auto *rstate = static_cast<PushplanStateSpace::StateType *>(state);
	auto *rspace = const_cast<PushplanStateSpace *>(static_cast<const PushplanStateSpace *>(space_));
	for (unsigned int i = 0; i < dim; ++i)
	{
		int type = rspace->GetDimStateSpaceType(i);

		switch (type) {
			case ob::StateSpaceType::STATE_SPACE_REAL_VECTOR:
			{
				ob::RealVectorBounds bounds = rspace->GetStateSpaceBounds(i);
				rstate->setJoint(i, rng_.uniformReal(bounds.low.at(0), bounds.high.at(0)));
				break;
			}
			case ob::StateSpaceType::STATE_SPACE_SO2:
			{
				rstate->setJoint(i, rng_.uniformReal(-pi, pi));
				break;
			}
			default:
				ROS_WARN("Skip unrecognized joint type");
				break;
		}
	}

	rstate->copyObjectState(m_objects);
}

void PushplanStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, double distance)
{
	const unsigned int dim = space_->getDimension();

	auto *rstate = static_cast<PushplanStateSpace::StateType *>(state);
	auto *rspace = const_cast<PushplanStateSpace *>(static_cast<const PushplanStateSpace *>(space_));
	const auto *rnear = static_cast<const PushplanStateSpace::StateType *>(near);
	for (unsigned int i = 0; i < dim; ++i)
	{
		int type = rspace->GetDimStateSpaceType(i);
		switch (type) {
			case ob::StateSpaceType::STATE_SPACE_REAL_VECTOR:
			{
				ob::RealVectorBounds bounds = rspace->GetStateSpaceBounds(i);
				double v = rng_.uniformReal(
										std::max(bounds.low.at(0), rnear->joint_state().at(i) - distance),
										std::min(bounds.high.at(0), rnear->joint_state().at(i) + distance));
				rstate->setJoint(i, v);
				break;
			}
			case ob::StateSpaceType::STATE_SPACE_SO2:
			{
				double v = rng_.uniformReal(rnear->joint_state().at(i) - distance, rnear->joint_state().at(i) + distance);
				v = std::fmod(v, 2.0 * pi);
				if (v < -pi)
					v += 2.0 * pi;
				else if (v >= pi)
					v -= 2.0 * pi;

				rstate->setJoint(i, v);
				break;
			}
			default:
				ROS_WARN("Skip unrecognized joint type");
				break;
		}
	}

	rstate->copyObjectState(rnear->objects());
}

void PushplanStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, double stdDev)
{
	const unsigned int dim = space_->getDimension();

	auto *rstate = static_cast<PushplanStateSpace::StateType *>(state);
	auto *rspace = const_cast<PushplanStateSpace *>(static_cast<const PushplanStateSpace *>(space_));
	const auto *rmean = static_cast<const PushplanStateSpace::StateType *>(mean);
	for (unsigned int i = 0; i < dim; ++i)
	{
		int type = rspace->GetDimStateSpaceType(i);
		double v = rng_.gaussian(rmean->joint_state().at(i), stdDev);
		switch (type) {
			case ob::StateSpaceType::STATE_SPACE_REAL_VECTOR:
			{
				ob::RealVectorBounds bounds = rspace->GetStateSpaceBounds(i);
				if (v < bounds.low.at(0))
					v = bounds.low.at(0);
				else if (v > bounds.high.at(0))
					v = bounds.high.at(0);

				rstate->setJoint(i, v);
				break;
			}
			case ob::StateSpaceType::STATE_SPACE_SO2:
			{
				v = std::fmod(v, 2.0 * pi);
				if (v < -pi)
					v += 2.0 * pi;
				else if (v >= pi)
					v -= 2.0 * pi;

				rstate->setJoint(i, v);
				break;
			}
			default:
				ROS_WARN("Skip unrecognized joint type");
				break;
		}
	}

	rstate->copyObjectState(rmean->objects());
}

PushplanStateSpace::PushplanStateSpace(Planner* p)
{
	m_robot = p->GetRobot();
	m_planning_joints = m_robot->GetPlanningJoints();
	m_rm = m_robot->RobotModel();
	m_rcm = m_robot->GetRCM();
	m_rmcm = m_robot->GetRMCM();

	// map planning joint indices to collision model indices
	m_planning_joint_to_collision_model_indices.resize(m_planning_joints.size(), -1);

	for (std::size_t i = 0; i < m_planning_joints.size(); ++i)
	{
		auto& joint_name = m_planning_joints[i];
		int jidx = m_rcm->jointVarIndex(joint_name);

		m_planning_joint_to_collision_model_indices[i] = jidx;
	}

	int joint_num = 0;
	for (auto& joint_name : m_planning_joints)
	{
		int jvaridx = m_rcm->jointVarIndex(joint_name);
		int jidx = m_rcm->jointVarJointIndex(jvaridx);
		auto jtype = m_rcm->jointType(jidx);
		switch (jtype) {
			case smpl::collision::JointType::FIXED:
				break;
			case smpl::collision::JointType::REVOLUTE:
			case smpl::collision::JointType::PRISMATIC:
			{
				auto* subspace = new ob::RealVectorStateSpace(1);
				subspace->setBounds(m_rcm->jointVarMinPosition(jvaridx), m_rcm->jointVarMaxPosition(jvaridx));
				subspace->setName(joint_name);

				ob::StateSpacePtr subspace_ptr(subspace);
				addSubspace(subspace_ptr, 1.0);
				addSubspaceToJointMap(subspace_ptr, joint_num);
				break;
			}
			case smpl::collision::JointType::CONTINUOUS:
			{
				auto* subspace = new ob::SO2StateSpace;
				subspace->setName(joint_name);
				ob::StateSpacePtr subspace_ptr(subspace);
				addSubspace(subspace_ptr, 1.0);
				addSubspaceToJointMap(subspace_ptr, joint_num);
				break;
			}
			case smpl::collision::JointType::PLANAR:
			case smpl::collision::JointType::FLOATING:
			default:
			{
				ROS_WARN("Skip unrecognized joint type");
				break;
			}
		}
		joint_num++;
	}

	lock(); // is this needed?
}

unsigned int PushplanStateSpace::getDimension() const
{
	return (int)m_planning_joints.size();
}

double PushplanStateSpace::getMaximumExtent() const
{
	double e = 0.0;
	for (unsigned int i = 0; i < m_componentCount; ++i)
	{
		if (m_weights[i] >= std::numeric_limits<double>::epsilon()) {  // avoid possible multiplication of 0 times infinity
			e += m_weights[i] * m_components[i]->getMaximumExtent();
		}
	}
	return e;
}

double PushplanStateSpace::getMeasure() const
{
	double m = 1.0;
	for (unsigned int i = 0; i < m_componentCount; ++i)
	{
		if (m_weights[i] >= std::numeric_limits<double>::epsilon()) {  // avoid possible multiplication of 0 times infinity
			m *= m_weights[i] * m_components[i]->getMeasure();
		}
	}
	return m;
}

void PushplanStateSpace::enforceBounds(ob::State *state) const
{
	for (std::size_t i = 0; i < state->as<StateType>()->joint_state().size(); ++i)
	{
		int type = m_joint_to_subspace.at(i)->getType();
		switch (type) {
			case ob::StateSpaceType::STATE_SPACE_REAL_VECTOR:
			{
				ob::RealVectorBounds bounds = GetStateSpaceBounds(i);
				double v = state->as<StateType>()->joint_state().at(i);
				if (v > bounds.high.at(0))
					state->as<StateType>()->setJoint(i, bounds.high.at(0));
				else if (v < bounds.low.at(0))
					state->as<StateType>()->setJoint(i, bounds.low.at(0));
				break;
			}
			case ob::StateSpaceType::STATE_SPACE_SO2:
			{
				double v = std::fmod(state->as<StateType>()->joint_state().at(i), 2.0 * pi);
				if (v < -pi)
					v += 2.0 * pi;
				else if (v >= pi)
					v -= 2.0 * pi;

				state->as<StateType>()->setJoint(i, v);
				break;
			}
			default:
				ROS_WARN("Skip unrecognized joint type");
				break;
		}
	}
}

bool PushplanStateSpace::satisfiesBounds(const ob::State *state) const
{
	smpl::RobotState joints = state->as<StateType>()->joint_state();
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		int type = m_joint_to_subspace.at(i)->getType();
		switch (type) {
			case ob::StateSpaceType::STATE_SPACE_REAL_VECTOR:
			{
				ob::RealVectorBounds bounds = GetStateSpaceBounds(i);
				if (joints.at(i) - std::numeric_limits<double>::epsilon() > bounds.high.at(0) ||
					joints.at(i) + std::numeric_limits<double>::epsilon() < bounds.low.at(0)) {
					return false;
				}
				break;
			}
			case ob::StateSpaceType::STATE_SPACE_SO2:
			{
				auto theta = smpl::angles::normalize_angle(joints.at(i));
				if ((theta - std::numeric_limits<double>::epsilon() > pi) ||
					(theta + std::numeric_limits<double>::epsilon() < -pi)) {
					return false;
				}
				break;
			}
			default:
				ROS_WARN("Skip unrecognized joint type");
				break;
		}
	}
	return true;
}

void PushplanStateSpace::copyState(ob::State *destination, const ob::State *source) const
{
	destination->as<StateType>()->setJointState(source->as<StateType>()->joint_state());
	destination->as<StateType>()->copyObjectState(source->as<StateType>()->objects());
}

double PushplanStateSpace::distance(const ob::State *state1, const ob::State *state2) const
{
	assert(satisfiesBounds(state1));
	assert(satisfiesBounds(state2));

	// compute only joint distance
	int joints = state1->as<StateType>()->num_joints();
	double dsum = 0.0;
	for (int i = 0; i < joints; ++i) {
		double dj = (state1->as<StateType>()->joint_state().at(i) - state2->as<StateType>()->joint_state().at(i));
		dsum += dj * dj;
	}
	return std::sqrt(dsum);
}

bool PushplanStateSpace::equalStates(const ob::State *state1, const ob::State *state2) const
{
	// compare only joint configurations
	int joints = state1->as<StateType>()->num_joints();
	for (int i = 0; i < joints; ++i)
	{
		if (std::fabs(state1->as<StateType>()->joint_state().at(i) - state2->as<StateType>()->joint_state().at(i)) >
				std::numeric_limits<double>::epsilon() * 2.0)
		{
			return false;
		}
	}

	// TOOD - compare objects?

	return true;
}

void PushplanStateSpace::interpolate(const ob::State *from, const ob::State *to, double t, ob::State *state) const
{
	const double res = DEG5/5;
	smpl::collision::MotionInterpolation interp(m_rcm.get());

	m_rmcm->fillMotionInterpolation(
				from->as<StateType>()->joint_state(),
				to->as<StateType>()->joint_state(),
				m_planning_joint_to_collision_model_indices,
				res,
				interp);

	int state_idx = t * interp.waypointCount();
	smpl::RobotState interm;
	interp.interpolate(state_idx, interm, m_planning_joint_to_collision_model_indices);

	state->as<StateType>()->setJointState(interm);
	// TODO - this seems wrong
	state->as<StateType>()->copyObjectState(from->as<StateType>()->objects());
}

ob::StateSamplerPtr PushplanStateSpace::allocDefaultStateSampler() const
{
	return std::make_shared<PushplanStateSampler>(this, m_start_objects);
}

ob::State *PushplanStateSpace::allocState() const
{
	auto *rstate = new StateType();
	rstate->as<StateType>()->reserveJointState(m_planning_joints.size());
	for (int i = 0; i < m_planning_joints.size(); ++i) {
		rstate->as<StateType>()->setJoint(i, 0.0);
	}
	rstate->as<StateType>()->copyObjectState(m_start_objects);
	return rstate;
}

void PushplanStateSpace::freeState(ob::State *state) const
{
	auto *rstate = static_cast<StateType *>(state);
	delete rstate;
}

void PushplanStateSpace::copyToReals(std::vector<double> &reals, const ob::State *source) const
{
	auto state = source->as<StateType>()->joint_state();
	reals.clear();
	reals = state;
}

void PushplanStateSpace::copyFromReals(ob::State *destination, const std::vector<double> &reals) const
{
	destination->as<StateType>()->setJointState(reals);
}

void PushplanStateSpace::FillMotionInterpolation(
	const smpl::RobotState& s1,
	const smpl::RobotState& s2,
	smpl::collision::MotionInterpolation& motion) const
{
	const double res = DEG5/5;
	m_rmcm->fillMotionInterpolation(
				s1, s2,
				m_planning_joint_to_collision_model_indices,
				res, motion);
}

void PushplanStateSpace::RunIK(const Eigen::Affine3d& pose, smpl::RobotState& state)
{
	smpl::RobotState seed;
	m_robot->GetRandomState(seed);
	m_rm->computeFK(seed);
	if (m_last_ik.empty())
	{
		while (!m_rm->computeIKSearch(pose, seed, state)) {
			m_robot->GetRandomState(seed);
		}
		m_last_ik = state;
	}
	else
	{
		int tries = 0;
		while (tries < 5 && !m_rm->computeIKSearch(pose, seed, state))
		{
			m_robot->GetRandomState(seed);
			tries++;
		}
		if (tries == 5) {
			state = m_last_ik;
		}
		else {
			m_last_ik = state;
		}
	}
}

void PushplanStateSpace::addSubspace(const ob::StateSpacePtr &component, double weight)
{
	if (m_locked)
		throw ompl::Exception("This state space is locked. No further components can be added");
	if (weight < 0.0)
		throw ompl::Exception("Subspace weight cannot be negative");
	m_components.push_back(component);
	m_weights.push_back(weight);
	m_weightSum += weight;
	m_componentCount = m_components.size();
}

void PushplanStateSpace::addSubspaceToJointMap(const ob::StateSpacePtr &subspace, int joint)
{
	assert(m_joint_to_subspace.find(joint) == m_joint_to_subspace.end());
	m_joint_to_subspace.insert({joint, subspace});
}

void PushplanStateSpace::StateType::setJointState(const smpl::RobotState& state)
{
	m_state = state;
}

void PushplanStateSpace::StateType::copyObjectState(const comms::ObjectsPoses& objects)
{
	m_objects = objects;
}

void PushplanStateSpace::StateType::setObjectState(const comms::ObjectsPoses& objects)
{
	for (std::size_t i = 0; i < m_objects.poses.size(); i++)
	{
		int loc = -1;
		for (auto it = objects.poses.begin(); it != objects.poses.end(); ++it)
		{
			if (it->id == m_objects.poses.at(i).id)
			{
				loc = std::distance(objects.poses.begin(), it);
			}
		}

		if (loc != -1)
		{
			m_objects.poses.at(i).xyz = objects.poses.at(loc).xyz;
			m_objects.poses.at(i).rpy = objects.poses.at(loc).rpy;
		}
		else
			ROS_ERROR("Object id %d not found in vector", objects.poses.at(loc).id);
	}
}

void PushplanMotionValidator::defaultSettings()
{
	m_state_space = dynamic_cast<PushplanStateSpace *>(si_->getStateSpace().get());
	if (m_state_space == nullptr)
		throw ompl::Exception("No state space for motion validator");
}

bool PushplanMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,
													std::pair<ob::State *, double> &lastValid) const
{
	if (!m_robot->SetScene(s1->as<PushplanStateSpace::StateType>()->objects()))
	{
		m_state_space->copyState(lastValid.first, s1);
		lastValid.second = 0.0;
		invalid_++;
		return false;
	}

	smpl::RobotState qnew;
	comms::ObjectsPoses qnew_objs;
	std::uint32_t result;

	double start_time = GetTime();
	bool steer_result = m_robot->SteerAction(
			s2->as<PushplanStateSpace::StateType>()->joint_state(), std::numeric_limits<int>::max(),
			s1->as<PushplanStateSpace::StateType>()->joint_state(), s1->as<PushplanStateSpace::StateType>()->objects(),
			qnew, qnew_objs, result);
	double sim_time = GetTime() - start_time;

	if (steer_result)
	{
		lastValid.first->as<PushplanStateSpace::StateType>()->setJointState(qnew);
		lastValid.first->as<PushplanStateSpace::StateType>()->copyObjectState(s1->as<PushplanStateSpace::StateType>()->objects());
		lastValid.first->as<PushplanStateSpace::StateType>()->setObjectState(qnew_objs);
		lastValid.second = 1.0;

		valid_++;
	}
	else
	{
		m_state_space->copyState(lastValid.first, s1);
		lastValid.second = 0.0;
		invalid_++;
	}

	if (result >= 9 && result <= 101)
	{
		m_state_space->AddSimTime(sim_time);
		m_state_space->IncrementSims();
	}

	return steer_result;
}

bool PushplanMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
	std::pair<ob::State *, double> lastValid;
	lastValid.first = m_state_space->allocState();
	bool steer_result = this->checkMotion(s1, s2, lastValid);
	if (steer_result) {
		valid_++;
	}
	else {
		invalid_++;
	}

	return steer_result;
}

static
bool WithinPositionTolerance(
	const Eigen::Affine3d& A,
	const Eigen::Affine3d& B,
	const Eigen::Vector3d tol)
{
	auto dx = std::fabs(A.translation()[0] - B.translation()[0]);
	auto dy = std::fabs(A.translation()[1] - B.translation()[1]);
	auto dz = std::fabs(A.translation()[2] - B.translation()[2]);
	return dx <= tol[0] && dy <= tol[1] && dz <= tol[2];
}

static
bool WithinOrientationTolerance(
	const Eigen::Affine3d& A,
	const Eigen::Affine3d& B,
	const Eigen::Vector3d tol)
{
	Eigen::Quaterniond qg(B.rotation());
	Eigen::Quaterniond q(A.rotation());
	if (q.dot(qg) < 0.0) {
		qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
	}

	auto theta = smpl::angles::normalize_angle(2.0 * acos(q.dot(qg)));
	return theta < tol[0];
}

static
auto WithinTolerance(
	const Eigen::Affine3d& A,
	const Eigen::Affine3d& B,
	const Eigen::Vector3d xyz_tolerance,
	const Eigen::Vector3d rpy_tolerance)
	-> std::pair<bool, bool>
{
	if (WithinPositionTolerance(A, B, xyz_tolerance))
	{
		if (WithinOrientationTolerance(A, B, rpy_tolerance)){
			return std::make_pair(true, true);
		}
		else {
			return std::make_pair(true, false);
		}
	}
	return std::make_pair(false, false);
}

bool PushplanPoseGoal::isSatisfied(const ob::State *st) const
{
	auto* state_space = dynamic_cast<PushplanStateSpace *>(si_->getStateSpace().get());
	auto pose = state_space->ComputeFK(st->as<PushplanStateSpace::StateType>()->joint_state());

	auto near = WithinTolerance(pose, m_pose,
								m_position_tolerance, m_orientation_tolerance);

	if (near.first && near.second)
	{
		return m_robot->CheckGraspTraj(
			st->as<PushplanStateSpace::StateType>()->joint_state(),
			st->as<PushplanStateSpace::StateType>()->objects());
	}

	return false;
}

bool PushplanPoseGoal::isSatisfied(const ob::State *st, double *distance) const
{
	auto* state_space = dynamic_cast<PushplanStateSpace *>(si_->getStateSpace().get());
	auto pose = state_space->ComputeFK(st->as<PushplanStateSpace::StateType>()->joint_state());

	auto near = WithinTolerance(pose, m_pose,
								m_position_tolerance, m_orientation_tolerance);
	*distance = distanceGoal(st);

	if (near.first && near.second)
	{
		return m_robot->CheckGraspTraj(
			st->as<PushplanStateSpace::StateType>()->joint_state(),
			st->as<PushplanStateSpace::StateType>()->objects());
	}

	return false;
}

double PushplanPoseGoal::distanceGoal(const ob::State *st) const
{
	auto* state_space = dynamic_cast<PushplanStateSpace *>(si_->getStateSpace().get());

	smpl::RobotState goal_state;
	state_space->RunIK(m_pose, goal_state);

	auto* state = state_space->allocState();
	state->as<PushplanStateSpace::StateType>()->setJointState(goal_state);
	double distance = state_space->distance(state, st);
	state_space->freeState(state);

	return distance;
}

void PushplanPoseGoal::sampleGoal(ob::State *st) const
{
	auto* state_space = dynamic_cast<PushplanStateSpace *>(si_->getStateSpace().get());

	smpl::RobotState state;
	state_space->RunIK(m_pose, state);

	st->as<PushplanStateSpace::StateType>()->setJointState(state);
	st->as<PushplanStateSpace::StateType>()->copyObjectState(state_space->GetStartObjects());
}

unsigned int PushplanPoseGoal::maxSampleCount() const
{
	return std::numeric_limits<unsigned int>::max();
}

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
	const ompl::base::StateSpace* space)
:
	Base(space)
{
	state_space = space;
}

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
	const ompl::base::StateSpacePtr& space)
:
	Base(space)
{
	state_space = space.get();
}

auto ProjectionEvaluatorFK::getDimension() const -> unsigned int
{
	return 3;
}

void ProjectionEvaluatorFK::project(
		const ompl::base::State* state,
		ompl::base::EuclideanProjection& projection) const
{
	smpl::RobotState values;
	this->state_space->as<PushplanStateSpace>()->copyToReals(values, state);

	auto pose = model->computeFK(values);
	projection.resize(getDimension(), 0.0);
	projection[0] = pose.translation().x();
	projection[1] = pose.translation().y();
	projection[2] = pose.translation().z();
	// smpl::angles::get_euler_zyx(
	// 		pose.rotation(), projection[3], projection[4], projection[5]);
}

void ProjectionEvaluatorFK::setCellSizes(const std::vector<double>& cell_sizes)
{
	this->Base::setCellSizes(cell_sizes);
}

void ProjectionEvaluatorFK::defaultCellSizes()
{
	this->Base::defaultCellSizes();
}

void ProjectionEvaluatorFK::setup()
{
	this->Base::setup();
}

void ProjectionEvaluatorFK::printSettings(std::ostream& out) const
{
	this->Base::printSettings(out);
}

void ProjectionEvaluatorFK::printProjection(
	const ompl::base::EuclideanProjection& projection, std::ostream& out) const
{
	this->Base::printProjection(projection, out);
}

} // namespace clutter
