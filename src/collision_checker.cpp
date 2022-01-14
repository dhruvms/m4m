#include <pushplan/collision_checker.hpp>
#include <pushplan/planner.hpp>
#include <pushplan/geometry.hpp>
#include <default_broadphase_callbacks.h>

#include <smpl/console/console.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace clutter
{

CollisionChecker::CollisionChecker(Planner* planner, const std::vector<Object>& obstacles)
:
m_planner(planner),
m_obstacles(obstacles),
m_rng(m_dev())
{
	m_fcl_immov = new fcl::DynamicAABBTreeCollisionManager();
	m_fcl_mov = new fcl::DynamicAABBTreeCollisionManager();

	// preprocess immovable obstacles
	for (size_t i = 0; i != m_obstacles.size(); ++i)
	{
		if (m_obstacles.at(i).id == 1) {
			m_base_loc = i;
			continue;
		}
		m_fcl_immov->registerObject(m_obstacles.at(i).GetFCLObject());
	}

	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
}

void CollisionChecker::InitMovableSet(const std::vector<std::shared_ptr<Agent> >& agents)
{
	for (size_t i = 0; i != agents->size(); ++i) {
		m_fcl_mov->registerObject(agents.at(i)->GetFCLObject());
	}
}

void CollisionChecker::UpdateTraj(const int& priority, const Trajectory& traj)
{
	if (int(m_trajs.size()) <= priority) {
		m_trajs.resize(priority + 1, {});
	}

	m_trajs.at(priority) = traj;
}

bool CollisionChecker::ImmovableCollision(const State& s, fcl::CollisionObject* o)
{
	LatticeState ls;
	ls.state = s;
	return this->ImmovableCollision(ls, o);
}

bool CollisionChecker::ImmovableCollision(const LatticeState& s, fcl::CollisionObject* o)
{
	// double start_time = GetTime(), time_taken;

	fcl::Transform3f pose;
	pose.setIdentity();
	fcl::Vec3f T(o->getTranslation());
	T.setValue(s.state.at(0), s.state.at(1), T[2]);
	pose.setTranslation(T);

	o->setTransform(pose);
	o->computeAABB();

	m_fcl_immov->setup();
	fcl::DefaultCollisionData collision_data;
	m_fcl_immov->collide(o, &collision_data, fcl::DefaultCollisionFunction);

	// time_taken = GetTime() - start_time;
	// SMPL_INFO("Immovable collision check: %f seconds.", time_taken);

	return collision_data.result.isCollision();
}

// // IsStateValid should only be called for priority > 1
// bool CollisionChecker::IsStateValid(
// 	const LatticeState& s, fcl::CollisionObject* o1, const int& priority)
// {
// 	// double start_time = GetTime(), time_taken;

// 	m_fcl_mov->unregisterObject(o1);
// 	auto o1_new = m_planner->GetObject(s, priority); // updates pose

// 	LatticeState robot;
// 	// Check against movables' FCL manager
// 	for (int p = 0; p < priority; ++p)
// 	{
// 		for (const auto& s2: m_trajs.at(p))
// 		{
// 			if (s.t == s2.t)
// 			{
// 				if (p == 0)
// 				{
// 					robot = s2; // store for later
// 					break;
// 				}
// 				else
// 				{
// 					auto o2 = m_planner->GetObject(s2, p);
// 					m_fcl_mov->update(o2);
// 				}
// 			}
// 		}
// 	}
// 	m_fcl_mov->setup();
// 	fcl::DefaultCollisionData collision_data;
// 	m_fcl_mov->collide(o1, &collision_data, fcl::DefaultCollisionFunction);
// 	bool collision = collision_data.result.isCollision();

// 	// time_taken = GetTime() - start_time;
// 	// SMPL_INFO("Movable collision check: %f seconds.", time_taken);

// 	// start_time = GetTime();
// 	// double start_time = GetTime(), time_taken;

// 	//  Check against robot collision
// 	if (!collision && !robot.state.empty()) {
// 		collision = collision || m_planner->CheckRobotCollision(robot, priority);
// 	}

// 	// time_taken = GetTime() - start_time;
// 	// SMPL_INFO("Robot collision check: %f seconds.", time_taken);

// 	m_fcl_mov->registerObject(o1_new);
// 	return !collision;
// }

State CollisionChecker::GetRandomStateOutside(fcl::CollisionObject* o)
{
	State g(2, 0.0);
	State gmin(2, 0.0), gmax(2, 0.0);

	gmin.at(0) = OutsideXMin();
	gmax.at(0) = OutsideXMax();

	gmin.at(1) = OutsideYMin();
	gmax.at(1) = OutsideYMax();

	do
	{
		g.at(0) = (m_distD(m_rng) * (gmax.at(0) - gmin.at(0))) + gmin.at(0);
		g.at(1) = (m_distD(m_rng) * (gmax.at(1) - gmin.at(1))) + gmin.at(1);
	}
	while (ImmovableCollision(g, o));

	return g;
}

} // namespace clutter
