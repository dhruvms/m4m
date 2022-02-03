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

	// preprocess immovable obstacles
	for (size_t i = 0; i != m_obstacles.size(); ++i)
	{
		if (m_obstacles.at(i).id == 1) {
			m_base_loc = i;
			continue;
		}
		LatticeState s;
		s.state.push_back(m_obstacles.at(i).o_x);
		s.state.push_back(m_obstacles.at(i).o_y);
		m_obstacles.at(i).UpdatePose(s);
		m_fcl_immov->registerObject(m_obstacles.at(i).GetFCLObject());
	}

	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
}

void CollisionChecker::UpdateTraj(const int& priority, const Trajectory& traj)
{
	if (int(m_trajs.size()) <= priority) {
		m_trajs.resize(priority + 1, {});
	}

	m_trajs.at(priority) = traj;
}

bool CollisionChecker::OutOfBounds(const LatticeState& s)
{
	bool oob = s.state.at(0) <= (m_obstacles.at(m_base_loc).o_x - m_obstacles.at(m_base_loc).x_size);
	oob = oob || s.state.at(0) >= (m_obstacles.at(m_base_loc).o_x + m_obstacles.at(m_base_loc).x_size);
	oob = oob || s.state.at(1) <= (m_obstacles.at(m_base_loc).o_y - m_obstacles.at(m_base_loc).y_size);
	oob = oob || s.state.at(1) >= (m_obstacles.at(m_base_loc).o_y + m_obstacles.at(m_base_loc).y_size);

	return oob;
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

// called by CBS::findConflicts
bool CollisionChecker::ObjectObjectCollision(Agent* a1, Agent* a2)
{
	fcl::CollisionRequest request;
	fcl::CollisionResult result;
	fcl::collide(a1->GetFCLObject(), a2->GetFCLObject(), request, result);
	return result.isCollision();
}

// called by Agent::generateSuccessor
bool CollisionChecker::ObjectObjectCollision(Agent* a1, const int& a2_id, const LatticeState& a2_q)
{
	Agent* a2 = m_planner->GetAgent(a2_id);
	a2->UpdatePose(a2_q);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;
	fcl::collide(a1->GetFCLObject(), a2->GetFCLObject(), request, result);
	return result.isCollision();
}

bool CollisionChecker::RobotObjectCollision(
	Agent* a1, const LatticeState& a1_state,
	const LatticeState& robot_state,
	int t, bool process)
{
	bool collision = false;
	if (!CC_2D) {
		collision = collision || m_planner->CheckRobotCollision(a1, robot_state, t, process);
	}
	else
	{
		auto o1_obj = m_planner->GetObject(a1->GetID())->back();
		State o1_loc = {a1_state.state.at(0), a1_state.state.at(1)};
		std::vector<State> o1_rect;
		bool rect_o1 = false;

		// preprocess rectangle once only
		if (o1_obj.Shape() == 0)
		{
			GetRectObjAtPt(o1_loc, o1_obj, o1_rect);
			rect_o1 = true;
		}

		auto robot_2d = m_planner->Get2DRobot(robot_state);

		if (!checkCollisionObjSet(o1_obj, o1_loc, rect_o1, o1_rect, robot_2d))
		{
			if (!CC_3D) {
				collision = true;
			}
			else {
				collision = collision || m_planner->CheckRobotCollision(a1, robot_state, t, process);
			}
		}
	}

	return collision;
}

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

bool CollisionChecker::FCLCollisionMultipleAgents(
	Agent* a1,
	const std::vector<int>& all_agent_ids,
	const std::vector<LatticeState>& all_agent_poses){
	/*
	EECBS stats
	Collision checks between a1 and all the other agents (at the current timestep)
	*/

	m_fcl_mov->unregisterObject(a1->GetFCLObject());

	for(int i = 0; i < all_agent_ids.size(); i++){
		Agent* agent = m_planner->GetAgent(all_agent_ids[i]);
		agent->UpdatePose(all_agent_poses[i]);
		m_fcl_mov->update(agent->GetFCLObject());
	}

	m_fcl_mov->setup();
	fcl::DefaultCollisionData<float> collision_data;
	m_fcl_mov->collide(a1->GetFCLObject(),
		&collision_data, fcl::DefaultCollisionFunction);

	m_fcl_mov->registerObject(a1->GetFCLObject());

	return collision_data.result.isCollision();

}

bool CollisionChecker::checkCollisionObjSet(
	const Object& o1, const State& o1_loc,
	bool rect_o1, const std::vector<State>& o1_rect,
	const std::vector<Object>* a2_objs)
{
	State o2_loc;
	bool rect_o2;
	std::vector<State> o2_rect;

	for (const auto& ao: *a2_objs)
	{
		rect_o2 = false;
		o2_loc = {ao.o_x, ao.o_y};
		if (ao.Shape() == 0)
		{
			GetRectObjAtPt(o2_loc, ao, o2_rect);
			rect_o2 = true;
		}

		if (rect_o1)
		{
			if (rect_o2)
			{
				if (rectRectCollision(o1_rect, o2_rect)) {
					return false;
				}
			}
			else
			{
				if (rectCircCollision(o1_rect, ao, o2_loc)) {
					return false;
				}
			}
		}
		else
		{
			if (rect_o2)
			{
				if (rectCircCollision(o2_rect, o1, o1_loc)) {
					return false;
				}
			}
			else
			{
				if (circCircCollision(o1, o1_loc, ao, o2_loc)) {
					return false;
				}
			}
		}
	}
	// SMPL_WARN("collision! objects ids %d and %d (movable) collide at time %d", o1.id, m_planner->GetObject(p)->id, s.t);
	// std::cout << o1.id << ',' << other_obj->id << ',' << s.t << std::endl;

	return true;
}

bool CollisionChecker::rectRectCollision(
	const std::vector<State>& r1, const std::vector<State>& r2)
{
	return RectanglesIntersect(r1, r2);
}

bool CollisionChecker::rectCircCollision(
	const std::vector<State>& r1, const Object& c1, const State& c1_loc)
{
	return (PointInRectangle(c1_loc, r1) ||
			LineSegCircleIntersect(c1_loc, c1.x_size, r1.at(0), r1.at(1)) ||
			LineSegCircleIntersect(c1_loc, c1.x_size, r1.at(1), r1.at(2)) ||
			LineSegCircleIntersect(c1_loc, c1.x_size, r1.at(2), r1.at(3)) ||
			LineSegCircleIntersect(c1_loc, c1.x_size, r1.at(3), r1.at(0)));
}

bool CollisionChecker::circCircCollision(
	const Object& c1, const State& c1_loc,
	const Object& c2, const State& c2_loc)
{
	double dist = EuclideanDist(c1_loc, c2_loc);
	return (dist < (c1.x_size + c2.x_size));
}

} // namespace clutter
