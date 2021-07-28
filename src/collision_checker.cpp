#include <pushplan/collision_checker.hpp>
#include <pushplan/planner.hpp>
#include <pushplan/geometry.hpp>

#include <smpl/console/console.h>

#include <iostream>

namespace clutter
{

CollisionChecker::CollisionChecker(Planner* planner, const std::vector<Object>& obstacles)
:
m_planner(planner),
m_obstacles(obstacles),
m_rng(m_dev())
{
	for (size_t i = 0; i != m_obstacles.size(); ++i)
	{
		if (m_obstacles.at(i).id == 1) {
			m_base_loc = i;
			break;
		}
	}
	MakeObjectRectangle(m_obstacles.at(m_base_loc), m_base);

	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
}

void CollisionChecker::UpdateTraj(const int& priority, const Trajectory& traj)
{
	if (int(m_trajs.size()) <= priority) {
		m_trajs.resize(priority + 1, {});
	}

	m_trajs.at(priority) = traj;
}

bool CollisionChecker::ImmovableCollision(const LatticeState& s, const Object& o, const int& priority)
{
	if (!FRIDGE)
	{
		SMPL_WARN("Do not handle non-fridge scenes. Return collision = true.");
		return true;
	}

	Object o_temp = o;
	o_temp.o_x = s.state.at(0);
	o_temp.o_y = s.state.at(1);

	return immovableCollision(o_temp, priority);
}

bool CollisionChecker::ImmovableCollision(const std::vector<Object>& objs, const int& priority)
{
	if (!FRIDGE)
	{
		SMPL_WARN("Do not handle non-fridge scenes. Return collision = true.");
		return true;
	}

	// (o.o_x, o.o_y) are already where o is
	for (const auto& o: objs)
	{
		if (immovableCollision(o, priority)) {
			return true;
		}
	}

	return false;
}

// IsStateValid should only be called for priority > 1
bool CollisionChecker::IsStateValid(
	const LatticeState& s, const Object& o, const int& priority)
{
	Object o_temp = o;
	o_temp.o_x = s.state.at(0);
	o_temp.o_y = s.state.at(1);

	for (int p = 0; p < priority; ++p)
	{
		for (const auto& s2: m_trajs.at(p))
		{
			if (s.t == s2.t)
			{
				auto other_objs = m_planner->GetObject(s2, p);
				// (other_o.o_x, other_o.o_y) are consistent with
				// s2.state
				for (const auto& other_o: *other_objs)
				{
					if (obstacleCollision(o_temp, other_o)) {
						return false;
					}
				}
				// SMPL_WARN("collision! objects ids %d and %d (movable) collide at time %d", o.id, m_planner->GetObject(p)->id, s.t);
				// std::cout << o.id << ',' << other_obj->id << ',' << s.t << std::endl;
			}
		}
	}
	return true;
}

// OOICollision should only be called for the EE rectangle object at some state
bool CollisionChecker::OOICollision(const Object& o)
{
	auto ooi_obj = m_planner->GetObject(*(m_planner->GetOOIState()), 0);
	return obstacleCollision(o, ooi_obj->back());
}

double CollisionChecker::BoundaryDistance(const State& p)
{
	double d = PtDistFromLine(p, m_base.at(0), m_base.at(1));
	d = std::min(d, PtDistFromLine(p, m_base.at(1), m_base.at(2)));
	d = std::min(d, PtDistFromLine(p, m_base.at(2), m_base.at(3)));
	d = std::min(d, PtDistFromLine(p, m_base.at(3), m_base.at(0)));
	return d;
}

State CollisionChecker::GetGoalState(const Object* o)
{
	State g(2, 0.0);
	State gmin(2, 0.0), gmax(2, 0.0);

	gmin.at(0) = m_base.at(0).at(0) - m_obstacles.at(m_base_loc).x_size;
	gmax.at(0) = m_base.at(0).at(0);

	gmin.at(1) = m_base.at(0).at(1) + (m_obstacles.at(m_base_loc).y_size/3);
	gmax.at(1) = m_base.back().at(1) - (m_obstacles.at(m_base_loc).y_size/3);

	if (o->shape == 0) // rectangle
	{
		std::vector<State> o_goal;
		do
		{
			g.at(0) = (m_distD(m_rng) * (gmax.at(0) - gmin.at(0))) + gmin.at(0);
			g.at(1) = (m_distD(m_rng) * (gmax.at(1) - gmin.at(1))) + gmin.at(1);
			GetRectObjAtPt(g, *o, o_goal);
		}
		while (RectanglesIntersect(o_goal, m_base));
	}
	else if (o->shape == 2) // circle
	{
		do
		{
			g.at(0) = (m_distD(m_rng) * (gmax.at(0) - gmin.at(0))) + gmin.at(0);
			g.at(1) = (m_distD(m_rng) * (gmax.at(1) - gmin.at(1))) + gmin.at(1);
		}
		while (LineSegCircleIntersect(g, (double)o->x_size, m_base.at(0), m_base.back()));
	}
	else
	{
		SMPL_ERROR("Invalid object type!");
		g.at(0) = -99;
		g.at(1) = -99;
	}

	return g;
}

bool CollisionChecker::immovableCollision(const Object& o, const int& priority)
{
	if (baseCollision(o, priority)) {
		// SMPL_WARN("collision with base!");
		return true;
	}

	for (const auto& obstacle: m_obstacles)
	{
		if (obstacle.id <= 5) { // shelf
			continue;
		}

		if (obstacleCollision(o, obstacle)) {
			// SMPL_WARN("collision! objects ids %d and %d (immovable) collide", o.id, obstacle.id);
			return true;
		}
	}
	return false;
}

bool CollisionChecker::obstacleCollision(
	const Object& o, const Object& obs)
{
	State o_loc = {o.o_x, o.o_y};
	State obs_loc = {obs.o_x, obs.o_y};

	if (o.shape == 0) // object is rectangle
	{
		std::vector<State> o_rect;
		GetRectObjAtPt(o_loc, o, o_rect);

		if (obs.shape == 0) // obstacle is rectangle
		{
			std::vector<State> obs_rect;
			GetRectObjAtPt(obs_loc, obs, obs_rect);

			return RectanglesIntersect(o_rect, obs_rect);
		}
		else if (obs.shape == 2) // obstacle is circle
		{
			return (PointInRectangle(obs_loc, o_rect) ||
					LineSegCircleIntersect(obs_loc, (double)obs.x_size, o_rect.at(0), o_rect.at(1)) ||
					LineSegCircleIntersect(obs_loc, (double)obs.x_size, o_rect.at(1), o_rect.at(2)) ||
					LineSegCircleIntersect(obs_loc, (double)obs.x_size, o_rect.at(2), o_rect.at(3)) ||
					LineSegCircleIntersect(obs_loc, (double)obs.x_size, o_rect.at(3), o_rect.at(0)));
		}
	}
	else if (o.shape == 2) // object is circle
	{
		if (obs.shape == 0) // obstacle is rectangle
		{
			std::vector<State> obs_rect;
			GetRectObjAtPt(obs_loc, obs, obs_rect);

			return (PointInRectangle(o_loc, obs_rect) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, obs_rect.at(0), obs_rect.at(1)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, obs_rect.at(1), obs_rect.at(2)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, obs_rect.at(2), obs_rect.at(3)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, obs_rect.at(3), obs_rect.at(0)));

		}
		else if (obs.shape == 2) // obstacle is circle
		{
			double dist = EuclideanDist(o_loc, obs_loc);
			return (dist < (o.x_size + obs.x_size));
		}
	}

	SMPL_WARN("Cannot handle object types. Return collision = true!");
	return true; // default to true
}

bool CollisionChecker::baseCollision(
	const Object& o, const int& priority)
{
	State o_loc = {o.o_x, o.o_y};

	if (o.shape == 0) // object is rectangle
	{
		std::vector<State> o_rect;
		GetRectObjAtPt(o_loc, o, o_rect);
		if (priority > 1) // object is movable
		{
			// no part of the object can be outside the sides or back of shelf
			for (const auto& p: o_rect)
			{
				if (p.at(0) > m_base.at(1).at(0) ||
					p.at(1) < m_base.at(0).at(1) ||
					p.at(1) > m_base.at(2).at(1))
				{
					return true;
				}
			}

			// the centroid of the object rectangle should be inside shelf
			if (!PointInRectangle(o_loc, m_base)) {
				return true;
			}
			return false;
		}
		else // object is robot or being extracted
		{
			// no part of the OOI can be outside the sides or back of shelf
			for (const auto& p: o_rect)
			{
				if (p.at(0) > m_base.at(1).at(0) ||
					p.at(1) < m_base.at(0).at(1) ||
					p.at(1) > m_base.at(2).at(1))
				{
					return true;
				}
			}
			return false;
		}
	}
	else if (o.shape == 2) // object is circle
	{
		if (priority > 1) // object is movable
		{
			// the centre of the object circle must be inside shelf
			// no part of the object can be outside the sides or back of shelf
			return (!PointInRectangle(o_loc, m_base) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(0), m_base.at(1)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(1), m_base.at(2)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(2), m_base.at(3)));
		}
		else // object is robot or being extracted
		{
			// no part of the OOI can be outside the sides or back of shelf
			return (LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(0), m_base.at(1)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(1), m_base.at(2)) ||
					LineSegCircleIntersect(o_loc, (double)o.x_size, m_base.at(2), m_base.at(3)));

		}
	}
}

} // namespace clutter
