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

bool CollisionChecker::ImmovableCollision(const State& s, const Object& o, const int& priority)
{
	if (!FRIDGE)
	{
		SMPL_WARN("Do not handle non-fridge scenes. Return collision = true.");
		return true;
	}

	if (baseCollision(s, o, priority)) {
		// SMPL_WARN("collision with base!");
		return true;
	}

	for (const auto& obstacle: m_obstacles)
	{
		if (obstacle.id <= 5) { // shelf
			continue;
		}

		Pointf obs_loc(obstacle.o_x, obstacle.o_y);
		if (obstacleCollision(s, o, obs_loc, obstacle)) {
			// SMPL_WARN("collision! objects ids %d and %d (immovable) collide", o.id, obstacle.id);
			return true;
		}
	}
	return false;
}

bool CollisionChecker::IsStateValid(
	const State& s, const Object& o, const int& priority)
{
	for (int p = 0; p < priority; ++p)
	{
		for (const auto& s2: m_trajs.at(p))
		{
			if (s.t == s2.t && obstacleCollision(s, o, s2.p, *(m_planner->GetObject(p)))) {
				// SMPL_WARN("collision! objects ids %d and %d (movable) collide", o.id, m_planner->GetObject(p)->id);

				return false;
			}
		}
	}
	return true;
}

Pointf CollisionChecker::GetGoalState(const Object* o)
{
	Pointf g;
	Pointf gmin, gmax;

	gmin.x = m_base.at(0).x - m_obstacles.at(m_base_loc).x_size;
	gmax.x = m_base.at(0).x;

	gmin.y = m_base.at(0).y + (m_obstacles.at(m_base_loc).y_size/3);
	gmax.y = m_base.back().y - (m_obstacles.at(m_base_loc).y_size/3);

	switch (o->shape)
	{
		case 0: { // rectangle
			std::vector<Pointf> o_goal;
			do
			{
				g.x = (m_distD(m_rng) * (gmax.x - gmin.x)) + gmin.x;
				g.y = (m_distD(m_rng) * (gmax.y - gmin.y)) + gmin.y;
				GetRectObjAtPt(g, *o, o_goal);
			} while (RectanglesIntersect(o_goal, m_base));
			break;
		}
		case 2: { // circle
			do
			{
				g.x = (m_distD(m_rng) * (gmax.x - gmin.x)) + gmin.x;
				g.y = (m_distD(m_rng) * (gmax.y - gmin.y)) + gmin.y;
			} while (LineSegCircleIntersect(g, o->x_size, m_base.at(0), m_base.back()));
			break;
		}
		default: {
			SMPL_ERROR("Invalid object type!");
			g.x = -99;
			g.y = -99;
		}
	}

	return g;
}

bool CollisionChecker::obstacleCollision(
	const State& s, const Object& o,
	const Pointf& obs_loc, const Object& obs)
{
	Pointf o_loc;
	DiscToCont(s.p, o_loc);

	if (o.shape == 0) // object is rectangle
	{
		std::vector<Pointf> o_rect;
		GetRectObjAtPt(o_loc, o, o_rect);

		if (obs.shape == 0) // obstacle is rectangle
		{
			std::vector<Pointf> obs_rect;
			GetRectObjAtPt(obs_loc, obs, obs_rect);

			return RectanglesIntersect(o_rect, obs_rect);
		}
		else if (obs.shape == 2) // obstacle is circle
		{
			return (PointInRectangle(obs_loc, o_rect) ||
					LineSegCircleIntersect(obs_loc, obs.x_size, o_rect.at(0), o_rect.at(1)) ||
					LineSegCircleIntersect(obs_loc, obs.x_size, o_rect.at(1), o_rect.at(2)) ||
					LineSegCircleIntersect(obs_loc, obs.x_size, o_rect.at(2), o_rect.at(3)) ||
					LineSegCircleIntersect(obs_loc, obs.x_size, o_rect.at(3), o_rect.at(0)));
		}
	}
	else if (o.shape == 2) // object is circle
	{
		if (obs.shape == 0) // obstacle is rectangle
		{
			std::vector<Pointf> obs_rect;
			GetRectObjAtPt(obs_loc, obs, obs_rect);

			return (PointInRectangle(o_loc, obs_rect) ||
					LineSegCircleIntersect(o_loc, o.x_size, obs_rect.at(0), obs_rect.at(1)) ||
					LineSegCircleIntersect(o_loc, o.x_size, obs_rect.at(1), obs_rect.at(2)) ||
					LineSegCircleIntersect(o_loc, o.x_size, obs_rect.at(2), obs_rect.at(3)) ||
					LineSegCircleIntersect(o_loc, o.x_size, obs_rect.at(3), obs_rect.at(0)));

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
	const State& s, const Object& o, const int& priority)
{
	Pointf o_loc;
	DiscToCont(s.p, o_loc);

	if (o.shape == 0) // object is rectangle
	{
		std::vector<Pointf> o_rect;
		GetRectObjAtPt(o_loc, o, o_rect);
		if (priority > 0) // object is movable
		{
			// no part of the object can be outside the sides or back of shelf
			for (const auto& p: o_rect)
			{
				if (p.x > m_base.at(1).x ||
					p.y < m_base.at(0).y ||
					p.y > m_base.at(2).y)
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
		else // object is being extracted
		{
			// no part of the OOI can be outside the sides or back of shelf
			for (const auto& p: o_rect)
			{
				if (p.x > m_base.at(1).x ||
					p.y < m_base.at(0).y ||
					p.y > m_base.at(2).y)
				{
					return true;
				}
			}
			return false;
		}
	}
	else if (o.shape == 2) // object is circle
	{
		if (priority > 0) // object is movable
		{
			// the centre of the object circle must be inside shelf
			// no part of the object can be outside the sides or back of shelf
			return (!PointInRectangle(o_loc, m_base) ||
					LineSegCircleIntersect(o_loc, o.x_size, m_base.at(0), m_base.at(1)) ||
					LineSegCircleIntersect(o_loc, o.x_size, m_base.at(1), m_base.at(2)) ||
					LineSegCircleIntersect(o_loc, o.x_size, m_base.at(2), m_base.at(3)));
		}
		else // object is being extracted
		{
			// no part of the OOI can be outside the sides or back of shelf
			return (LineSegCircleIntersect(o_loc, o.x_size, m_base.at(0), m_base.at(1)) ||
					LineSegCircleIntersect(o_loc, o.x_size, m_base.at(1), m_base.at(2)) ||
					LineSegCircleIntersect(o_loc, o.x_size, m_base.at(2), m_base.at(3)));

		}
	}
}

} // namespace clutter
