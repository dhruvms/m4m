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
	// preprocess immovable obstacles
	for (size_t i = 0; i != m_obstacles.size(); ++i)
	{
		if (m_obstacles.at(i).id == 1)
		{
			m_base_loc = i;
			MakeObjectRectangle(m_obstacles.at(m_base_loc), m_base);
		}
		else if (m_obstacles.at(i).id <= 5) { // shelf
			continue;
		}
		else
		{
			State obs_loc = {m_obstacles.at(i).o_x, m_obstacles.at(i).o_y};
			std::vector<State> obs_rect;
			GetRectObjAtPt(obs_loc, m_obstacles.at(i), obs_rect);
			m_obs_rects[m_obstacles.at(i).id] = obs_rect;
		}
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
	const LatticeState& s, const Object& o1, const int& priority)
{
	State o1_loc = {s.state.at(0), s.state.at(1)}, o2_loc;
	std::vector<State> o1_rect, o2_rect;
	bool rect_o1 = false, rect_o2 = false;

	// preprocess rectangle once only
	if (o1.shape == 0)
	{
		GetRectObjAtPt(o1_loc, o1, o1_rect);
		rect_o1 = true;
	}

	for (int p = 0; p < priority; ++p)
	{
		for (const auto& s2: m_trajs.at(p))
		{
			if (s.t == s2.t)
			{
				auto o2_objs = m_planner->GetObject(s2, p);
				// (o2.o_x, o2.o_y) are consistent with
				// s2.state
				for (const auto& o2: *o2_objs)
				{
					o2_loc = {o2.o_x, o2.o_y};
					if (o2.shape == 0)
					{
						GetRectObjAtPt(o2_loc, o2, o2_rect);
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
							if (rectCircCollision(o1_rect, o2, o2_loc)) {
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
							if (circCircCollision(o1, o1_loc, o2, o2_loc)) {
								return false;
							}
						}
					}
				}
				// SMPL_WARN("collision! objects ids %d and %d (movable) collide at time %d", o1.id, m_planner->GetObject(p)->id, s.t);
				// std::cout << o1.id << ',' << other_obj->id << ',' << s.t << std::endl;
			}
		}
	}
	return true;
}

// OOICollision should only be called for the EE rectangle object at some state
bool CollisionChecker::OOICollision(const Object& o)
{
	State o_loc = {o.o_x, o.o_y};
	std::vector<State> o_rect, ooi_rect;
	bool rect_o = false, rect_ooi = false;

	// preprocess rectangle once only
	if (o.shape == 0)
	{
		GetRectObjAtPt(o_loc, o, o_rect);
		rect_o = true;
	}

	auto ooi = m_planner->GetObject(*(m_planner->GetOOIState()), 0)->back();
	State ooi_loc = {ooi.o_x, ooi.o_y};
	if (ooi.shape == 0)
	{
		GetRectObjAtPt(ooi_loc, ooi, ooi_rect);
		rect_ooi = true;
	}

	if (rect_o)
	{
		if (rect_ooi)
		{
			if (rectRectCollision(o_rect, ooi_rect)) {
				return true;
			}
		}
		else
		{
			if (rectCircCollision(o_rect, ooi, ooi_loc)) {
				return true;
			}
		}
	}
	else
	{
		if (rect_ooi)
		{
			if (rectCircCollision(ooi_rect, o, o_loc)) {
				return true;
			}
		}
		else
		{
			if (circCircCollision(o, o_loc, ooi, ooi_loc)) {
				return true;
			}
		}
	}

	return false;
}

double CollisionChecker::BoundaryDistance(const State& p)
{
	double d = PtDistFromLine(p, m_base.at(0), m_base.at(1));
	d = std::min(d, PtDistFromLine(p, m_base.at(1), m_base.at(2)));
	d = std::min(d, PtDistFromLine(p, m_base.at(2), m_base.at(3)));
	d = std::min(d, PtDistFromLine(p, m_base.at(3), m_base.at(0)));
	return d;
}

State CollisionChecker::GetRandomStateOutside(const Object* o)
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
	State o_loc = {o.o_x, o.o_y};
	std::vector<State> o_rect;
	// preprocess rectangle once only
	bool rect = false;
	if (o.shape == 0)
	{
		GetRectObjAtPt(o_loc, o, o_rect);
		rect = true;
	}

	if (rect)
	{
		if (rectCollisionBase(o_loc, o_rect, priority)) {
			// SMPL_WARN("collision with base!");
			return true;
		}

		for (const auto& obstacle: m_obstacles)
		{
			if (obstacle.id <= 5) { // shelf
				continue;
			}

			if (obstacle.shape == 0) // rectangle
			{
				if (rectRectCollision(o_rect, m_obs_rects[obstacle.id])) {
					return true;
				}
			}
			else if (obstacle.shape == 2) // circle
			{
				State obs_loc = {obstacle.o_x, obstacle.o_y};
				if (rectCircCollision(o_rect, obstacle, obs_loc)) {
					return true;
				}
			}
		}
	}
	else
	{
		if (circCollisionBase(o_loc, o, priority)) {
			// SMPL_WARN("collision with base!");
			return true;
		}

		for (const auto& obstacle: m_obstacles)
		{
			if (obstacle.id <= 5) { // shelf
				continue;
			}

			if (obstacle.shape == 0) // rectangle
			{
				if (rectCircCollision(m_obs_rects[obstacle.id], o, o_loc)) {
					return true;
				}
			}
			else if (obstacle.shape == 2) // circle
			{
				State obs_loc = {obstacle.o_x, obstacle.o_y};
				if (circCircCollision(o, o_loc, obstacle, obs_loc)) {
					return true;
				}
			}
		}
	}

	return false;
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
			LineSegCircleIntersect(c1_loc, (double)c1.x_size, r1.at(0), r1.at(1)) ||
			LineSegCircleIntersect(c1_loc, (double)c1.x_size, r1.at(1), r1.at(2)) ||
			LineSegCircleIntersect(c1_loc, (double)c1.x_size, r1.at(2), r1.at(3)) ||
			LineSegCircleIntersect(c1_loc, (double)c1.x_size, r1.at(3), r1.at(0)));
}

bool CollisionChecker::circCircCollision(
	const Object& c1, const State& c1_loc,
	const Object& c2, const State& c2_loc)
{
	double dist = EuclideanDist(c1_loc, c2_loc);
	return (dist < (c1.x_size + c2.x_size));
}

bool CollisionChecker::rectCollisionBase(
	const State& o_loc, const std::vector<State>& o_rect, const int& priority)
{
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

bool CollisionChecker::circCollisionBase(
	const State& o_loc, const Object& o, const int& priority)
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

} // namespace clutter
