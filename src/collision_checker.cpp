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

void CollisionChecker::InitMovableSet(std::vector<Agent>* agents)
{
	for (size_t i = 0; i != agents->size(); ++i) {
		m_fcl_mov->registerObject(agents->at(i).GetFCLObject());
	}
}

void CollisionChecker::AddToMovableSet(Agent* agent)
{
	m_fcl_mov->registerObject(agent->GetFCLObject());
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

// IsStateValid should only be called for priority > 1
bool CollisionChecker::IsStateValid(
	const LatticeState& s, const Object& o1, const int& priority)
{
	State o1_loc = {s.state.at(0), s.state.at(1)};
	std::vector<State> o1_rect;
	bool rect_o1 = false;

	// preprocess rectangle once only
	if (o1.Shape() == 0)
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
				// (o2.o_x, o2.o_y) are consistent with
				// s2.state
				auto a2_objs = m_planner->GetObject(s2, p);
				if (!checkCollisionObjSet(o1, o1_loc, rect_o1, o1_rect, a2_objs)) {
					return false;
				}
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
	if (o.Shape() == 0)
	{
		GetRectObjAtPt(o_loc, o, o_rect);
		rect_o = true;
	}

	auto ooi = m_planner->GetObject(*(m_planner->GetOOIState()), 0)->back();
	State ooi_loc = {ooi.o_x, ooi.o_y};

	if (rect_o)	{
		return PointInRectangle(ooi_loc, o_rect);
	}
	else {
		return EuclideanDist(o_loc, ooi_loc) < o.x_size;
	}

	return false;
}

bool CollisionChecker::UpdateConflicts(
	const LatticeState& s, const Object& o1, const int& priority)
{
	State o1_loc = {s.state.at(0), s.state.at(1)};
	std::vector<State> o1_rect;
	bool rect_o1 = false;

	// preprocess rectangle once only
	if (o1.Shape() == 0)
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
				// (o2.o_x, o2.o_y) are consistent with
				// s2.state
				auto a2_objs = m_planner->GetObject(s2, p);
				if (!checkCollisionObjSet(o1, o1_loc, rect_o1, o1_rect, a2_objs))
				{
					int id1 = o1.id, id2 = a2_objs->back().id;
					int priority2 = p;
					if (p == 0 || p == 1) {
						id2 = 100;
						priority2 = 0;
					}
					updateConflicts(id1, priority, id2, priority2, s.t);
				}
			}
		}
	}

	return true;
}

auto CollisionChecker::GetConflictsOf(int pusher) const ->
std::unordered_map<std::pair<int, int>, int, std::PairHash>
{
	auto pushed = m_conflicts;
	pushed.clear();
	for (const auto& c: m_conflicts)
	{
		if (c.first.second == pusher) {
			pushed[c.first] = c.second;
		}
	}

	return pushed;
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

	gmin.at(0) = OutsideXMin();
	gmax.at(0) = OutsideXMax();

	gmin.at(1) = OutsideYMin();
	gmax.at(1) = OutsideYMax();

	if (o->Shape() == 0) // rectangle
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
	else if (o->Shape() == 2) // circle
	{
		do
		{
			g.at(0) = (m_distD(m_rng) * (gmax.at(0) - gmin.at(0))) + gmin.at(0);
			g.at(1) = (m_distD(m_rng) * (gmax.at(1) - gmin.at(1))) + gmin.at(1);
		}
		while (LineSegCircleIntersect(g, o->x_size, m_base.at(0), m_base.back()));
	}
	else
	{
		SMPL_ERROR("Invalid object type!");
		g.at(0) = -99;
		g.at(1) = -99;
	}

	return g;
}

bool CollisionChecker::updateConflicts(
	int id1, int p1,
	int id2, int p2, int t)
{
	auto key = std::make_pair(id1, id2);
	auto search = m_conflicts.find(key);
	if (search != m_conflicts.end())
	{
		if (search->second > t) {
			m_conflicts[key] = t;
		}
	}
	else {
		m_conflicts.emplace(key, t);
	}
}

void CollisionChecker::cleanupChildren(std::vector<int>& check)
{
	for (const auto& c: m_conflicts)
	{
		auto loc = std::find(check.begin(), check.end(), c.first.second);
		if (loc != check.end())
		{
			loc = std::find(check.begin(), check.end(), c.first.first);
			if (loc == check.end()) {
				check.push_back(c.first.first);
			}
		}
	}
}

} // namespace clutter
