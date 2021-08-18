#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <pushplan/types.hpp>

#include <boost/functional/hash.hpp>

#include <vector>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iostream>

namespace std
{

class PairHash
{
public:
	// id is returned as hash function
	size_t operator()(const pair<int, int>& s) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, s.first);
		boost::hash_combine(seed, s.second);
		return seed;
	}
};

inline
bool operator==(const pair<int, int>& a, const pair<int, int>& b)
{
	return (a.first == b.first && a.second == b.second) ||
				(a.first == b.second && a.second == b.first);
}

inline
ostream& operator<<(ostream& os, unordered_set<pair<int, int>, PairHash> const& s)
{
	os << "[" << s.size() << "] { ";
	for (pair<int, int> i : s)
		os << "(" << i.first << ", " << i.second << ") ";
	return os << "}\n";
}

} // namespace std

namespace clutter
{

class Planner;

class CollisionChecker
{
public:
	CollisionChecker(Planner* planner, const std::vector<Object>& obstacles);

	void AddObstacle(const Object& o) {
		m_obstacles.push_back(o);
	};

	void UpdateTraj(const int& priority, const Trajectory& traj);

	bool ImmovableCollision(const LatticeState& s, const Object& o, const int& priority);
	bool ImmovableCollision(const std::vector<Object>& objs, const int& priority);

	bool IsStateValid(const LatticeState& s, const Object& o, const int& priority);

	bool OOICollision(const Object& o);

	double BoundaryDistance(const State& p);
	double GetMinX() { return m_base.at(0).at(0); };

	State GetRandomStateOutside(const Object* o);

	double GetBaseWidth() { return std::fabs(m_base.at(0).at(0) - m_base.at(1).at(0)); };
	double GetBaseLength() { return std::fabs(m_base.at(0).at(1) - m_base.at(3).at(1)); };

	double GetTableHeight() { return m_obstacles.at(0).o_z + m_obstacles.at(0).z_size; };
	double OutsideXMin() { return m_base.at(0).at(0) - m_obstacles.at(m_base_loc).x_size; };
	double OutsideYMin() { return m_base.at(0).at(1) + (m_obstacles.at(m_base_loc).y_size/3); };
	double OutsideXMax() { return m_base.at(0).at(0); };
	double OutsideYMax() { return m_base.back().at(1) - (m_obstacles.at(m_base_loc).y_size/3); };

	int NumObstacles() { return (int)m_obstacles.size(); };
	const std::vector<Object>* GetObstacles() { return &m_obstacles; };

	void PrintConflicts() { std::cout << m_conflicts << std::endl; }
	auto GetConflicts() const -> const std::unordered_set<std::pair<int, int>, std::PairHash>* {
		return &m_conflicts;
	};

private:
	Planner* m_planner = nullptr;

	std::vector<Object> m_obstacles;
	std::unordered_map<int, std::vector<State> > m_obs_rects;
	size_t m_base_loc;
	std::vector<State> m_base;
	std::vector<Trajectory> m_trajs;
	std::unordered_set<std::pair<int, int>, std::PairHash> m_conflicts;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	bool immovableCollision(const Object& o, const int& priority);

	bool rectRectCollision(
		const std::vector<State>& r1, const std::vector<State>& r2);
	bool rectCircCollision(
		const std::vector<State>& r1,
		const Object& c1, const State& c1_loc);
	bool circCircCollision(
		const Object& c1, const State& c1_loc,
		const Object& c2, const State& c2_loc);

	bool rectCollisionBase(
		const State& o_loc, const std::vector<State>& o_rect,
		const int& priority);
	bool circCollisionBase(
		const State& o_loc, const Object& o,
		const int& priority);

	bool updateConflicts(
		const Object& o1, int p1,
		const Object& o2, int p2);
};

} // namespace clutter


#endif // COLLISION_CHECKER_HPP
