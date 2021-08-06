#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <pushplan/types.hpp>

#include <vector>
#include <random>
#include <unordered_map>

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

private:
	Planner* m_planner = nullptr;

	std::vector<Object> m_obstacles;
	std::unordered_map<int, std::vector<State> > m_obs_rects;
	size_t m_base_loc;
	std::vector<State> m_base;
	std::vector<Trajectory> m_trajs;

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
};

} // namespace clutter


#endif // COLLISION_CHECKER_HPP
