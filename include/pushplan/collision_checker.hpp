#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <pushplan/types.hpp>

#include <vector>
#include <random>

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
	void SetPhase(int phase) { m_phase = phase; };

	void UpdateTraj(const int& priority, const Trajectory& traj);

	bool ImmovableCollision(const LatticeState& s, const Object& o, const int& priority);
	bool ImmovableCollision(const std::vector<Object>& objs, const int& priority);

	bool IsStateValid(const LatticeState& s, const Object& o, const int& priority);

	bool OOICollision(const Object& o);

	double BoundaryDistance(const State& p);

	State GetGoalState(const Object* o);

	double GetBaseWidth() { return std::fabs(m_base.at(0).at(0) - m_base.at(1).at(0)); };
	double GetBaseLength() { return std::fabs(m_base.at(0).at(1) - m_base.at(3).at(1)); };

	int NumObstacles() { return (int)m_obstacles.size(); };
	const std::vector<Object>* GetObstacles() { return &m_obstacles; };

private:
	Planner* m_planner = nullptr;

	std::vector<Object> m_obstacles;
	size_t m_base_loc;
	std::vector<State> m_base;
	std::vector<Trajectory> m_trajs;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	int m_phase;

	bool immovableCollision(const Object& o, const int& priority);
	bool obstacleCollision(const Object& o, const Object& obs);
	bool baseCollision(const Object& o, const int& priority);
};

} // namespace clutter


#endif // COLLISION_CHECKER_HPP
