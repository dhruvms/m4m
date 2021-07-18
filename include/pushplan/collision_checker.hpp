#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <pushplan/agent.hpp>

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
	bool ImmovableCollision(const State& s, const Object& o, const int& priority);
	bool IsStateValid(const State& s, const Object& o, const int& priority);
	bool OOICollision(const State& s, const Object& o);

	float BoundaryDistance(const Pointf& p);

	Pointf GetGoalState(const Object* o);

	float GetBaseWidth() { return std::fabs(m_base.at(0).x - m_base.at(1).x); };
	float GetBaseLength() { return std::fabs(m_base.at(0).y - m_base.at(3).y); };

	int NumObstacles() { return (int)m_obstacles.size(); };
	const std::vector<Object>* GetObstacles() { return &m_obstacles; };

private:
	Planner* m_planner = nullptr;

	std::vector<Object> m_obstacles;
	size_t m_base_loc;
	std::vector<Pointf> m_base;
	std::vector<Trajectory> m_trajs;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	int m_phase;

	bool obstacleCollision(
		const State& s, const Object& o,
		const Pointf& obs_loc, const Object& obs);
	bool baseCollision(
		const State& s, const Object& o, const int& priority);
};

} // namespace clutter


#endif // COLLISION_CHECKER_HPP
