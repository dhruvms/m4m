#ifndef TYPES_HPP
#define TYPES_HPP

#include <ostream>
#include <vector>

namespace clutter
{

struct Point
{
	int x, y;
	Point() : x(0), y(0) {};
	Point(int _x, int _y) : x(_x), y(_y) {};
	Point(const Point& _p) : x(_p.x), y(_p.y) {};
};

struct Pointf
{
	float x, y;
	Pointf() : x(0.0), y(0.0) {};
	Pointf(float _x, float _y) : x(_x), y(_y) {};
	Pointf(const Pointf& _p) : x(_p.x), y(_p.y) {};
};

struct Object
{
	int id, shape, type;
	float o_x, o_y, o_z;
	float o_roll, o_pitch, o_yaw;
	float x_size, y_size, z_size;
	float mass, mu;
	bool movable, locked;
};

struct State
{
	Point p;
	int t;

	State() {};
	State(int _x, int _y, int _t)
	{
		p.x = _x;
		p.y = _y;
		t = _t;
	};
	State(const Point& _p, int _t) : p(_p), t(_t) {};
};

inline
bool operator==(const State& a, const State& b)
{
	return (
		a.p.x == b.p.x &&
		a.p.y == b.p.y &&
		a.t == b.t
	);
}

inline
std::ostream& operator<<(std::ostream& out, const State s)
{
	return out << '(' << s.p.x << ", " << s.p.y << ", " << s.t << ")";
}

struct Statef
{
	Pointf p;
	int t;

	Statef() {};
	Statef(float _x, float _y, int _t)
	{
		p.x = _x;
		p.y = _y;
		t = _t;
	};
	Statef(const Pointf& _p, int _t) : p(_p), t(_t) {};
};

inline
std::ostream& operator<<(std::ostream& out, const Statef s)
{
	return out << '(' << s.p.x << ", " << s.p.y << ", " << s.t << ")";
}

typedef std::vector<Statef> Trajectory;
typedef std::vector<State*> CLOSED;

class Search
{
public:
	virtual int set_start(int start_id) = 0;
	virtual int set_goal(int goal_id) = 0;
	virtual void set_max_planning_time(double max_planning_time_ms) = 0;
	virtual int get_n_expands() const = 0;
	virtual void reset() = 0;

	virtual int replan(
		std::vector<int>* solution_path, int* solution_cost) = 0;
};

} // namespace clutter

#endif // TYPES_HPP
