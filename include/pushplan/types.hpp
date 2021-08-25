#ifndef TYPES_HPP
#define TYPES_HPP

#include <ostream>
#include <vector>

namespace clutter
{

typedef std::vector<int> Coord;
typedef std::vector<double> State;

struct LatticeState
{
	Coord coord;
	State state;
	int t;
};

typedef std::vector<LatticeState> Trajectory;
typedef std::vector<LatticeState*> STATES;

inline
bool operator==(const LatticeState& a, const LatticeState& b)
{
	return (
		a.coord == b.coord &&
		a.t == b.t
	);
}

} // namespace clutter

namespace std {

template <>
struct hash<clutter::LatticeState>
{
	typedef clutter::LatticeState argument_type;
	typedef std::size_t result_type;
	result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace clutter
{

struct Object
{
	int id, shape, type;
	double o_x, o_y, o_z;
	double o_roll, o_pitch, o_yaw;
	double x_size, y_size, z_size;
	double mass, mu;
	bool movable, locked;
};

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

struct RobotModelConfig
{
	std::string group_name;
	std::vector<std::string> planning_joints;
	std::string kinematics_frame;
	std::string chain_tip_link;
	std::vector<std::string> push_links;
};

struct PlannerConfig
{
	std::string discretization;
	std::string mprim_filename;
	bool use_xyz_snap_mprim;
	bool use_rpy_snap_mprim;
	bool use_xyzrpy_snap_mprim;
	bool use_short_dist_mprims;
	double xyz_snap_dist_thresh;
	double rpy_snap_dist_thresh;
	double xyzrpy_snap_dist_thresh;
	double short_dist_mprims_thresh;
};

} // namespace clutter

#endif // TYPES_HPP
