#ifndef RRT_HPP
#define RRT_HPP

#include <pushplan/sampling/sampling_planner.hpp>

namespace clutter {
namespace sampling {

class RRT : public SamplingPlanner
{
public:
	RRT();
	RRT(int samples, int steps, double gbias, double gthresh, double timeout);

	bool Solve() override;

private:
	int m_N, m_steps;
	double m_gbias, m_gthresh, m_timeout;

	bool extend(
		const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result) override;
	bool selectVertex(const smpl::RobotState& qrand, Vertex_t& nearest) override;
	bool steer(
		const smpl::RobotState& qrand,
		Node* xnear,
		smpl::RobotState& qnew,
		comms::ObjectsPoses& qnew_objs,
		std::uint32_t& result) override;
};

} // namespace sampling
} // namespace clutter

#endif // RRT_HPP
