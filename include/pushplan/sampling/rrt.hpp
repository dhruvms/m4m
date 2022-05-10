#ifndef RRT_HPP
#define RRT_HPP

#include <pushplan/sampling/sampling_planner.hpp>

namespace clutter {
namespace sampling {

class RRT : public SamplingPlanner
{
public:
	RRT();
	RRT(int samples, double gbias, double gthresh, double eps, double timeout);

	bool Solve() override;
	virtual ExtractPath() = 0;

private:
	int m_N;
	double m_gbias, m_gthresh, m_eps;

	std::uint32_t extend(const smpl::RobotState& sample) override;
	Node* selectVertex(const smpl::RobotState& qrand) override;
	bool steer(
		const smpl::RobotState& qrand,
		Node* xnear,
		smpl::RobotState& qnew) override;
};

} // namespace sampling
} // namespace clutter

#endif // RRT_HPP
