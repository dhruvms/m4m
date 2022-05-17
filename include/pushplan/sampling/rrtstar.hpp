#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include <pushplan/sampling/rrt.hpp>

namespace clutter {
namespace sampling {

class RRTStar : public RRT
{
public:
	RRTStar();
	RRTStar(int samples, int steps, double gbias, double gthresh, double timeout);

	bool Solve() override;
	void SetRewireFactor(double rewire_factor) { m_rewire_factor = rewire_factor; };

private:
	double m_rewire_factor, m_dofs, m_radius;

	bool extend(
		const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result) override;
	bool nearestNeighbours(const smpl::RobotState& state, std::vector<Vertex_t>& nns);
	void removeEdge(const Vertex_t& source, const Vertex_t& dest);
};

} // namespace sampling
} // namespace clutter

#endif // RRTSTAR_HPP
