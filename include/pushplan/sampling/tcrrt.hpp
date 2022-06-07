#ifndef TCRRT_HPP
#define TCRRT_HPP

#include <pushplan/sampling/rrt.hpp>

namespace clutter {
namespace sampling {

class TCRRT : public RRT
{
public:
	TCRRT(
		int mode=0, int I=1000, int J=100);
	TCRRT(
		int samples, int steps, double gbias, double gthresh, double timeout,
		int mode=0, int I=1000, int J=100);

	bool Solve() override;

	void SetMode(int mode) override { m_mode = mode; };
	void SetConstraintHeight(double z) override {
		m_T_0_t.translation().z() = z;
	};

private:
	int m_mode, m_I, m_J;
	double m_dmax;
	Eigen::Affine3d m_T_0_t;

	bool extend(
		const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result) override;

	bool rgd_extend(
		smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
		Vertex_t& new_v, std::uint32_t& result);
	bool ts_extend(
		smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
		Vertex_t& new_v, std::uint32_t& result);
	bool fr_extend(
		smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
		Vertex_t& new_v, std::uint32_t& result);

	Eigen::VectorXd compute_task_error(const smpl::RobotState& qs);
	smpl::RobotState random_displacement(const smpl::RobotState& q);
	void jacobian(const smpl::RobotState& q, Eigen::MatrixXd& Jq);
	void retract_config(smpl::RobotState& qs, const Eigen::VectorXd& xerr);
};

} // namespace sampling
} // namespace clutter

#endif // TCRRT_HPP
