#include <pushplan/sampling/tcrrt.hpp>
#include <pushplan/sampling/node.hpp>
#include <pushplan/utils/geometry.hpp>

#include <stdexcept>

namespace clutter {
namespace sampling {

TCRRT::TCRRT(
	int mode, int I, int J)
:
RRT(),
m_mode(mode), m_I(I), m_J(J)
{
	m_dmax = (m_steps * m_gthresh);
	m_T_0_t = Eigen::Affine3d::Identity();
}

TCRRT::TCRRT(
	int samples, int steps, double gbias, double gthresh, double timeout,
	int mode, int I, int J)
:
RRT(samples, steps, gbias, gthresh, timeout),
m_mode(mode), m_I(I), m_J(J)
{
	m_dmax = (m_steps * m_gthresh);
	m_T_0_t = Eigen::Affine3d::Identity();
}

bool TCRRT::Solve()
{
	if (m_mode < 0 || m_mode > 2) {
		throw std::runtime_error("Invalid TC-RRT mode!");
	}

	return RRT::Solve();
}

bool TCRRT::extend(const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result)
{
	// find qnear in tree
	Node* xnear = nullptr;
	Vertex_t near_v;
	if (selectVertex(sample, near_v)) {
		xnear = m_G[near_v];
	}
	else
	{
		result = 0x0002710; // 10000
		return false;
	}

	double extend_dist = configDistance(sample, xnear->robot_state());
	if (extend_dist < m_gthresh)
	{
		result = 0x000003E8; // 1000 if sampled config too near to tree node
		return true;
	}

	// compute simple extended state qs
	smpl::RobotState qs(xnear->robot_state().size(), 0.0);
	for (std::size_t i = 0; i < qs.size(); ++i)
	{
		double dir_i = smpl::angles::shortest_angle_diff(sample.at(i), xnear->robot_state().at(i))/extend_dist;
		qs.at(i) = xnear->robot_state().at(i) + (m_steps * m_gthresh) * dir_i;
	}

	switch (m_mode)
	{
		case 0:
		{
			return rgd_extend(qs, xnear, near_v, new_v, result);
		}
		case 1:
		{
			return ts_extend(qs, xnear, near_v, new_v, result);
		}
		case 2:
		{
			return fr_extend(qs, xnear, near_v, new_v, result);
		}
	}
}

bool TCRRT::rgd_extend(
	smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
	Vertex_t& new_v, std::uint32_t& result)
{
	int i = 0, j = 0;

	smpl::RobotState qnear = xnear->robot_state();
	Eigen::VectorXd xerr = compute_task_error(qs);
	while (i < m_I && j < m_J && xerr.norm() > m_gthresh)
	{
		++i;
		++j;

		smpl::RobotState qs2 = random_displacement(qs);
		Eigen::VectorXd xerr2 = compute_task_error(qs2);

		if (xerr2.norm() < xerr.norm())
		{
			j = 0;
			qs = qs2;
			xerr = xerr2;
		}

		if (xerr.norm() <= m_gthresh)
		{
			smpl::RobotState qnew;
			comms::ObjectsPoses qnew_objs;

			double steer_time = GetTime();
			bool steered = steer(qs, xnear, qnew, qnew_objs, result);
			steer_time = GetTime() - steer_time;

			if (steered)
			{
				// successful steer
				Node* xnew = new Node(qnew, qnew_objs, xnear);
				addNode(xnew, new_v);
				addEdge(near_v, new_v);
			}

			if (result >= 9 && result <= 101)
			{
				++m_stats["sims"];
				m_stats["sim_time"] += steer_time;
			}

			return steered;
		}
	}

	return false;
}

bool TCRRT::ts_extend(
	smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
	Vertex_t& new_v, std::uint32_t& result)
{
	smpl::RobotState qnear = xnear->robot_state();
	Eigen::MatrixXd Jqs, Jqs_pinv, C;
	jacobian(qnear, Jqs);
	Jqs_pinv = MoorePenrosePInv(Jqs);
	C = Eigen::MatrixXd::Zero(6, 6);
	C(2) = 1;

	Eigen::VectorXd qdiff = Eigen::VectorXd::Zero(qs.size());
	for (int jidx = 0; jidx < qs.size(); jidx++) {
		qdiff(jidx) = smpl::angles::shortest_angle_diff(qs.at(jidx), qnear.at(jidx));
	}

	Eigen::VectorXd qdiff2 = qdiff - Jqs_pinv * C * Jqs * qdiff;
	for (int jidx = 0; jidx < qs.size(); jidx++) {
		qs.at(jidx) = qnear.at(jidx) + qdiff2(jidx);
	}
	return rgd_extend(qs, xnear, near_v, new_v, result);
}

bool TCRRT::fr_extend(
	smpl::RobotState& qs, Node* xnear, const Vertex_t& near_v,
	Vertex_t& new_v, std::uint32_t& result)
{
	smpl::RobotState qr = qs, qnear = xnear->robot_state();
	Eigen::VectorXd xerr = compute_task_error(qs);

	while (xerr.norm() > m_gthresh)
	{
		retract_config(qs, xerr);
		if (configDistance(qs, qr) > configDistance(qr, qnear)) {
			return false;
		}
		xerr = compute_task_error(qs);
	}

	smpl::RobotState qnew;
	comms::ObjectsPoses qnew_objs;

	double steer_time = GetTime();
	bool steered = steer(qs, xnear, qnew, qnew_objs, result);
	steer_time = GetTime() - steer_time;

	if (steered)
	{
		// successful steer
		Node* xnew = new Node(qnew, qnew_objs, xnear);
		addNode(xnew, new_v);
		addEdge(near_v, new_v);
	}

	if (result >= 9 && result <= 101)
	{
		++m_stats["sims"];
		m_stats["sim_time"] += steer_time;
	}

	return steered;
}

Eigen::VectorXd TCRRT::compute_task_error(const smpl::RobotState& qs)
{
	Eigen::Affine3d ee_pose, T_t_e;
	m_robot->ComputeFK(qs, ee_pose);
	T_t_e = m_T_0_t.inverse() * ee_pose;

	Eigen::VectorXd xerr = Eigen::VectorXd::Zero(6);
	xerr[2] = T_t_e.translation().z();
	return xerr;
}

smpl::RobotState TCRRT::random_displacement(const smpl::RobotState& q)
{
	// smpl::RobotState qs2;
	// while (true)
	// {
	// 	m_robot->GetRandomState(qs2);
	// 	if (configDistance(q, qs2) <= m_dmax) {
	// 		break;
	// 	}
	// }

	// return qs2;

	Eigen::VectorXd disp = Eigen::VectorXd::Zero(q.size());
	for (std::size_t i = 0; i < q.size(); ++i) {
		disp(i) = m_distG(m_rng);
	}
	disp /= disp.norm();
	disp *= m_distD(m_rng) * m_dmax;

	smpl::RobotState qs2 = q;
	for (std::size_t i = 0; i < q.size(); ++i) {
		qs2.at(i) += disp(i);
	}

	return qs2;
}

void TCRRT::jacobian(const smpl::RobotState& q, Eigen::MatrixXd& Jq)
{
	Eigen::Affine3d ee_pose;
	m_robot->ComputeFK(q, ee_pose);
	m_robot->ComputeJacobian(q, Jq);

	double phi, theta, psi;
	smpl::angles::get_euler_zyx(ee_pose.rotation(), phi, theta, psi);
	double cphi, ctheta, sphi, stheta;
	cphi = std::cos(phi);
	ctheta = std::cos(theta);
	sphi = std::sin(phi);
	stheta = std::sin(theta);

	Eigen::MatrixXd E = Eigen::MatrixXd::Identity(6, 6);
	E(3, 3) = cphi/ctheta;
	E(3, 4) = sphi/ctheta;
	E(4, 3) = -sphi;
	E(4, 4) = cphi;
	E(5, 3) = (cphi * stheta)/ctheta;
	E(5, 4) = (sphi * stheta)/ctheta;

	Jq = E * Jq;
}

void TCRRT::retract_config(smpl::RobotState& qs, const Eigen::VectorXd& xerr)
{
	Eigen::MatrixXd Jqs, Jqs_pinv;
	jacobian(qs, Jqs);
	Jqs_pinv = MoorePenrosePInv(Jqs);

	Eigen::VectorXd qerr = Jqs_pinv * xerr;
	for (int jidx = 0; jidx < qs.size(); jidx++) {
		qs.at(jidx) -= qerr(jidx);
	}
}

} // namespace sampling
} // namespace clutter
