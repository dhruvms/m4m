#include <pushplan/agent.hpp>
#include <pushplan/geometry.hpp>
#include <pushplan/cbs_nodes.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/conflicts.hpp>
#include <pushplan/constants.hpp>

#include <smpl/console/console.h>

#include <iostream>
#include <algorithm>

namespace clutter
{

// void Agent::ResetObject()
// {
// 	m_obj_desc.o_x = m_obj_desc.o_x;
// 	m_obj_desc.o_y = m_obj_desc.o_y;
// }

// bool Agent::SetObjectPose(
// 	const std::vector<double>& xyz,
// 	const std::vector<double>& rpy)
// {
// 	m_obj_desc.o_x = xyz.at(0);
// 	m_obj_desc.o_y = xyz.at(1);
// 	m_obj_desc.o_z = xyz.at(2);

// 	m_obj_desc.o_roll = rpy.at(0);
// 	m_obj_desc.o_pitch = rpy.at(1);
// 	m_obj_desc.o_yaw = rpy.at(2);
// }

bool Agent::Init()
{
	// m_obj_desc.o_x = m_obj_desc.o_x;
	// m_obj_desc.o_y = m_obj_desc.o_y;

	// m_init.t = 0;
	// m_init.hc = 0;
	// m_init.state.clear();
	// m_init.state.push_back(m_obj_desc.o_x);
	// m_init.state.push_back(m_obj_desc.o_y);
	// m_init.state.push_back(m_obj_desc.o_z);
	// m_init.state.push_back(m_obj_desc.o_roll);
	// m_init.state.push_back(m_obj_desc.o_pitch);
	// m_init.state.push_back(m_obj_desc.o_yaw);
	// ContToDisc(m_init.state, m_init.coord);

	// if (!m_focal) {
	// 	m_focal = std::make_unique<Focal>(this, 1.0); // make A* search object
	// }
	// this->reset();
	// this->SetStartState(m_init);
	// this->SetGoalState(m_init.coord);

	for (int gx = 0; gx < m_ngr->getDistanceField()->numCellsX(); ++gx) {
	for (int gy = 0; gy < m_ngr->getDistanceField()->numCellsY(); ++gy) {
	for (int gz = 0; gz < m_ngr->getDistanceField()->numCellsZ(); ++gz) {
			double wx, wy, wz;
			m_ngr->gridToWorld(gx, gy, gz, wx, wy, wz);
			m_ngr_complement.emplace_back(wx, wy, wz);
	}
	}
	}

	return true;
}

bool Agent::Init()
{
	m_objs.back().o_x = m_orig_o.o_x;
	m_objs.back().o_y = m_orig_o.o_y;

	m_init.t = 0;
	m_init.hc = 0;
	m_init.state.clear();
	m_init.state.push_back(m_objs.back().o_x);
	m_init.state.push_back(m_objs.back().o_y);
	m_init.state.push_back(m_objs.back().o_z);
	m_init.state.push_back(m_objs.back().o_roll);
	m_init.state.push_back(m_objs.back().o_pitch);
	m_init.state.push_back(m_objs.back().o_yaw);
	ContToDisc(m_init.state, m_init.coord);

	if (!m_focal) {
		m_focal = std::make_unique<Focal>(this, 1.0); // make A* search object
	}
	this->reset();
	this->SetStartState(m_init);
	this->SetGoalState(m_init.coord);

	return true;
}

void Agent::reset()
{
	// reset everything
	for (LatticeState* s : m_states) {
		if (s != nullptr) {
			delete s;
			s = nullptr;
		}
	}
	m_state_to_id.clear();
	m_states.clear();

	m_focal->reset();
}

bool Agent::SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f)
{
	m_solve.clear();
	expands = 0;
	min_f = 0;
	// collect agent constraints
	m_constraints.clear();
	for (auto& constraint : ct_node->m_constraints)
	{
		if (constraint->m_me == ct_node->m_replanned) {
			m_constraints.push_back(constraint);
		}
	}
	m_cbs_solution = &(ct_node->m_solution);
	m_cbs_id = ct_node->m_replanned;
	m_max_time = ct_node->m_makespan;

	std::vector<int> solution;
	int solcost;
	bool result = m_focal->replan(&solution, &solcost);

	if (result)
	{
		convertPath(solution);
		*sol_path = &(this->m_solve);
		expands = m_focal->get_n_expands();
		min_f = m_focal->get_min_f();
	}

	return result;
}

const std::vector<Object>* Agent::GetObject(const LatticeState& s)
{
	m_obj_desc.o_x = s.state.at(0);
	m_obj_desc.o_y = s.state.at(1);
	return &m_objs;
}

void Agent::GetSE2Push(std::vector<double>& push)
{
	push.clear();
	push.resize(3, 0.0); // (x, y, yaw)

	double move_dir = std::atan2(
					m_solve.back().state.at(1) - m_solve.front().state.at(1),
					m_solve.back().state.at(0) - m_solve.front().state.at(0));
	push.at(0) = m_obj_desc.o_x + std::cos(move_dir + M_PI) * (m_obj_desc.x_size + 0.05);
	push.at(1) = m_obj_desc.o_y + std::sin(move_dir + M_PI) * (m_obj_desc.x_size + 0.05);
	push.at(2) = move_dir;

	if (m_obj_desc.shape == 0)
	{
		// get my object rectangle
		std::vector<State> rect;
		State o = {m_obj_desc.o_x, m_obj_desc.o_y};
		GetRectObjAtPt(o, m_obj_desc, rect);

		// find rectangle side away from push direction
		push.at(0) = m_obj_desc.o_x + std::cos(move_dir + M_PI) * 0.5;
		push.at(1) = m_obj_desc.o_y + std::sin(move_dir + M_PI) * 0.5;
		State p = {push.at(0), push.at(1)}, intersection;
		double op = EuclideanDist(o, p);
		int side = 0;
		for (; side <= 3; ++side)
		{
			LineLineIntersect(o, p, rect.at(side), rect.at((side + 1) % 4), intersection);
			if (PointInRectangle(intersection, rect))
			{
				if (EuclideanDist(intersection, o) + EuclideanDist(intersection, p) <= op + 1e-6) {
					break;
				}
			}
		}

		// compute push point on side
		intersection.at(0) += std::cos(move_dir + M_PI) * 0.08;
		intersection.at(1) += std::sin(move_dir + M_PI) * 0.08;

		// update push
		push.at(0) = intersection.at(0);
		push.at(1) = intersection.at(1);
	}
}

} // namespace clutter
