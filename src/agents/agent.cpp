#include <pushplan/agents/agent.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/search/cbs_nodes.hpp>
#include <pushplan/search/conflicts.hpp>
#include <pushplan/search/wastar.hpp>
#include <pushplan/search/focal.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/geometry.hpp>

#include <smpl/console/console.h>
#include <smpl/ros/propagation_distance_field.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/marker.h>
#include <sbpl_collision_checking/collision_operations.h>

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

void Agent::InitNGR()
{
	bool propagate_negative_distances = true;
	double sx, sy, sz, ox, oy, oz, max_distance;

	m_ph.getParam("occupancy_grid/size_x", sx);
	m_ph.getParam("occupancy_grid/size_y", sy);
	m_ph.getParam("occupancy_grid/size_z", sz);
	m_ph.getParam("occupancy_grid/origin_x", ox);
	m_ph.getParam("occupancy_grid/origin_y", oy);
	m_ph.getParam("occupancy_grid/origin_z", oz);
	m_ph.getParam("occupancy_grid/max_dist", max_distance);
	m_ph.getParam("planning_frame", m_planning_frame);

	using DistanceMapType = smpl::PropagationDistanceField;

	m_df = std::make_shared<DistanceMapType>(
			ox, oy, oz,
			sx, sy, sz,
			DF_RES,
			max_distance, propagate_negative_distances);

	bool ref_counted = false;
	m_ngr = std::make_unique<smpl::OccupancyGrid>(m_df, ref_counted);
	m_ngr->setReferenceFrame(m_planning_frame);
}

bool Agent::Init()
{
	m_init.t = 0;
	m_init.hc = 0;
	m_init.state.clear();
	m_init.state = { 	m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z,
						m_obj_desc.o_roll, m_obj_desc.o_pitch, m_obj_desc.o_yaw };
	ContToDisc(m_init.state, m_init.coord);

	return true;
}

void Agent::UpdateNGR(const std::vector<std::vector<Eigen::Vector3d>>& voxels, bool vis)
{
	for (auto& voxel_list : voxels) {
		m_ngr->addPointsToField(voxel_list);
	}

	if (vis) {
		// SV_SHOW_INFO(m_ngr->getBoundingBoxVisualization());
		// SV_SHOW_INFO(m_ngr->getDistanceFieldVisualization());
		SV_SHOW_INFO(m_ngr->getOccupiedVoxelsVisualization());
	}
}

void Agent::ComputeNGRComplement(
	double ox, double oy, double oz,
	double sx, double sy, double sz, bool vis)
{
	int x_c, y_c, z_c;
	m_ngr->worldToGrid(ox + (sx / 2.0), oy + (sy / 2.0), oz + (sz / 2.0), x_c, y_c, z_c);
	double res = m_ngr->resolution();
	int x_s = ((sx / 2.0) / res) + 0.5;
	int y_s = ((sy / 2.0) / res) + 0.5;
	int z_s = (((sz / 2.0) - res) / res) + 0.5;

	std::vector<Eigen::Vector3d> complement_voxel_vecs;
	for (int x = x_c - x_s; x < x_c + x_s; ++x)
	{
		for (int y = y_c - y_s; y < y_c + y_s; ++y)
		{
			bool complement = true;
			double wx, wy, wz;
			for (int z = z_c - z_s; z < z_c + z_s; ++z)
			{
				if (m_ngr->getDistanceField()->getCellDistance(x, y, z) <= 0.0 ||
					m_obs_grid->getDistanceField()->getCellDistance(x, y, z) <= 0.0)
				{
					complement = false;
					break;
				}
			}
			if (complement)
			{
				m_ngr->gridToWorld(x, y, z_c - z_s, wx, wy, wz);
				LatticeState s;
				s.state = {wx, wy};

				if (!stateObsCollision(s))
				{
					if (stateOutsideNGR(s))
					{
						m_ngr_complement.emplace(wx, wy, wz);
						if (vis) {
							complement_voxel_vecs.emplace_back(wx, wy, wz);
						}
					}
				}
			}
		}
	}

	if (vis)
	{
		SV_SHOW_INFO(smpl::visual::MakeCubesMarker(
						complement_voxel_vecs,
						m_ngr->resolution(),
						smpl::visual::Color{ 0.8f, 0.255f, 1.0f, 1.0f },
						m_planning_frame,
						"complement"));
	}
}

// find best NGR complement cell
// 1. if object is fully outside NGR, this is the initial location
// 2. if object is partially inside NGR, this is the "farthest" object cell
// 3. if object is fully inside NGR, this is the closest cell outside NGR
// (ideally would inflate the NGR by the object and then find such a cell)
bool Agent::ComputeGoal(bool backwards)
{
	if (backwards) {
		m_goal = m_init.coord;
		return true;
	}

	if (stateOutsideNGR(m_init))
	{
		m_goal = m_init.coord;
		return true;
	}
	else
	{
		Eigen::Affine3d T = Eigen::Translation3d(m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z) *
						Eigen::AngleAxisd(m_obj_desc.o_yaw, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(m_obj_desc.o_pitch, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(m_obj_desc.o_roll, Eigen::Vector3d::UnitX());
		auto voxels_state = m_obj.VoxelsState();
		// transform voxels into the model frame
		std::vector<Eigen::Vector3d> new_voxels(voxels_state->model->voxels.size());
		for (size_t i = 0; i < voxels_state->model->voxels.size(); ++i) {
			new_voxels[i] = T * voxels_state->model->voxels[i];
		}

		voxels_state->voxels = std::move(new_voxels);

		double best_pos_dist = std::numeric_limits<double>::lowest(), best_neg_dist = std::numeric_limits<double>::lowest(), dist;
		Eigen::Vector3i best_outside_pos, best_inside_pos, pos;
		bool inside = false, outside = false;
		for (const Eigen::Vector3d& v : voxels_state->voxels)
		{
			auto cell = std::dynamic_pointer_cast<smpl::PropagationDistanceField>(m_df)->getNearestCell(v[0], v[1], v[2], dist, pos);
			if (dist > 0 && dist > best_pos_dist)
			{
				best_pos_dist = dist;
				best_outside_pos = pos;
				if (!outside) {
					outside = true;
				}
			}
			if (dist < 0 && dist > best_neg_dist)
			{
				best_neg_dist = dist;
				best_inside_pos = pos; // actually the nearest free space (boundary?) cell
				if (!inside) {
					inside = true;
				}
			}
		}
		if (!inside) {
			SMPL_ERROR("How is m_init not valid with NGR but we are not inside?");
		}

		double wx, wy, wz;
		if (inside && outside) {
			m_ngr->gridToWorld(best_outside_pos[0], best_outside_pos[1], best_outside_pos[2], wx, wy, wz);
		}
		else {
			m_ngr->gridToWorld(best_inside_pos[0], best_inside_pos[1], best_inside_pos[2], wx, wy, wz);
		}

		State goal = {wx, wy};
		ContToDisc(goal, m_goal);

		return true;
	}
}

bool Agent::CreateLatticeAndSearch(bool backwards)
{
	m_lattice = std::make_unique<AgentLattice>();
	m_lattice->init(this, backwards);
	m_lattice->reset();

	if (backwards)
	{
		m_search = std::make_unique<WAStar>(m_lattice.get(), 1.0);
		m_search->reset();

		for (const auto& s : m_ngr_complement_states) {
			m_search->push_start(m_lattice->PushStart(s));
		}
		m_search->push_goal(m_lattice->PushGoal(m_goal));
	}
	else
	{
		m_search = std::make_unique<Focal>(m_lattice.get(), 100.0);
		m_search->reset();

		m_search->push_start(m_lattice->PushStart(m_init));
		m_search->push_goal(m_lattice->PushGoal(m_goal));
	}

	return true;
}

bool Agent::SatisfyPath(
	HighLevelNode* ct_node,
	Trajectory** sol_path,
	int& expands,
	int& min_f,
	std::unordered_set<int>* to_avoid)
{
	m_solve.clear();
	expands = 0;
	min_f = 0;

	// collect agent constraints
	m_lattice->SetCTNode(ct_node);
	if (to_avoid != nullptr) {
		m_lattice->AvoidAgents(*to_avoid);
	}

	std::vector<int> solution;
	int solcost;
	bool result = m_search->replan(&solution, &solcost);

	if (result)
	{
		m_lattice->ConvertPath(solution);
		*sol_path = &(this->m_solve);
		expands = m_search->get_n_expands();
		min_f = m_search->get_min_f();
	}

	return result;
}

void Agent::UpdatePose(const LatticeState& s)
{
	m_obj.UpdatePose(s);
}

bool Agent::OutOfBounds(const LatticeState& s)
{
	return m_cc->OutOfBounds(s);
}

bool Agent::ImmovableCollision()
{
	return m_cc->ImmovableCollision(m_obj.GetFCLObject());
}

bool Agent::ObjectObjectCollision(const int& a2_id, const LatticeState& a2_q)
{
	return m_cc->ObjectObjectCollision(m_obj.GetFCLObject(), a2_id, a2_q);
}

bool Agent::ObjectObjectsCollision(
	const std::vector<int>& other_ids,
	const std::vector<LatticeState>& other_poses)
{
	return m_cc->ObjectObjectsCollision(m_obj.GetFCLObject(), other_ids, other_poses);
}

bool Agent::OutsideNGR(const LatticeState& s)
{
	return stateOutsideNGR(s);
}

const Object* Agent::GetObject(const LatticeState& s)
{
	m_obj.desc.o_x = s.state.at(0);
	m_obj.desc.o_y = s.state.at(1);
	return &m_obj;
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

// return false => no collision with obstacles
bool Agent::stateObsCollision(const LatticeState& s)
{
	m_obj.UpdatePose(s);
	return m_cc->ImmovableCollision(m_obj.GetFCLObject());
}

// return false => collide with NGR
bool Agent::stateOutsideNGR(const LatticeState& s)
{
	Eigen::Affine3d T = Eigen::Translation3d(s.state[0], s.state[1], m_obj_desc.o_z) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

	m_obj.SetTransform(T);
	std::vector<const smpl::collision::CollisionSphereState*> q = {
									m_obj.SpheresState()->spheres.root() };

	double padding = 0.0, dist;
	return smpl::collision::CheckVoxelsCollisions(
							m_obj, q, *(m_ngr.get()), padding, dist);
}

} // namespace clutter
