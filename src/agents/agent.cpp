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
#include <smpl/debug/marker_conversions.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/marker.h>
#include <sbpl_collision_checking/collision_operations.h>
#include <leatherman/viz.h>

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

bool Agent::Init(bool backwards)
{
	m_init.t = 0;
	m_init.hc = 0;
	m_init.state.clear();
	m_init.state = { 	m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z,
						m_obj_desc.o_roll, m_obj_desc.o_pitch, m_obj_desc.o_yaw };
	ContToDisc(m_init.state, m_init.coord);
	// VisualiseState(m_init, "start_state", 90);

	if (m_solve.empty()) {
		computeGoal(backwards);
	}
	else {
		m_goal = m_solve.back().coord;
	}
	createLatticeAndSearch(backwards);

	return true;
}

void Agent::ComputeNGRComplement(
	double ox, double oy, double oz,
	double sx, double sy, double sz, bool vis)
{
	int x_c, y_c, z_c;
	m_ngr_grid->worldToGrid(ox + (sx / 2.0), oy + (sy / 2.0), oz + (sz / 2.0), x_c, y_c, z_c);
	double res = m_ngr_grid->resolution();
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
				if (m_ngr_grid->getDistanceField()->getCellDistance(x, y, z) <= 0.0 ||
					m_obs_grid->getDistanceField()->getCellDistance(x, y, z) <= 0.0)
				{
					complement = false;
					break;
				}
			}
			if (complement)
			{
				m_ngr_grid->gridToWorld(x, y, z_c - z_s, wx, wy, wz);
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
						m_ngr_grid->resolution(),
						smpl::visual::Color{ 0.8f, 0.255f, 1.0f, 1.0f },
						m_planning_frame,
						"complement"));
	}
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

void Agent::VisualiseState(const Coord& c, const std::string& ns, int hue)
{
	LatticeState s;
	s.coord = c;
	DiscToCont(s.coord, s.state);
	VisualiseState(s, ns, hue);
}

void Agent::VisualiseState(const LatticeState& s, const std::string& ns, int hue)
{
	Eigen::Affine3d T = Eigen::Translation3d(s.state[0], s.state[1], m_obj_desc.o_z) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

	m_obj.SetTransform(T);

	std::vector<std::vector<double>> sphere_positions;
	std::vector<double> sphere_radii;
	for (auto& sphere : m_obj.SpheresState()->spheres)
	{
		if (!sphere.isLeaf()) {
			continue;
		}
		m_obj.updateSphereState(smpl::collision::SphereIndex(sphere.parent_state->index, sphere.index()));

		sphere_positions.push_back({ sphere.pos.x(), sphere.pos.y(), sphere.pos.z() });
		sphere_radii.push_back(sphere.model->radius);
	}

	auto ma = viz::getSpheresMarkerArray(
		sphere_positions, sphere_radii, hue, "", "agent_state", GetID());
	for (auto& m : ma.markers) {
		m.header.frame_id = m_ngr_grid->getReferenceFrame();
	}

	std::vector<smpl::visual::Marker> markers;
	markers.reserve(ma.markers.size());
	smpl::visual::Marker m;
	for (auto& mm : ma.markers) {
		smpl::visual::ConvertMarkerMsgToMarker(mm, m);
		markers.push_back(m);
	}
	for (auto& marker : markers) {
		marker.ns = ns;
	}

	SV_SHOW_INFO_NAMED(ns, markers);
}

const Object* Agent::GetObject(const LatticeState& s)
{
	m_obj.desc.o_x = s.state.at(0);
	m_obj.desc.o_y = s.state.at(1);
	return &m_obj;
}

bool Agent::GetSE2Push(std::vector<double>& push)
{
	push.clear();
	double move_dir = std::atan2(
					m_solve.back().state.at(1) - m_solve.front().state.at(1),
					m_solve.back().state.at(0) - m_solve.front().state.at(0));

	// default values
	push[0] = m_obj_desc.o_x;
	push[1] = m_obj_desc.o_y;
	push[2] = move_dir;

	// Ray-AABB intersection code from
	// https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/ray-tracing-rendering-simple-shapes&src=1

	// AABB bounds
	m_obj.UpdatePose(m_init);
	auto aabb = m_obj.GetFCLObject()->getAABB();
	std::vector<fcl::Vec3f> bounds = {aabb.min_, aabb.max_};

	// Push direction and inverse direction
	Eigen::Vector3f push_dir(
			std::cos(move_dir + M_PI),
			std::sin(move_dir + M_PI),
			0.0);
	push_dir.normalize();
	Eigen::Vector3f inv_dir = push_dir.array().inverse();

	// Push direction sign vector
	Eigen::Vector3i push_sign(
		inv_dir[0] < 0, inv_dir[1] < 0, inv_dir[2] < 0);

	float tmin, tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[push_sign[0]][0] - m_obj_desc.o_x) * inv_dir[0];
	tmax = (bounds[1 - push_sign[0]][0] - m_obj_desc.o_x) * inv_dir[0];
	tymin = (bounds[push_sign[1]][1] - m_obj_desc.o_y) * inv_dir[1];
	tymax = (bounds[1 - push_sign[1]][1] - m_obj_desc.o_y) * inv_dir[1];

	if ((tmin > tymax) || (tymin > tmax)) {
		return false;
	}

	if (tymin > tmin) {
		tmin = tymin;
	}
	if (tymax < tmax) {
		tmax = tymax;
	}

	tzmin = (bounds[push_sign[2]][2] - m_obj_desc.o_z) * inv_dir[2];
	tzmax = (bounds[1 - push_sign[2]][2] - m_obj_desc.o_z) * inv_dir[2];

	if ((tmin > tzmax) || (tzmin > tmax)) {
		return false;
	}

	if (tzmin > tmin) {
		tmin = tzmin;
	}
	if (tzmax < tmax) {
		tmax = tzmax;
	}

	float t = tmin;

	if (t < 0)
	{
		t = tmax;
		if (t < 0) {
			return false;
		}
	}

	if (t > 1) {
		return false;
	}

	push[0] = m_obj_desc.o_x + push_dir[0] * t;
	push[1] = m_obj_desc.o_y + push_dir[1] * t;

	return true;
}

// find best NGR complement cell
// 1. if object is fully outside NGR, this is the initial location
// 2. if object is partially inside NGR, this is the "farthest" object cell
// 3. if object is fully inside NGR, this is the closest cell outside NGR
// (ideally would inflate the NGR by the object and then find such a cell)
bool Agent::computeGoal(bool backwards)
{
	if (backwards) {
		m_goal = m_init.coord;

		// VisualiseState(m_goal, "goal_state", 20);
		return true;
	}

	if (stateOutsideNGR(m_init))
	{
		m_goal = m_init.coord;

		// VisualiseState(m_goal, "goal_state", 20);
		return true;
	}
	else
	{
		Eigen::Affine3d T = Eigen::Translation3d(m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z) *
						Eigen::AngleAxisd(m_obj_desc.o_yaw, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(m_obj_desc.o_pitch, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(m_obj_desc.o_roll, Eigen::Vector3d::UnitX());
		m_obj.updateVoxelsState(T);
		auto voxels_state = m_obj.VoxelsState();

		double best_pos_dist = std::numeric_limits<double>::lowest(), best_neg_dist = std::numeric_limits<double>::lowest(), dist;
		Eigen::Vector3i best_outside_pos, best_inside_pos, pos;
		bool inside = false, outside = false;

		auto ngr_df = m_ngr_grid->getDistanceField();
		for (const Eigen::Vector3d& v : voxels_state->voxels)
		{
			auto cell = dynamic_cast<smpl::PropagationDistanceField&>(*ngr_df).getNearestCell(v[0], v[1], v[2], dist, pos);
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
			SMPL_ERROR("m_init !stateOutsideNGR and !inside?");
			m_goal = m_init.coord;

			// VisualiseState(m_goal, "goal_state", 20);
			return true;
		}

		double wx, wy, wz;
		if (inside && outside) {
			m_ngr_grid->gridToWorld(best_outside_pos[0], best_outside_pos[1], best_outside_pos[2], wx, wy, wz);
		}
		else {
			m_ngr_grid->gridToWorld(best_inside_pos[0], best_inside_pos[1], best_inside_pos[2], wx, wy, wz);
		}

		State goal = {wx, wy};
		ContToDisc(goal, m_goal);

		// VisualiseState(m_goal, "goal_state", 20);
		return true;
	}
}

bool Agent::createLatticeAndSearch(bool backwards)
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
							m_obj, q, *(m_ngr_grid.get()), padding, dist);
}

} // namespace clutter
