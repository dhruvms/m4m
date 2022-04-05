#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <pushplan/types.hpp>

#include <sbpl_collision_checking/base_collision_models.h>
#include <sbpl_collision_checking/base_collision_states.h>
#include <sbpl_collision_checking/shapes.h>
#include <fcl/collision_object.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>

namespace clutter
{

struct ObjectDesc
{
	int id, shape, type;
	double o_x, o_y, o_z;
	double o_roll, o_pitch, o_yaw;
	double x_size, y_size, z_size;
	double mass, mu, yaw_offset;
	bool movable, locked, ycb;
};

struct Object
{
	ObjectDesc desc;

	smpl::collision::CollisionShape* smpl_shape = nullptr;
	smpl::collision::CollisionObject* smpl_co = nullptr;

	smpl::collision::CollisionSpheresModel* spheres_model = nullptr;
	smpl::collision::CollisionSpheresState* spheres_state = nullptr;
    smpl::collision::CollisionVoxelsModel* voxels_model = nullptr;
    smpl::collision::CollisionVoxelsState* voxels_state = nullptr;

	moveit_msgs::CollisionObject* moveit_obj = nullptr;
	fcl::CollisionObject* fcl_obj = nullptr;

	int Shape() const;
	bool CreateCollisionObjects();
	bool CreateSMPLCollisionObject();
	bool GenerateCollisionModels();

	void SetTransform(const Eigen::Affine3d& T) { m_T = T; };
	void updateSphereState(const smpl::collision::SphereIndex& sidx);

	// bool TransformAndVoxelise(
	// 	const Eigen::Affine3d& transform,
	// 	const double& res, const Eigen::Vector3d& origin,
	// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);
	// bool Voxelise(
	// 	const double& res, const Eigen::Vector3d& origin,
	// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);

	auto SpheresState() -> smpl::collision::CollisionSpheresState* { return spheres_state; }
	auto VoxelsState() -> smpl::collision::CollisionVoxelsState* { return voxels_state; }

	void UpdatePose(const LatticeState& s);

	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		msg = *moveit_obj;
	};
	fcl::CollisionObject* GetFCLObject() { return fcl_obj; };

private:
	Eigen::Affine3d m_T;

	bool createSpheresModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
	    const smpl::collision::Affine3dVector& transforms);
	bool createVoxelsModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
	    const smpl::collision::Affine3dVector& transforms);
	bool generateSpheresModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
		const smpl::collision::Affine3dVector& transforms,
		smpl::collision::CollisionSpheresModelConfig& spheres_model);
	void generateVoxelsModel(
		smpl::collision::CollisionVoxelModelConfig& voxels_model);
	bool voxelizeAttachedBody(
		const std::vector<shapes::ShapeConstPtr>& shapes,
		const smpl::collision::Affine3dVector& transforms,
		smpl::collision::CollisionVoxelsModel& model) const;
};

} // namespace clutter


#endif // OBJECT_HPP
