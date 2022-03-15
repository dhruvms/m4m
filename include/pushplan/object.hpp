#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <pushplan/types.hpp>

#include <sbpl_collision_checking/shapes.h>
#include <fcl/collision_object.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>

namespace clutter
{

struct Object
{
	int id, shape, type;
	double o_x, o_y, o_z;
	double o_roll, o_pitch, o_yaw;
	double x_size, y_size, z_size;
	double mass, mu;
	bool movable, locked, ycb;

	smpl::collision::CollisionShape* smpl_shape = nullptr;
	smpl::collision::CollisionObject* smpl_co = nullptr;
	std::vector<std::vector<Eigen::Vector3d>> obj_voxels;

	moveit_msgs::CollisionObject* moveit_obj = nullptr;
	fcl::CollisionObject* fcl_obj = nullptr;

	int Shape() const;
	bool CreateCollisionObjects();
	bool CreateSMPLCollisionObject();

	bool TransformAndVoxelise(
		const Eigen::Affine3d& transform,
		const double& res, const Eigen::Vector3d& origin,
		const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);
	bool Voxelise(
		const double& res, const Eigen::Vector3d& origin,
		const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);

	void UpdatePose(const LatticeState& s);

	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		msg = *moveit_obj;
	};
	fcl::CollisionObject* GetFCLObject() { return fcl_obj; };
};

} // namespace clutter


#endif // OBJECT_HPP
