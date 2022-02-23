#ifndef TYPES_HPP
#define TYPES_HPP

#include <pushplan/constants.hpp>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

#include <ostream>
#include <vector>
#include <memory>

namespace clutter
{

typedef std::vector<int> Coord;
typedef std::vector<double> State;

struct LatticeState
{
	Coord coord;
	State state;
	int t, hc;
};

typedef std::vector<LatticeState> Trajectory;
typedef std::vector<LatticeState*> STATES;

inline
bool operator==(const LatticeState& a, const LatticeState& b)
{
	return (
		a.coord == b.coord &&
		a.t == b.t
	);
}

} // namespace clutter

namespace std {

template <>
struct hash<clutter::LatticeState>
{
	typedef clutter::LatticeState argument_type;
	typedef std::size_t result_type;
	result_type operator()(const argument_type& s) const;
};

} // namespace std

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

	moveit_msgs::CollisionObject* moveit_obj = nullptr;
	fcl::CollisionObject* fcl_obj = nullptr;

	int Shape() const;
	bool CreateCollisionObjects();
	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		msg = *moveit_obj;
	};
	fcl::CollisionObject* GetFCLObject() { return fcl_obj; };

	void UpdatePose(const LatticeState& s)
	{
		// Update Moveit pose
		geometry_msgs::Pose moveit_pose;
		Eigen::Quaterniond q;
		geometry_msgs::Quaternion orientation;

		moveit_pose.position.x = s.state.at(0);
		moveit_pose.position.y = s.state.at(1);
		moveit_pose.position.z = o_z;

		double yaw_offset = 0.0;
		if (ycb)
		{
			auto itr2 = YCB_OBJECT_DIMS.find(shape);
			if (itr2 != YCB_OBJECT_DIMS.end()) {
				yaw_offset = itr2->second.at(3);
			}
		}

		smpl::angles::from_euler_zyx(o_yaw - yaw_offset, o_pitch, o_roll, q);
		tf::quaternionEigenToMsg(q, orientation);

		moveit_pose.orientation = orientation;

		if (ycb) {
			moveit_obj->mesh_poses[0] = moveit_pose;
		}
		else {
			moveit_obj->primitive_poses[0] = moveit_pose;
		}

		// Update FCL pose
		fcl::Quaternion3f fcl_q(q.w(), q.x(), q.y(), q.z());
		fcl::Vec3f fcl_T(s.state.at(0), s.state.at(1), o_z);
		fcl::Transform3f fcl_pose(fcl_q, fcl_T);

		fcl_obj->setTransform(fcl_pose);
		fcl_obj->computeAABB();
	}
};

class Search
{
public:
	virtual int set_start(int start_id) = 0;
	virtual int set_goal(int goal_id) = 0;
	virtual void set_max_planning_time(double max_planning_time_ms) = 0;
	virtual int get_n_expands() const = 0;
	virtual void reset() = 0;

	virtual int replan(
		std::vector<int>* solution_path, int* solution_cost) = 0;
};

struct RobotModelConfig
{
	std::string group_name;
	std::vector<std::string> planning_joints;
	std::string kinematics_frame;
	std::string chain_tip_link;
	std::vector<std::string> push_links;
	std::vector<std::string> gripper_joints;
};

struct PlannerConfig
{
	std::string discretization;
	std::string mprim_filename;
	bool use_xyz_snap_mprim;
	bool use_rpy_snap_mprim;
	bool use_xyzrpy_snap_mprim;
	bool use_short_dist_mprims;
	double xyz_snap_dist_thresh;
	double rpy_snap_dist_thresh;
	double xyzrpy_snap_dist_thresh;
	double short_dist_mprims_thresh;
};

inline
int Object::Shape() const
{
	if (!ycb) {
		return shape;
	}
	else {
		if (shape == 36) {
			return 0;
		}
		int s = x_size == y_size ? 2 : 0;
		return s;
	}
}

inline
bool Object::CreateCollisionObjects()
{
	moveit_obj = new moveit_msgs::CollisionObject();
	moveit_obj->id = std::to_string(id);

	geometry_msgs::Pose pose;
	Eigen::Quaterniond q;
	geometry_msgs::Quaternion orientation;
	if (!ycb)
	{
		shape_msgs::SolidPrimitive prim;

		if (shape == 0)
		{
			// Create Moveit collision object shape
			prim.type = shape_msgs::SolidPrimitive::BOX;
			prim.dimensions.resize(3);
			prim.dimensions[0] = x_size * 2;
			prim.dimensions[1] = y_size * 2;
			prim.dimensions[2] = z_size * 2;

			// Create FCL collision object
			std::shared_ptr<fcl::Box> fcl_shape =
				std::make_shared<fcl::Box>(x_size * 2, y_size * 2, z_size * 2);
			fcl_obj = new fcl::CollisionObject(fcl_shape);
		}
		else if (shape == 2)
		{
			// Create Moveit collision object shape
			prim.type = shape_msgs::SolidPrimitive::CYLINDER;
			prim.dimensions.resize(2);
			prim.dimensions[0] = z_size;
			prim.dimensions[1] = x_size;

			// Create FCL collision object
			std::shared_ptr<fcl::Cylinder> fcl_shape =
				std::make_shared<fcl::Cylinder>(x_size, z_size);
			fcl_obj = new fcl::CollisionObject(fcl_shape);
		}

		int table_ids = FRIDGE ? 5 : 1;
		if (id <= table_ids)
		{
			prim.dimensions[0] *= (1.0 + (sqrt(3) * DF_RES));
			prim.dimensions[1] *= (1.0 + (sqrt(3) * DF_RES));
			prim.dimensions[2] *= (1.0 + (sqrt(3) * DF_RES));
		}

		pose.position.x = o_x;
		pose.position.y = o_y;
		pose.position.z = o_z;

		smpl::angles::from_euler_zyx(o_yaw, o_pitch, o_roll, q);
		tf::quaternionEigenToMsg(q, orientation);

		pose.orientation = orientation;
		moveit_obj->primitives.push_back(prim);
		moveit_obj->primitive_poses.push_back(pose);
	}
	else
	{
		// Create Moveit collision object
		std::string stl_mesh(__FILE__);
		auto found = stl_mesh.find_last_of("/\\");
		stl_mesh = stl_mesh.substr(0, found + 1) + "../../dat/ycb/?/tsdf/nontextured.stl";

		auto itr1 = YCB_OBJECT_NAMES.find(shape);
		std::string object_name(itr1->second);
		found = stl_mesh.find_last_of("?");
		stl_mesh.insert(found, object_name);
		found = stl_mesh.find_last_of("?");
		stl_mesh.erase(found, 1);
		stl_mesh.insert(0, "file://");

		shapes::Mesh* mesh = nullptr;
		mesh = shapes::createMeshFromResource(stl_mesh, Eigen::Vector3d::Constant(1.0));
		shape_msgs::Mesh mesh_msg;
		shapes::ShapeMsg shape_msg = mesh_msg;
		if (shapes::constructMsgFromShape(mesh, shape_msg)) {
			mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
		}
		else {
			SMPL_ERROR("Failed to get Mesh msg!");
			return false;
		}

		pose.position.x = o_x;
		pose.position.y = o_y;
		pose.position.z = o_z;

		double yaw_offset = 0.0;
		auto itr2 = YCB_OBJECT_DIMS.find(shape);
		if (itr2 != YCB_OBJECT_DIMS.end()) {
			yaw_offset = itr2->second.at(3);
		}

		smpl::angles::from_euler_zyx(
				o_yaw - yaw_offset, o_pitch, o_roll, q);
		tf::quaternionEigenToMsg(q, orientation);
		pose.orientation = orientation;

		moveit_obj->meshes.push_back(mesh_msg);
		moveit_obj->mesh_poses.push_back(pose);

		// Create FCL collision object
		auto object_mesh = moveit_obj->meshes[0];

	    std::vector<fcl::Vec3f> vertices;
	    std::vector<fcl::Triangle> triangles;

	    vertices.reserve(object_mesh.vertices.size());
	    for (unsigned int i = 0; i < object_mesh.vertices.size(); ++i) {
	        double x = object_mesh.vertices[i].x;
	        double y = object_mesh.vertices[i].y;
	        double z = object_mesh.vertices[i].z;
	        vertices.push_back(fcl::Vec3f(x, y, z));
	    }

	    triangles.reserve(object_mesh.triangles.size());
	    for (unsigned int i = 0; i < object_mesh.triangles.size(); ++i) {
	        fcl::Triangle t;
	        t[0] = object_mesh.triangles[i].vertex_indices[0];
	        t[1] = object_mesh.triangles[i].vertex_indices[1];
	        t[2] = object_mesh.triangles[i].vertex_indices[2];
	        triangles.push_back(t);
	    }
	    typedef fcl::BVHModel<fcl::OBBRSS> Model;
	    std::shared_ptr<Model> mesh_geom = std::make_shared<Model>();

	    mesh_geom->beginModel();
	    mesh_geom->addSubModel(vertices, triangles);
	    mesh_geom->endModel();

	    fcl_obj = new fcl::CollisionObject(mesh_geom);
	}

	return true;
}

} // namespace clutter

#endif // TYPES_HPP
