#ifndef NODE_HPP
#define NODE_HPP

#include <comms/ObjectsPoses.h>
#include <smpl/types.h>

namespace clutter {
namespace sampling {

class Node
{
public:
	Node() : m_parent(nullptr), m_cost(-1.0), m_goal(false) {};
	Node(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects,
		Node* parent=nullptr,
		const double& cost=-1.0);

	void set_objects(const comms::ObjectsPoses& objects) { m_objects = objects; };
	void set_parent(Node* parent);
	void set_cost(double cost) { m_cost = cost; };
	void set_goal_node() { m_goal = true; };
	bool is_goal_node() { return m_goal; };
	void remove_child(Node* child) { m_children.remove(child); };

	auto robot_state() const -> const smpl::RobotState& { return m_state; }
	auto objects() const -> const comms::ObjectsPoses& { return m_objects; }
	const Node* parent() const { return m_parent; }
	Node* parent() { return m_parent; }
	auto cost() const -> const double&  { return m_cost; }

private:
	smpl::RobotState m_state;
	comms::ObjectsPoses m_objects;
	Node* m_parent;
	std::list<Node*> m_children;
	double m_cost;
	bool m_goal;
};

} // namespace sampling
} // namespace clutter

#endif // NODE_HPP
