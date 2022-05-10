#ifndef NODE_HPP
#define NODE_HPP

#include <comms/ObjectsPoses.h>
#include <smpl/types.h>

namespace clutter {
namespace sampling {

class Node
{
public:
	Node() : m_parent(nullptr) {};
	Node(
		const smpl::RobotState& state,
		const comms::ObjectsPoses& objects,
		Node* parent=nullptr);

	auto robot_state() const -> const smpl::RobotState& { return m_state; }
	auto objects() const -> const comms::ObjectsPoses& { return m_objects; }
	const Node* parent() const { return m_parent; }

private:
	smpl::RobotState m_state;
	comms::ObjectsPoses m_objects;
	Node* m_parent;
	std::list<Node*> m_children;
};

} // namespace sampling
} // namespace clutter

#endif // NODE_HPP
