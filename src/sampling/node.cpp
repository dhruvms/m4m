#include <pushplan/sampling/node.hpp>

namespace clutter {
namespace sampling {

Node::Node(
	const smpl::RobotState& state,
	const comms::ObjectsPoses& objects,
	Node* parent,
	const double& cost) :
m_state(state),
m_objects(objects),
m_parent(parent),
m_cost(cost)
{
	if (m_parent) {
		m_parent->m_children.push_back(this);
	}
}

void Node::set_parent(Node* parent)
{
	m_parent = parent;
	m_parent->m_children.push_back(this);
}

} // namespace sampling
} // namespace clutter
