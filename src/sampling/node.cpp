#include <pushplan/sampling/node.hpp>

namespace clutter {
namespace sampling {

Node::Node(
	const smpl::RobotState& state,
	const comms::ObjectsPoses& objects,
	Node* parent) :
m_state(state),
m_objects(objects),
m_parent(parent)
{
	if (m_parent) {
		m_parent->m_children.push_back(this);
	}
}

} // namespace sampling
} // namespace clutter
