#include <pushplan/cbswp.hpp>
#include <pushplan/helpers.hpp>
#include <pushplan/robot.hpp>
#include <pushplan/agent.hpp>
#include <pushplan/collision_checker.hpp>
#include <pushplan/constants.hpp>

#include <smpl/console/console.h>

namespace clutter
{

CBSwP::CBSwP() :
CBS()
{
}

CBSwP::CBSwP(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs,
	int scene_id) :
CBS(r, objs, scene_id)
{
}

void CBSwP::growConstraintTree(HighLevelNode* parent)
{
	// expand CT node
	HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
	addConstraints(parent, child[0], child[1]);
	for (int i = 0; i < 2; ++i)
	{
		// does the parent contain the opposite prioritisation?
		if (!parent->m_priorities.Connected(parent->m_conflict[i], parent->m_conflict[1 - i]))
		{
			child[i]->m_priorities.Copy(parent->m_priorities);
			// does the child already contain the "new" prioritisation?
			if (!parent->m_priorities.Connected(parent->m_conflict[1 - i], parent->m_conflict[i]))
			{
				// sanity check
				if (parent->m_conflict[i] == -1 || parent->m_conflict[1 - i] == -1)
				{
					SMPL_ERROR("Improper access into conflict: -1 returned!");
					delete (child[i]);
					continue;
				}
				child[i]->m_priorities.Add(parent->m_conflict[1 - i], parent->m_conflict[i]);
			}
		}
		else
		{
			// cyclical prioritisation is not allowed - do not generate child
			delete (child[i]);
			continue;
		}

		if (updateChild(parent, child[i])) {
			parent->m_children.push_back(child[i]);
		}
		else {
			delete (child[i]);
			continue;
		}
	}
	parent->clear();
}

bool CBSwP::initialiseRoot()
{
	if (CBS::initialiseRoot()) {
		m_OPEN.top()->m_priorities.Clear();
		return true;
	}

	return false;
}

} // namespace clutter
