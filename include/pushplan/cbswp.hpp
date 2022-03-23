#ifndef CBSWP_HPP
#define CBSWP_HPP

#include <pushplan/cbs.hpp>
#include <pushplan/cbs_nodes.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <memory>

namespace clutter
{

class Robot;
class Agent;

class CBSwP : public CBS
{
public:
	CBSwP();
	CBSwP(std::shared_ptr<Robot> r, std::vector<std::shared_ptr<Agent> > objs, int scene_id=-1);

private:
	bool initialiseRoot() override;
	void growConstraintTree(HighLevelNode* parent) override;
};

} // namespace clutter


#endif // CBSWP_HPP
