#include <pushplan/planner.hpp>

#include <string>

using namespace clutter;

int main(int argc, char** argv)
{
	std::string scene(argv[1]);
	Planner p(scene);
	p.WHCAStar();

	return 0;
}
