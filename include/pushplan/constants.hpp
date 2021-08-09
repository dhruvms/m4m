#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <vector>
#include <map>
#include <string>

namespace clutter
{

extern bool FRIDGE;
extern double MAX_PLANNING_TIME;
extern int WINDOW;
extern double GOAL_THRESH;
extern double RES;
extern int GRID;
extern int COST_MULT;
extern double SEMI_MINOR;
extern double R_MASS;
extern double R_SPEED;

extern const std::vector<int> YCB_OBJECTS;
extern const std::map<int, std::string> YCB_OBJECT_NAMES;
extern const std::map<int, double> YCB_OBJECT_HEIGHTS;

} // namespace clutter


#endif // CONSTANTS_HPP
