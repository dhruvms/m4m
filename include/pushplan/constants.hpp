#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <vector>
#include <map>
#include <string>

namespace clutter
{

extern bool FRIDGE;
extern double WHCA_PLANNING_TIME;
extern int WINDOW;
extern double GOAL_THRESH;
extern double RES;
extern int GRID;
extern int COST_MULT;
extern double SEMI_MINOR;
extern double R_MASS;
extern double R_SPEED;
extern bool SAVE;
extern double DF_RES;
extern bool CC_2D;
extern bool CC_3D;

extern const std::vector<int> YCB_OBJECTS;
extern const std::map<int, std::string> YCB_OBJECT_NAMES;
extern const std::map<int, std::vector<double>> YCB_OBJECT_DIMS;

extern const std::map<int, std::string> EXEC_TRAJ_FAIL;

extern double DEG5;

} // namespace clutter


#endif // CONSTANTS_HPP
