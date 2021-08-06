#include <pushplan/constants.hpp>

bool clutter::FRIDGE;
double clutter::MAX_PLANNING_TIME;
int clutter::WINDOW;
double clutter::GOAL_THRESH;
double clutter::RES;
int clutter::GRID;
int clutter::COST_MULT;
double clutter::SEMI_MINOR;
double clutter::R_MASS;

const std::vector<int> clutter::YCB_OBJECTS = {2, 3, 4, 5, 6, 10, 19, 24, 25, 35};
const std::map<int, std::string> clutter::YCB_OBJECT_NAMES = {
	{2, "002_master_chef_can"},
	{3, "003_cracker_box"},
	{4, "004_sugar_box"},
	{5, "005_tomato_soup_can"},
	{6, "006_mustard_bottle"},
	{10, "010_potted_meat_can"},
	{19, "019_pitcher_base"},
	{24, "024_bowl"},
	{25, "025_mug"},
	{35, "035_power_drill"},
};
const std::map<int, double> clutter::YCB_OBJECT_HEIGHTS = {
	{2, 0.0695},
	{3, 0.095},
	{4, 0.0775},
	{5, 0.05},
	{6, 0.095},
	{10, 0.041},
	{19, 0.1175},
	{24, 0.027},
	{25, 0.039},
	{35, 0.092},
};
