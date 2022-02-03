#include <pushplan/constants.hpp>

bool clutter::FRIDGE;
double clutter::WHCA_PLANNING_TIME;
int clutter::WINDOW;
double clutter::GOAL_THRESH;
double clutter::RES;
int clutter::GRID;
int clutter::COST_MULT;
double clutter::SEMI_MINOR;
double clutter::R_MASS;
double clutter::R_SPEED;
bool clutter::SAVE;
double clutter::DF_RES;
bool clutter::CC_2D;
bool clutter::CC_3D;

const std::vector<int> clutter::YCB_OBJECTS = {2,3,4,5,6,8,9,10,19,21,25,36}; // 7,11,24,35
const std::map<int, std::string> clutter::YCB_OBJECT_NAMES = {
	{2, "002_master_chef_can"},
	{3, "003_cracker_box"},
	{4, "004_sugar_box"},
	{5, "005_tomato_soup_can"},
	{6, "006_mustard_bottle"},
	// {7, "007_tuna_fish_can"},
	{8, "008_pudding_box"},
	{9, "009_gelatin_box"},
	{10, "010_potted_meat_can"},
	// {11, "011_banana"},
	{19, "019_pitcher_base"},
	{21, "021_bleach_cleanser"},
	// {24, "024_bowl"},
	{25, "025_mug"},
	// {35, "035_power_drill"},
	{36, "036_wood_block"}
};
const std::map<int, std::vector<double>> clutter::YCB_OBJECT_DIMS = {
	{2, {0.051, 0.051, 0.0695, 0.0}},
	{3, {0.03, 0.079, 0.105, 0.0}},
	{4, {0.019, 0.0445, 0.0875, -0.785398}},
	{5, {0.033, 0.033, 0.0505, 0.0}},
	{6, {0.025, 0.0425, 0.0875, 0.785398}},
	// {7, {0.0425, 0.0425, 0.0165, 0.0}},
	{8, {0.055, 0.0175, 0.0445, 0.785398}},
	{9, {0.0425, 0.014, 0.0365, 0.785398}},
	{10, {0.025, 0.0485, 0.041, -0.785398}},
	// {11, {0.018, 0.095, 0.018, -0.523599}},
	{19, {0.054, 0.054, 0.1175, 0.0}},
	{21, {0.032, 0.049, 0.125, 0.785398}},
	// {24, {0.0795, 0.0795, 0.0265, 0.0}},
	{25, {0.041, 0.041, 0.04, 0.0}},
	// {35, {0.0175, 0.023, 0.092, 0.785398}},
	{36, {0.0425, 0.0425, 0.1, 0.0}},
};


const std::map<int, std::string> clutter::EXEC_TRAJ_FAIL = {
	{0, "no violation"},
	{1, "object toppled"},
	{2, "obstacle collision"},
	{3, "table collision"},
	{4, "velocity constraint violation"},
	{99, "grabbed wrong object"},
};

double clutter::DEG5 = 0.0872665;
