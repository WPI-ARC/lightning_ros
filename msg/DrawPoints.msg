Float64Array[] points
IntArray[] edges

#name for the group that is having points displayed (e.g. right_arm)
string model_group_name

#unique name for the group of points being displayed (e.g. rr_planner_tree_start)
string point_group_name

int32 point_type
int32 POINT_TYPE_POSES = 1
int32 POINT_TYPE_ANGLES = 2

#fraction of the path to be displayed
#e.g. if display_density == 0.25, then every 4th point will be displayed
float64 display_density

float64 point_radius
float64 red
float64 green
float64 blue

int32 action
int32 ACTION_ADD = 3
int32 ACTION_CLEAR = 4

