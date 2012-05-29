#list of points for drawing
Float64Array[] points
#list of edges to display: edges[i] is the list of indicies of points that point i is connected to
IntArray[] edges

#name for the group that is having points displayed (e.g. right_arm)
string model_group_name

#unique name for the group of points being displayed (e.g. rr_planner_tree_start)
string point_group_name

#type of point that is sent (poses: xyz coordinates; angles: joint configurations that will be transformed into poses)
int32 point_type
int32 POINT_TYPE_POSES = 1
int32 POINT_TYPE_ANGLES = 2

#type of action (add: draw the points, clear: clear all points)
int32 action
int32 ACTION_ADD = 3
int32 ACTION_CLEAR = 4

#fraction of the path to be displayed
#e.g. if display_density == 0.25, then every 4th point will be displayed
float64 display_density

#radius of the points to draw
float64 point_radius

#specifies color for points
float64 red
float64 green
float64 blue

