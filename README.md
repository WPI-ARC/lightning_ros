# LightningROS
LightningROS is planner implementing the planner described at
http://arc.wpi.edu/download.php?p=9 (see "Combined Learning and Manipulation
Planning" under the [projects tab](http://arc.wpi.edu/#projects-1) at the WPI
ARC Lab website).

This project was originally worked on at a repository over in
[SourceForge](http://sourceforge.net/p/lightningros/wiki/Home/), but it never
got updated beyond working with ROS Fuerte. This version has been updated to
run with ROS Indigo and MoveIt.

No major changes were made to the actual functionality and algorithms that the
LightningROS library uses, so much of the information available on the original
SourceForge documentation is still relevant, although it should be taken with a
grain of salt.

Further documentation beyond what is shown here can be found on [the wiki](https://github.com/WPI-ARC/lightning_ros/wiki)
for this github repository.

##Setup the package
1. Clone the repository into a `lightning` directory in your ROS Workspace.
2. Run `catkin_make --pkg lightning` to build lightning.

##Run lightning tests

To run tests, make sure the ROBOT environment variable is set to "sim".

1. Specify the number of test iterations with the `num_test_iterations`
argument and the arm to use (`right` or `left`) with the `arm_to_use` argument
in the `*_test.launch file`
2. Run `roslaunch lightning table_test.launch` or `roslaunch lightning box_test.launch`
  1. `table_test`: The scene is a tabletop with four boxes on it. The positions
     of the boxes are randomly chosen and a goal position is chosen around the
     top of the table.
  2. `box_test`: The scene is a tabletop with a large box on it.
       The box opening is facing towards the robot, and the goal position is
       chosen to be inside the box.
3. Lightning will store paths in the library for the given scene.

##Run lightning:
1. To start lightning, run `roslaunch lightning lightning.launch`.
2. To do planning, send a GetMotionPlanRequest to the `lightning_get_path`
service. Lightning does not currently handle path constraints.
It only plans for a start and a goal.

##Path library management:

There are three management actions for the path library: storing a path,
deleting a path, and deleting a library.

1. storing a path: Send the path as an array of JointTrajectoryPoints. Also,
  specify the robot name and the joint names, as those define which library to
  use. If the library for the given robot name and joint names does not exist, a
  new one will be created.
2. deleting a path: Send the path id of the path to delete. Again, specify the
robot name and the joint names corresponding to the library that is to be changed.
3. deleting a library: Send the robot name and joint names corresponding to the
library that is to be deleted.
