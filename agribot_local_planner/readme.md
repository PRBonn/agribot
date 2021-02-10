# AGRIBOT LOCAL PLANNER #

This package provide a simple implementation of a PID controller for robot
navigation. During robot navigation along a given path, this controller attempts
to stabilize both the linear and rotational velocity in an independent manner.
No velocity profile is computed before the robot starts moving. The environment
is considered static and, at the moment, no dynamics are taken into account.
This has been implemented as a ROS `move_base/base_local_planner` plugin.

## How to build ##

To build this package, just move it to your `catkin_ws` and build.

## How to run ##

The use of this package is constrained to the use of ROS `move_base` framework
for navigation. To use it as `base_local_planner`, add in your launcher in the
tag defining the `move_base` node the following line.
```
<param name="base_local_planner" value="agribot_local_planner/AgribotLocalPlanner" />
```
A simple tuning of the PID controller is provided. However, it may need to be
newly tuned accordingly to the robot and task taken into account. To this end,
controller parameters have been defined as dynamic parameters and can be tuned
using `dynamic_reconfigure`.

Enjoy!
