# BUG-0 PATHFINDING ALGORITHM
## REQUIREMENTS

Before running this algorithm, the full desktop version of ROS Noetic must be installed on a stable linux distribution.
Python 3 is also required. This software was designed and run on a machine running Debian Linux 10 (buster).

## HOW TO RUN

To run the bug-0 path-finding algorithm, first add the contents of the project zip directory to the src directory of your catkin workspace.

This will include:
```
bug0_alg : directory and its contents
stage    : directory and its contents
```

At this point ```catkin_make``` will need to be run in your catkin workspace to integrate the packages into your workspace.

Then open a terminal and startup roscore by navigating to the directory of your catkin workspace and running the following lines of code:
```
source ~/.bashrc
roscore
```

Next open a second terminal and startup ros stage by navigating to the stage directory added previously, and running the following line of code:
```
rosrun stage_ros stageros bug-test.world
```

If additional worlds wish to be used instead, they can be added to this directory and run accordingly. What is necessary is that a ros stage environment must be running and must be generating the following ros topics:
```
/base_pose_ground_truth
/base_scan
/cmd_vel
```
At this point, the robot should be positioned in stage at its intended start location and orientation.

Finally, open a third terminal and startup the bug-0 algorithm by navigating to the directory of your catkin workspace and running the following lines of code:
```
source devel/setup.bash
rosrun bug0_alg bug0_alg.py
```

The software will request a set of target coordinates; these must be in the format ```(target_x, target_y, target_theta)```. The parentheses and spaces are not required, but the commas are. 

<br>

## HOW IT WORKS
The bug0 algorithm uses the ```/base_pose_ground_truth``` ros topic within a minimal range and distance to look for obstacles, while travelling in the direction of the goal when it cannot find any obstacles in front of it. When it encounters an obstacle, it travels to the left around the obstacle until it cannot see any obstacles blocking the path directly to the goal. This application does this by moving away from obstacle when the front sensor is too close, towards the obstacle when a right sensor becomes too far away, and forward when the right sensor is in a sweet spot. The follow are constants that can be altered in the code as desired:
```
bug0_alg.py
    ALLOWABLE_GOAL_ERROR: how close the final (x,y,t) must be to input ones 
    WALL_BUFFER: how far away the bot will be from obstacles
    ALLOWABLE_WALL_DRIFT: how much the bot can drift from the WALL_BUFFER
    FAR_ENUF_AWAY: how far the bot "see" obstacles
    USE_TARGET_THETA: changes to only need target (x,y) when false
bug_sensor.py
    READING_SPREAD: number of points the sensors read from
    FRONT_ANGLE: angle of front sensor (0 is straight ahead, [-3/4pi:3/4pi])
    RIGHT_ANGLE: angle of right sensor (0 is straight ahead, [-3/4pi:3/4pi])
bug_position.py
    THETA_OFFSET: how close angles need to be their intended values
```

<br>

## KNOWN BUGS
The current iteration generates some error in the final target x and y coordinates when rotating to the target theta orientation. This is because the robot moves forward while rotating, a decision made both because the actual robot moves in this way and to avoid a common corner case causing the robot to rotate back and forth indefinitely upon reaching an obstacle. As a result, the target theta can be disabled if more accuracy in the final x and y coordinates is required. This is done by changing the constant USE_TARGET_THETA in the main bug0_alg.py file to False and running as normal. 