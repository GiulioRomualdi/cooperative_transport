Cooperative transport with iRobot Create 2.
===
Ros node implementation.

Prerequisites
---
* [irobotcreate2](https://github.com/MirkoFerrati/irobotcreate2ros.git)

Instructions
---
Just clone the repository in the src folder of a catkin workspace. Then run catkin_make.

Usage
---
To launch the simulation with one robot type:
```
roslaunch cooperative_transport sim_one_robot.launch
```

To launch the simulation with multiple robots type:
```
roslaunch cooperative_transport sim_multiple_robots.launch
```

These launch files start gazebo, the main controller (for each simulated robot) and a node that simulates the ir light bumper sensors of the irobot using the Hoyouko laser range finder data provided by gazebo.

The ir readings are available in the topics /ir_bumper_x where x identifies the robot.

To launch the controller alone, i.e., without gazebo and ir simulator, type:
```
roslaunch cooperative_transport controller.launch
```


