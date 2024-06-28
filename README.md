# autonomous_mobile_robots
With Webot "epuck" robot perfomed the following tasks.

**SLAM:-** <br/>
* Devloped an Simultaneous localization and mapping algorithm in which the e-puck robot maps dynamically the complete internal wall configuration of the maze. <br/>
* After mapping is completed, the maze configuration is sotred in the graph data structure and then, the traversal is performed to navigate to goal from any start location. <br/>

**PID_WALL_FOLLOWING**

* Developed a PID motion controller where a proportional gain  _kp_  is applied to the front sensor, left distance sensor, and right distance sensor. The robot starts 20 inches away from the front wall and stops at a 5-inch distance using the front sensor. Additionally, the controller was tested with Lidar, using a single _kp_   value for all Lidar PID computations. (source code can be found in pid_wall_following/pid_motion.py)
* Implemented a **wall-following** PID controller using the best _kp_ value identified in Task 1. The robot follows the wall, making 90-degree turns when it reaches an end wall and 180-degree turns if no 90-degree turns are possible. The robot can start at any grid cell with any orientation and follows either the left or right wall as instructed. The performance was tested in two different maze configurations. Additionally, both distance sensors and Lidars were used for comparison. (source code can be found in pid_wall_following/pid_wall_following.py)



**CAMERA_NAVIGATION**


* Developed a program that enables the robot to move towards a yellow-colored cylinder using camera-based detection, stopping at 5 inches from the goal, employing a PID controller to adjust its movement based on the distance and orientation of the detected yellow blob.
* Implemented the **Bug Zero algorithm** for the robot to navigate towards the same goal while avoiding walls, using the camera to compute distances and orientations, switching between moving towards the goal and wall-following behaviors, ensuring the robot stops within 5 inches of the goal.