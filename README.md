# The Software Project for the Automated Travel Entity.

![alt text](automated.jpg)


## Milestone 1

Meeting this milestone means that:

- Your robot should be able to drive in the maze. Keyboard control is OK. This means that you can control the motors and the motion of the robot.
- Your robot should have a dead reckoning system (odometry) on the robot.
- You can use tf(ROS transform library) and demonstrate if to display the position of the robot and the current laser scan from the laser in rviz as the robot moves through the environment.

Your robot should now be able to move, use standard messages to visualise information in the ROS standard visualiser(rviz) and you have defined a proper transformation, using tf, between the robot frame and the laser.


## Milestone 2

In addition to MS1, meeting this milestone means that:

- Using the RGBD camera, your stationary robot can detect an object (of your choice) placed within the arm's reach, mark the pose with respect to the robot in rviz, and then pick it up (or are close to do so showing that you have a working closed loop).
- You can read the map file and turn that into your own map representation and display it along with the position of the robot.
- Your robot can receive a target pose and move to it and abort if close to an obstacle. You should be able to argue for how this will be used later.

Your robot should now also be able to use the camera to detect objects. You can also use your arm and you will have started realising that the arm is in the way when you move and when you are close to obstacles. When moving to a target position you will have realised that you need to pay attention to obstacles.

## Milestone 3

In addition to MS1 and MS2, meeting this milestone means that:

- You have a localisation system that can track the pose of the robot when it starts from a know position.
- You can command the robot to move to any position in the map. Path planning can be based on the given map and thus the robot may not succeed in reaching the position. If the path is free your robot should be able to reach the position.
- You can detect and identify at least one object and mark it in the map.
- You can pick up an object that was detected during path execution.
- You can record an evidence rosbag with camera images of the detected/identified objects and their positions following the specification in the wiki.
- Your robot speaks every time the robot detects an object and explain what it sees.

## Milestone 4

In addition to MS1, MS2 and MS3, meeting this milestone means that:

- You should be able to find a good path to the gate and get out, clearly showing that the robot is able to use information that was gathered via mapping the environment (rubble, objects, etc) when it plans the path.





