# robotcraft_maze

This repository is a ROS package that can be used to solve the basic and pro maze which are part of the competition in the RobotCraft 2019 summer programme.

#### Basic Maze
With the help of three infrared sensor, the robot tries to find the exit of the maze. At the same time, it uses its LIDAR to map the maze. However, it is not allowed to use the LIDAR for distance measurements or localisation in this part of the competition.

#### Pro Maze
With the help of the previously mapped maze, the robot now calculates the shortest path out of the maze and tries to follow this path until it reaches the exit. Therefore, it strongly relies on its odometry calculations which come from LIDAR and motor encoder data readings.
