cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_basic src/maze_BASICsolver.cpp)
target_link_libraries(maze_basic ${catkin_LIBRARIES})

#add_executable(maze_pro src/maze_PROsolver.cpp) # or .py?
#target_link_libraries(maze_pro ${catkin_LIBRARIES})

add_executable(map_saver src/map_saver.cpp)
target_link_libraries(map_saver ${catkin_LIBRARIES})

add_executable(robot_driver src/robot_driver.cpp)
target_link_libraries(robot_driver ${catkin_LIBRARIES})
