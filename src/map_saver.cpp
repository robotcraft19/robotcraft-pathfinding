/**
 * @file map_saver.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2019-08-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

class MapSaver {

private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;

	float robot_x, robot_y;

	int loop_counter;

	void saveMap() {
		/* Runs map_saver node in map_server package to save
		*  occupancy grid from /map topic as image */

		// Save as map_backup.pgm
		system("cd ~/catkin_ws/src/robotcraft_maze/scans && rosrun map_server map_saver -f map_backup");
	}

	void saveRobotPose() {
		/* Saves the robot's latest pose which can be used
		*  for target position calculation */
		const char* homeDir = getenv("HOME");
		std::string file(homeDir);
		file.append("/catkin_ws/src/robotcraft_maze/scans/rob_pos_backup.txt");

		std::ofstream position_file;
		position_file.open (file);
		position_file << this->robot_x << "\n" << this->robot_y << "\n";
		position_file.close();

	}

	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
	{
    this->robot_x = msg->pose.pose.position.x;
    this->robot_y = msg->pose.pose.position.y;
	}

public:

    MapSaver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

				// Add odom subscriber
				this->odom_sub = this->n.subscribe("odom", 5, &MapSaver::odomCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
        		// Increase loop counter
        		loop_counter++;

	        	// Save map every 100 loops
	        	if(loop_counter % 100 == 0) {
	        	saveMap();
						saveRobotPose();
						ROS_WARN_STREAM("Saving image of map as map_backup.pgm"
						<< " and saving robot's pose as rob_pos_backup.txt"
					 	<< " in ~/catkin_ws/src/robotcraft_maze/scans");
	        	}

						// Receive messages
						ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }
};



int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "map_saver");

    // Create our controller object and run it
    auto controller = MapSaver();
    controller.run();

    // And make good on our promise
    return 0;
}
