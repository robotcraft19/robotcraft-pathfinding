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
#include "ros/ros.h"
#include <ros/console.h>

class MapSaver {

private:
	ros::NodeHandle n;

	int loop_counter;

	void saveMap() {
		/* Runs map_saver node in map_server package to save 
		*  occupancy grid from /map topic as image */

		// Save as map_backup.pgm
		system("roscd robotcraft_maze && rosrun map_server map_saver -f map_backup");
		ROS_WARN_STREAM("Saving image of map as map_backup.pgm in " << "~/catkin_ws/src/robotcraft_maze/scans");

	}

public:

    MapSaver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

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
        	}

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