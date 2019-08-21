#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


class BasicSolver {

private:

    ros::NodeHandle n;

    // Publishers
    //ros::Publisher odom_pub;

    // Subscribers
    //ros::Subscriber pose_sub; 

public:

    BasicSolver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Setup publishers
        // this->left_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);

        // Setup subscribers
        //this->pose_sub = this->n.subscribe("pose", 10, &RobotDriver::poseCallback, this);
        ROS_INFO("RUNNING NODE");

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Receive messages and publish inside callbacks
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }
};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "maze_basic_solver");

    // Create our controller object and run it
    auto controller = BasicSolver();
    controller.run();

    // And make good on our promise
    return 0;
}