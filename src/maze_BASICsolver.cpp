/**
 * @file maze_BASICsolver.cpp
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
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

#define TARGET_DISTANCE 0.20

class BasicSolver {

private:

    // Node handle
    ros::NodeHandle n;

    // Publishers
    ros::Publisher cmd_vel_pub;

    // Subscribers
    ros::Subscriber front_ir_sub;
    ros::Subscriber left_ir_sub;
    ros::Subscriber right_ir_sub;

    // External Parameters
    bool left;
    bool right;

    // Global variables
    float front_distance;
    float left_distance;
    float right_distance;

    // PID control
    float old_prop_error;
    float integral_error;
    
    float KP = 10;
    float KI = 0.0;
    float KD = 0.0;
    float time_interval = 0.1;

    // Helper variables
    bool robot_lost;
    int lost_counter;



    geometry_msgs::Twist calculateCommand(){
    
    // Check if robot is lost (after 75 loops without sensing any wall)
    calculateRobotLost();

	// Create message
	auto msg = geometry_msgs::Twist();
    if (right) 
    {
        if (front_distance < TARGET_DISTANCE)
        {
            // Prevent robot from crashing
            msg.angular.z = 1.25; // maximum angular speed
            msg.linear.x = -0.04;
        }
        else if (robot_lost == true)
        {
            // Robot is lost, go straight to find wall
            msg.linear.x = 0.08;
        } 
        else
        {
            // Robot keeps using normal PID controller
            float gain = calculateGain(right_distance);
            msg.linear.x = 0.08;
            msg.angular.z = gain;
        }
    }

    else if (left) 
    {
        if (front_distance < TARGET_DISTANCE)
        {
            // Prevent robot from crashing
            msg.angular.z = -1.25; // maximum angular speed
            msg.linear.x = -0.04;
        }
        else if (robot_lost == true)
        {
            // Robot is lost, go straight to find wall
            msg.linear.x = 0.08;
        } 
        else
        {
            // Robot keeps using normal PID controller
            float gain = calculateGain(left_distance);
            msg.linear.x = 0.08;
            msg.angular.z = gain;
        }
    }

	return msg;

    }
    

    void frontIRCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    	// Extract range, first (and only) element of array
        this->front_distance = msg->ranges[0]; 
    }
    void leftIRCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        // Extract range, first (and only) element of array
    	this->left_distance = msg->ranges[0]; 
    }
    void rightIRCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        // Extract range, first (and only) element of array
    	this->right_distance = msg->ranges[0]; 
    }


    float calculateGain(float value) 
	{	
        // Calculate errors
	    float error = TARGET_DISTANCE - value;
	    float new_der_err = error - this->old_prop_error;
	    float new_int_err = this->integral_error + error;

        // Calculate gain
	    float gain = this->KP*error + this->KI*new_int_err*this->time_interval
	                 + this->KD*new_der_err/this->time_interval;

        // Update old errors
	    this->old_prop_error = error;
	    this->integral_error = new_int_err;  
	    
        // Restrict gain to prevent overshooting on sharp corners
        if(left)
            gain = -gain;
            if(gain > 0.4)
                gain = 0.4;

            
	    if(right)
            if(gain < -0.4) gain = -0.4;


	    return gain;
	}

	void calculateRobotLost() 
	{
        if (right)
        {
            // Calculations needed to check if robot is lost
            if (front_distance > TARGET_DISTANCE && right_distance > TARGET_DISTANCE 
                && left_distance > TARGET_DISTANCE) 
            {
                ++lost_counter;

                // π / 0.4 ≈ 8.0, after 80 loops robot has made at least half a rotation
                if (lost_counter >= 100) {
                    robot_lost = true;
                    ROS_WARN("ROBOT LOST! SEARCHING WALL...");
                }
            } 
            else if(front_distance < TARGET_DISTANCE || right_distance < TARGET_DISTANCE) 
            {
                robot_lost = false;
                lost_counter = 0;
            }
        }
        else if (left)
        {
            // Calculations needed to check if robot is lost
            if (front_distance > TARGET_DISTANCE && right_distance > TARGET_DISTANCE 
                && left_distance > TARGET_DISTANCE) 
            {
                ++lost_counter;

                // π / 0.4 ≈ 8.0, after 80 loops robot has made at least half a rotation
                if (lost_counter >= 100) {
                    robot_lost = true;
                    ROS_WARN("ROBOT LOST! SEARCHING WALL...");
                }
            } 
            else if(front_distance < TARGET_DISTANCE || left_distance < TARGET_DISTANCE) 
            {
                robot_lost = false;
                lost_counter = 0;
                // ROS_WARN("WALL FOUND");
            }
        }
	}


public:

    BasicSolver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        n.getParam("left", this->left);
        n.getParam("right", this->right);

        ROS_INFO("Right = %d\n", this->right);
        ROS_INFO("Left = %d\n", this->left);

        if (left && right) 
        {
            if (rand() > 0.5) 
                left = false;
            else
                right = false;
        }

        if (!left && !right) 
        {
            if (rand() > 0.5) 
                left = true;
            else
                right = true;
        }

        ROS_INFO("Right = %d\n", right);
        ROS_INFO("Left = %d\n", left);

        // Setup publishers
    	this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Setup subscribers
        this->front_ir_sub = this->n.subscribe("base_scan_1", 10, &BasicSolver::frontIRCallback, this);
        this->left_ir_sub = this->n.subscribe("base_scan_2", 10, &BasicSolver::leftIRCallback, this);
        this->right_ir_sub = this->n.subscribe("base_scan_3", 10, &BasicSolver::rightIRCallback, this);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
        	// Calculate the command to apply
        	auto msg = calculateCommand();

        	 // Publish the new command
        	this->cmd_vel_pub.publish(msg);

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