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
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <ros/console.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include <ctime>

#define TARGET_DISTANCE 0.17

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
    ros::Subscriber odom_sub;

    // Services
    ros::ServiceServer  basic_serv;

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
    int lost_counter = 0;
    bool robot_stop;
    float robot_x, robot_y;



    geometry_msgs::Twist calculateCommand(){
    	// Create message
    	auto msg = geometry_msgs::Twist();

      if(!this->robot_stop) {
        // Check if robot is lost (after 75 loops without sensing any wall)
        calculateRobotLost();

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
      }

      else {
        // Stop robot (set velocities to 0)
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
      }

    	return msg;

    }


    void frontIRCallback(const sensor_msgs::Range& msg){
    	// Extract range, first (and only) element of array
        this->front_distance = msg.range;
    }
    void leftIRCallback(const sensor_msgs::Range& msg){
        // Extract range, first (and only) element of array
    	this->left_distance = msg.range;
    }
    void rightIRCallback(const sensor_msgs::Range& msg){
        // Extract range, first (and only) element of array
    	this->right_distance = msg.range;
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
                if (lost_counter >= 200) {
                    robot_lost = true;
                    // ROS_WARN("ROBOT LOST! SEARCHING WALL...");
                }
            }
            else if((front_distance < TARGET_DISTANCE || right_distance < TARGET_DISTANCE) && (right_distance > 0.07) && (front_distance > 0.07))
            {
                robot_lost = false;
                lost_counter = 0;
            }
        }
        else if (left)
        {
            // Calculations needed to100 check if robot is lost
            if (front_distance > TARGET_DISTANCE && right_distance > TARGET_DISTANCE
                && left_distance > TARGET_DISTANCE)
            {
                ++lost_counter;

                // π / 0.4 ≈ 8.0, after 80 loops robot has made at least half a rotation
                if (lost_counter >= 200) {
                    robot_lost = true;
                    // ROS_WARN("ROBOT LOST! SEARCHING WALL...");
                }
            }
            else if((front_distance < TARGET_DISTANCE || left_distance < TARGET_DISTANCE) && (left_distance > 0.07) && (front_distance > 0.07))
            {
                robot_lost = false;
                lost_counter = 0;
            }
        }
	}

  bool basicServiceCallback(std_srvs::Empty::Request  &req,
           std_srvs::Empty::Response &res)
  {
    ROS_INFO("Request to stop robot and save map and position received...");
    this->robot_stop = true;
    saveMap();
    saveRobotPose();
    return true;
  }

  void saveMap() {
    /* Runs map_saver node in map_server package to save
    *  occupancy grid from /map topic as image */

    // Save as map.pgm
    system("cd ~/catkin_ws/src/robotcraft_maze/scans && rosrun map_server map_saver -f map");
  }

  void saveRobotPose() {
    /* Saves the robot's latest pose which can be used
    *  for target position calculation */
    const char* homeDir = getenv("HOME");
    std::string file(homeDir);
    file.append("/catkin_ws/src/robotcraft_maze/scans/robot_position.txt");

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

    BasicSolver(){
        // Initialize ROS
        this->n = ros::NodeHandle();
	      srand(time(NULL));

        n.getParam("left", this->left);
        n.getParam("right", this->right);
        n.getParam("robot_lost", this->robot_lost);

        int rnd = rand() % 100;

        if (left && right)
        {
            if (rnd > 50)
                left = false;
            else
                right = false;
        }

        if (!left && !right)
        {
            if (rnd > 50)
                left = true;
            else
                right = true;
        }

        ROS_INFO("Right = %d\n", right);
        ROS_INFO("Left = %d\n", left);
        ROS_INFO("Robot is LOST = %d\n", robot_lost);

        // Setup publishers
    	  this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Setup subscribers
        this->front_ir_sub = this->n.subscribe("ir_front_sensor", 5, &BasicSolver::frontIRCallback, this);
        this->left_ir_sub = this->n.subscribe("ir_left_sensor", 5, &BasicSolver::leftIRCallback, this);
        this->right_ir_sub = this->n.subscribe("ir_right_sensor", 5, &BasicSolver::rightIRCallback, this);
        this->odom_sub = this->n.subscribe("odom", 5, &BasicSolver::odomCallback, this);

        // Setup services
        this->basic_serv = n.advertiseService("stop_save", &BasicSolver::basicServiceCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(8); // Prevent lost sync error
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
    sleep(15); // give LIDAR time to start spinning
    controller.run();

    // And make good on our promise
    return 0;
}
