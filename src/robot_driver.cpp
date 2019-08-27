#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>


class RobotDriver
{
private:
    ros::NodeHandle n;

    // Publishers
    ros::Publisher odom_pub;
    ros::Publisher left_ir_pub;
    ros::Publisher front_ir_pub;
    ros::Publisher right_ir_pub;

    // Broadcasters
    tf::TransformBroadcaster odom_broadcaster;

    // Subscribers
    ros::Subscriber pose_sub; // Get pose of robot
    ros::Subscriber cmd_vel_sub; // Get velocity of robot
    ros::Subscriber ir_left_sub;
    ros::Subscriber ir_front_sub;
    ros::Subscriber ir_right_sub;

    // Velocity variables received from cmd_vel topic and published to odom
    float linear_vel, angular_vel;

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        // Save pose, convert and publish to /odom topic
        double x = msg->x;
        double y = msg->y;
        double th = msg->theta;
        ros::Time current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        this->odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = this->linear_vel;
        odom.twist.twist.linear.y = 0; // differential robot
        odom.twist.twist.angular.z = this->angular_vel;

        //publish the message
        this->odom_pub.publish(odom);
    }

    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // Receive speed of robot
        this->linear_vel = msg->linear.x;
        this->angular_vel = msg->angular.z;
    }


    void leftIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Publish left distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "left_ir";
        ir_msg.radiation_type = 1;                    
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = msg->data;;
        this->left_ir_pub.publish(ir_msg); 
    }
    void frontIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Publish front distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "front_ir";
        ir_msg.radiation_type = 1;                    
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = msg->data;;
        this->front_ir_pub.publish(ir_msg); 
    }
    void rightIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Publish front distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "right_ir";
        ir_msg.radiation_type = 1;                   
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = msg->data;;
        this->right_ir_pub.publish(ir_msg); 
    }

public:
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Setup publishers
        this->odom_pub = this->n.advertise<nav_msgs::Odometry>("odom", 10);
        this->left_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);
        this->front_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_front_sensor", 10);
        this->right_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_right_sensor", 10);

        // Setup subscribers
        this->pose_sub = this->n.subscribe("pose", 10, &RobotDriver::poseCallback, this);
        this->cmd_vel_sub = this->n.subscribe("cmd_vel", 10, &RobotDriver::velocityCallback, this);
        this->ir_left_sub = this->n.subscribe("left_distance", 10, &RobotDriver::leftIRCallback, this);
        this->ir_front_sub = this->n.subscribe("front_distance", 10, &RobotDriver::frontIRCallback, this);
        this->ir_right_sub = this->n.subscribe("right_distance", 10, &RobotDriver::rightIRCallback, this);

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

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "robot_driver");

    // Create our controller object and run it
    auto controller = RobotDriver();
    controller.run();

    // And make good on our promise
    return 0;
}