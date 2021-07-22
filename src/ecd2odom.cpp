/**
 * Reference: github.com/RBinsonB/Nox_robot
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

double radius = 0.0625; // wheel radius in m
double wheelbase = 0.4; // distance between two wheels in m

double motor1_speed = 0.0, motor2_speed = 0.0, speed_time_interval = 0.0;
ros::Time motor_speed_timestamp(0.0), current_time;

double x_pos = 0.0, y_pos = 0.0, theta = 0.0;
double vx = 0.0, vy = 0.0, vth = 0.0;

void handle_ecd(const geometry_msgs::Vector3Stamped& speed_from_ecd)
{
    motor1_speed = speed_from_ecd.vector.x;
    motor2_speed = speed_from_ecd.vector.y;
    speed_time_interval = speed_from_ecd.vector.z;
    motor_speed_timestamp = speed_from_ecd.header.stamp;
    ROS_INFO("motor1 speed: %f m/s", motor1_speed);
    ROS_INFO("motor2 speed: %f m/s", motor2_speed);
    ROS_INFO("speed time interval: %f ms", speed_time_interval);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ziirobot_ecd2odom");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("robot/lin_spd_ecd", 50, handle_ecd);
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("robot/odom0", 50);

    double rate = 20.0;
    ros::Rate r(rate);

    double dt = 0.0;    // time interval
    double dxy = 0.0;   // linear distance displacement
    double dth = 0.0;   // angular displacement
    double dx = 0.0, dy = 0.0;  // linear distance displacement in x, y direction respectively.

    char odom[] = "odom";
    char base_link[] = "base_link";

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.covariance[0] = 1e-9;
    odom_msg.pose.covariance[7] = 1e-9;
    odom_msg.pose.covariance[14] = 1e-9;
    odom_msg.pose.covariance[21] = 1e-9;
    odom_msg.pose.covariance[28] = 1e-9;
    odom_msg.pose.covariance[35] = 1e-9;
    odom_msg.twist.covariance[0] = 1e-9;
    odom_msg.twist.covariance[7] = 1e-9;
    odom_msg.twist.covariance[14] = 1e-9;
    odom_msg.twist.covariance[21] = 1e-9;
    odom_msg.twist.covariance[28] = 1e-9;
    odom_msg.twist.covariance[35] = 1e-9;

    while (nh.ok())
    {
        current_time = motor_speed_timestamp;
        dt = speed_time_interval / 1000;    // unit: s
        dxy = (motor1_speed + motor2_speed) * dt / 2;   // unit: m
        dth = (motor2_speed - motor1_speed) / wheelbase * dt;   // unit: rad
        dx = cos(dth) * dxy;
        dy = sin(dth) * dxy;

        x_pos += (cos(theta) * dx - sin(theta) * dy);
        y_pos += (sin(theta) * dx + cos(theta) * dy);
        theta += dth;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        vx = (dt == 0) ? 0 : (motor1_speed + motor2_speed) / 2;
        vth = (dt == 0) ? 0 : (motor2_speed - motor1_speed) / wheelbase;
        odom_msg.child_frame_id = base_link;
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = vth;

        pub.publish(odom_msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}