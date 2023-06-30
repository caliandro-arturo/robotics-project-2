/*
 *  Limits finder
 *  
 *  Node meant to find the maximum values of instant and acceleration.
 */

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

#define SUB_TOPIC "/t265/odom"
#define DT 2e-1

/*
 *  The results are sent using the nav_msgs::Odometry message type, in
 *  the following configuration:
 *  - maximum forward speed:            position.x
 *  - maximum backward speed:           position.y
 *  - maximum angular speed:            position.z
 *  - maximum forward acceleration:     orientation.x
 *  - maximum backward acceleration:    orientation.y
 *  - maximum angular acceleration:     orientation.z
 *  Other fields are used to keep track of real-time speed/acceleration values.
 */
class KinematicState
{
private:
    bool isInit = false;
    double x = 0;
    double y = 0;
    double yaw = 0;
    double actual_vel_linear = 0;
    double actual_acc_linear = 0;
    double actual_vel_angular = 0;
    double actual_acc_angular = 0;
    ros::Time last_timedelta = ros::Time::now();
public:
    double vel_linear_forward = 0;
    double vel_linear_backward = 0;
    double acc_linear_forward = 0;
    double acc_linear_backward = 0;
    double vel_angular = 0;
    double acc_angular = 0;

    nav_msgs::Odometry newData(const nav_msgs::Odometry &msg)
    {
        nav_msgs::Odometry new_msg;
        if (!isInit)
        {
            init(msg);
            return new_msg;
        }
        double dt = (msg.header.stamp - last_timedelta).toSec();
        // Further discretization to reduce noise
        if (dt >= DT) {
            double new_x = msg.pose.pose.position.x - std::cos(yaw)*0.25;
            double new_y = msg.pose.pose.position.y - std::sin(yaw)*0.25;
            double distance = std::sqrt(std::pow(new_x - x, 2) + std::pow(new_y - y, 2));
            double new_vel_linear = distance / dt;
            actual_acc_linear = (new_vel_linear - actual_vel_linear) / dt;
            bool isForward = (new_x - x)*cos(yaw) > 0 && (new_y - y)*sin(yaw) > 0;
            if (isForward)
            {
                vel_linear_forward = std::max(vel_linear_forward, new_vel_linear);
                acc_linear_forward = std::max(acc_linear_forward, actual_acc_linear);
            }
            else
            {
                vel_linear_backward = std::max(vel_linear_backward, new_vel_linear);
                acc_linear_backward = std::max(acc_linear_backward, actual_acc_linear);
            }
            actual_vel_linear = new_vel_linear;

            double new_yaw = tf::getYaw(msg.pose.pose.orientation);
            double deltaYaw = new_yaw - yaw;
            if (deltaYaw > M_PI)
                deltaYaw -= 2*M_PI;
            else if (deltaYaw < -M_PI)
                deltaYaw += 2*M_PI;
            double new_vel_angular = (deltaYaw) / dt;
            actual_acc_angular = (new_vel_angular - actual_vel_angular) / dt;
            vel_angular = std::max(vel_angular, std::abs(new_vel_angular));
            acc_angular = std::max(acc_angular, std::abs(actual_acc_angular));
            actual_vel_angular = new_vel_angular;

            x = new_x;
            y = new_y;
            yaw = new_yaw;
            last_timedelta = msg.header.stamp;
        }
        new_msg.pose.pose.position.x = vel_linear_forward;
        new_msg.pose.pose.position.y = vel_linear_backward;
        new_msg.pose.pose.position.z = vel_angular;
        new_msg.pose.pose.orientation.x = acc_linear_forward;
        new_msg.pose.pose.orientation.y = acc_linear_backward;
        new_msg.pose.pose.orientation.z = acc_angular;
        new_msg.twist.twist.linear.x = actual_vel_linear;
        new_msg.twist.twist.linear.y = actual_acc_linear;
        new_msg.twist.twist.linear.z = actual_vel_angular;
        new_msg.twist.twist.angular.x = actual_acc_angular;

        return new_msg;
    }

    // Set initial values.
    void init(const nav_msgs::Odometry &msg)
    {
        yaw = tf::getYaw(msg.pose.pose.orientation);
        x = msg.pose.pose.position.x - cos(yaw)*0.25;
        y = msg.pose.pose.position.y - sin(yaw)*0.25;
        last_timedelta = msg.header.stamp;
        isInit = true;
    }
};

// Handle the PubSub mechanism
class LimitsPubSub
{
public:
    LimitsPubSub()
    {
        limits = n.advertise<nav_msgs::Odometry>("limits", 1, true);
        sub = n.subscribe(SUB_TOPIC, 1, &LimitsPubSub::callback, this);
    }

    void callback(const nav_msgs::Odometry &msg)
    {
        limits.publish(state.newData(msg));
    }

private:
    ros::NodeHandle n;
    ros::Publisher limits;
    ros::Subscriber sub;
    KinematicState state;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "limits_finder");
    ROS_INFO("Node started.");
    LimitsPubSub limitsPubSub;
    ros::spin();
    return 0;
}