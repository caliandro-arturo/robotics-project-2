/**
 * TF publisher
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#define SUB_TOPIC "/t265/odom"


// Handle the subscription and the TF publishing.
class OdometricPubSub
{
public:
    OdometricPubSub()
    {
        sub = n.subscribe(SUB_TOPIC, 1000, &OdometricPubSub::callback, this);
    }

    // Send the odometry information to the tf transform broadcaster.
    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Transform transform;
        tf::Vector3 v;
        tf::pointMsgToTF(msg->pose.pose.position, v);
        transform.setOrigin(v);
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "t265"));
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformBroadcaster br;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ROS_INFO("Node started.");
    OdometricPubSub odom;
    ros::spin();
    return 0;
}