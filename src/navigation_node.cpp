/**
 * Navigation node: sends goal and waits the robot to reach them.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

#include <fstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define CSV_PATH "/waypoints.csv"

move_base_msgs::MoveBaseGoal createGoal(std::string pose) {
    std::istringstream iss(pose);
    std::string current;
    std::vector<double> doubleData;
    while (std::getline(iss, current, ','))
    {
        try
        {
            double val = std::stod(current);
            doubleData.push_back(val);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Invalid data in CSV file: discarded");
            continue;
        }
    }
    if (doubleData.size() != 3)
    {
        ROS_ERROR("Invalid line in CSV file");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = doubleData[0];
    goal.target_pose.pose.position.y = doubleData[1];
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(doubleData[2]);
    return goal;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    // Open the csv file with waypoints
    std::string csv_path = ros::package::getPath("second_project") + CSV_PATH;
    std::ifstream file(csv_path);
    if (!file.is_open())
    {
        ROS_ERROR("Cannot open %s.", csv_path.c_str());
        return 1;
    }
    
    std::string line;
    int count = 1;
    while (std::getline(file, line) && ros::ok())
    {
        ROS_INFO("Sending goal #%d...", count);
        ac.sendGoal(createGoal(line));
        ac.waitForResult();
        actionlib::SimpleClientGoalState result = ac.getState();
        ROS_INFO("Goal #%d: %s.", count, result.getText().c_str());
        count++;
    }

    return 0;
}