/**
 * Navigation
 */

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define CSV_PATH "waypoints.csv"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;

    //MOVEBASE
    MoveBaseClient mb("move_base", true);
    mb.waitForServer();

    //CSV READER
    std::string csv = CSV_PATH;
    std::ifstream file(csv);

    if (!file.is_open())
    {
        ROS_ERROR("Error in file opening");
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string current;
        std::vector<double> doubleData;

        while (std::getline(iss, current, ','))
        {
            try
            {
                double val = std::stod(current);
                doubleData.push_back(val);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Invalid data in CSV file: %s", e.what());
            }
        }
        if (doubleData.size() != 3)
        {
            ROS_ERROR("Invalid line in CSV file");
            continue;
        }

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";

        goal.target_pose.pose.position.x = doubleData[0];
        goal.target_pose.pose.position.y = doubleData[1];
        goal.target_pose.pose.orientation.z = sin(doubleData[2] / 2.0);

        mb.sendGoal(goal);
        mb.waitForResult();

        //ros spin once e ok all'interno del while

    }

    file.close();

}