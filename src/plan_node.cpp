//
//

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>

ros::Publisher path_pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_planner_node");

    ros::NodeHandle nh;
    path_pub = nh.advertise<nav_msgs::Path>("custom_planner/path", 1);

    // Crea un'istanza del planner A*
    global_planner::PlannerCore planner;

    // Crea un'istanza del messaggio di percorso
    nav_msgs::Path path;
    path.header.frame_id = "map";

    // Coordinate del punto di partenza
    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.orientation.w = 1.0;

    // Coordinate del punto di arrivo
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = 1.0;
    goal_pose.pose.position.y = 1.0;
    goal_pose.pose.orientation.w = 1.0;

    // Genera il percorso utilizzando l'algoritmo A*
    bool success = planner.makePlan(start_pose, goal_pose, path);

    if (success) {
        // Pubblica il percorso
        path_pub.publish(path);
    } else {
        ROS_ERROR("Impossibile pianificare il percorso.");
    }


    return 0;
}
