#include <ros/ros.h>
#include <go.hpp>

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "go_node");
    ROS_INFO("Main Node Started");

    GoNode go_node;

    go_node.run();

    return 0;
}