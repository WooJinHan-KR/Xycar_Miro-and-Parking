#include <ros/ros.h>
#include <MazeEscape.hpp>

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "MazeEscaper_node");
    ROS_INFO("Main Node Started");

    MazeEscaper MazeEscaper_node;

    MazeEscaper_node.run();

    return 0;
}