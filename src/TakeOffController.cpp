#include "takeoff_sphinx_test/TakeOffController.hpp"
#include <std_msgs/Empty.h>
#include <thread>
#include <chrono>

namespace takeoff_sphinx_test {
    TakeOffController::TakeOffController(ros::NodeHandle& nodeHandle) :
        nodeHandle_(nodeHandle)
    {
        ROS_INFO_STREAM("in takeoff_sphinx_constructor");
        nodeHandle.getParam("/takeoff_sphinx_test/sphinx/takeoff", takeoff_name);
        nodeHandle.getParam("/takeoff_sphinx_test/sphinx/land", land_name);
        ROS_INFO_STREAM(takeoff_name << " " << land_name);
        takeoff_talker = nodeHandle.advertise<std_msgs::Empty>(takeoff_name, 1000);
        land_talker = nodeHandle.advertise<std_msgs::Empty>(land_name, 1000);
        std_msgs::Empty msg;
        takeoff_talker.publish(msg);
        std::this_thread::sleep_for (std::chrono::seconds(3));
        land_talker.publish(msg);
    }

    TakeOffController::~TakeOffController()
    {
    }
}