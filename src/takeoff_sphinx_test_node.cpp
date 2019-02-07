#include <ros/ros.h>
#include <string>
#include <std_msgs/Empty.h>
#include <thread>
#include <chrono>
#include "takeoff_sphinx_test/TakeOffController.hpp"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


using std::string;

void odomCallback(nav_msgs::Odometry msg)
{
    // ROS_INFO("the odometry readings is [%s]", msg -> );
    geometry_msgs::PoseWithCovariance poseCov = msg.pose;
    geometry_msgs::Pose pose = poseCov.pose;
    geometry_msgs::Point point = pose.position;
    double x = point.x, y = point.y, z = point.z;
    ROS_INFO("the odometry reading for coordinate is x -> [%lf], y -> [%lf], z -> [%lf]", x, y, z);
    // ROS_INFO(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "takeoff_sphinx_test");
    ros::NodeHandle nodeHandle;
    ROS_INFO_STREAM("started node handler in takeoff \n");

    // ros::NodeHandle nodeHandle_;
    string takeoff_name;
    string land_name;
    ros::Publisher takeoff_talker;
    ros::Publisher land_talker;
    std_msgs::Empty msg_takeoff, msg_land;
    ros::Subscriber odom_sub;
    int spinCount = 0;


    ROS_INFO_STREAM("in takeoff_sphinx_constructor");
    //nodeHandle.getParam("/takeoff_sphinx_test/sphinx/takeoff", takeoff_name);
    //nodeHandle.getParam("/takeoff_sphinx_test/sphinx/land", land_name);
    //ROS_INFO_STREAM(takeoff_name << " " << land_name);
    takeoff_talker = nodeHandle.advertise<std_msgs::Empty>("/bebop/takeoff", 1000);
    land_talker = nodeHandle.advertise<std_msgs::Empty>("/bebop/land", 1000);
    ros::Duration(10).sleep();
    ROS_INFO_STREAM("\n-----TAKEOFF-----\n");
    takeoff_talker.publish(msg_takeoff);
    //std::this_thread::sleep_for (std::chrono::seconds(5));
    odom_sub = nodeHandle.subscribe("/bebop/odom", 1000, odomCallback);
    while (spinCount < 10) {
        // take some odometry readings
        ros::spinOnce();
        spinCount++;
    }

    
    ros::Duration(20).sleep();
    ROS_INFO_STREAM("\n-----LAND--------\n");
    land_talker.publish(msg_land);

    // ros::spinOnce();
    return 0;
}