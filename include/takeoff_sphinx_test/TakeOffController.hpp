#pragma once

#include <ros/ros.h>
#include <string>

using std::string;

namespace takeoff_sphinx_test {
    class TakeOffController {
        public:
            TakeOffController(ros::NodeHandle& nodeHandle);

            virtual ~TakeOffController();
        private:
            ros::NodeHandle nodeHandle_;
            string takeoff_name;
            string land_name;
            ros::Publisher takeoff_talker;
            ros::Publisher land_talker;
    };
}