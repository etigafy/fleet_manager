// #define DEBUG_MSGS

#include "rclcpp/rclcpp.hpp"
#include "TaskPlanner.h"


// ------------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPlanner>());
    rclcpp::shutdown();
    return 0;
}





// TODO:
// + tell planner when client is charging
// + redo and streamline landmark booking
// + redo task generation pipeline
// - redo taskGroup generation in receiver
// - properly handle processing time of machines in receiver