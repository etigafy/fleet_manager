// #define DEBUG_MSGS

#include "rclcpp/rclcpp.hpp"
#include "TurtleClient.h"

bool checkInput(int, char*[]);

// ------------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    // if( !checkInput(argc, argv) ) return -1;

    // int clientID = atoi(argv[1]);

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // create nodes
    rclcpp::Node::SharedPtr node = std::make_shared<TurtleClient>();

    executor.add_node(node);
    executor.spin();

    // cleanup
    rclcpp::shutdown();

    return 0;
}







// ------------------------------------------------------------------------------------------------
bool checkInput(int argc, char* argv[])
{
    // check size of argv
    if(argc != 2)
    {
        printf("Invalid number of arguments: provide ClientID!\n");
        return false;
    }

    // extract clientID argument
    const char* input = argv[1];

    // get length of input
    size_t size = 0;

    for(size_t i = 0; ; i++)
    {
        if( input[i] != '\0' ) size++;
        else break;
    }

    // check if input is number
    for(size_t i = 0; i < size; i++)
    {
        if( !isdigit(input[i]) )
        {
            printf("Invalid arguments: NaN!\n");
            return false;
        }
    }

    // check blacklisted clientIDs
    if(atoi(input) == 0)
    {
        printf("Invalid arguments: clientID is reserved!\n");
        return false;
    }
    
    printf("ClientID: %s\n", input);
    return true;
}