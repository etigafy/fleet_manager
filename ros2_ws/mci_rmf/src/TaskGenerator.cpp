#include <chrono>

#include "Task.h"
#include "Package.h"
#include "rclcpp/rclcpp.hpp"
#include "h_interfaces/srv/delivery_request.hpp"
#include "gui_interfaces/srv/gui.hpp"

using namespace std::chrono_literals;
using namespace h_interfaces::srv;
using namespace gui_interfaces::srv;

int main(int argc, char **argv)
{
    if(argc != 2) return -1;

    rclcpp::init(argc, argv);

    // create node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("DeliveryRequest_client");
    rclcpp::Client<Gui>::SharedPtr client = node->create_client<Gui>("GuiService");

    // create request
    auto request = std::make_shared<Gui::Request>();
    request->type = PACKAGE_SINGLE;
    request->pickupid = 1;
    request->dropoffid = 1;
    request->machineidarray.push_back(1);
    // request->machineidarray.push_back(2);
    // request->machineidarray.push_back(3);

    // send request
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    // wait for service result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        printf("isaccepted: %i\n", result.get()->isaccepted);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service deliveryRequest");
    }

    rclcpp::shutdown();
    return 0;
}