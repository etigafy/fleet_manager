#include "rclcpp/rclcpp.hpp"
#include "h_interfaces/srv/elevator_pos.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if(argc != 4)
  {
    RCLCPP_INFO(rclcpp::get_logger("Elevator Client"), "invalid input args: pos vel acc");
    return -1;
  }

  // create node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("elevator_client");
  rclcpp::Client<h_interfaces::srv::ElevatorPos>::SharedPtr client =
    node->create_client<h_interfaces::srv::ElevatorPos>("elevator_service");

  // create request
  auto request = std::make_shared<h_interfaces::srv::ElevatorPos::Request>();
  request->position = atof(argv[1]);
  request->velocity = atof(argv[2]);
  request->acceleration = atof(argv[3]);

  // check service availability
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("Elevator Client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("Elevator Client"), "elevator_service not available, waiting...");
  }

  // send request
  auto result = client->async_send_request(request);

  // wait for result
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->status)
    {
      RCLCPP_INFO(rclcpp::get_logger("Elevator Client"), "Moved elevator to: %f mm", request->position);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("Elevator Client"), "Could not move elevator!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Elevator Client"), "Failed to call service elevator_service");
  }

  rclcpp::shutdown();
  return 0;
}