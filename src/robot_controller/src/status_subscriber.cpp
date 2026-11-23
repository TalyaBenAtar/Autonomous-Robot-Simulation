
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ROS 2 node responsible for receiving robot status messages.
class StatusSubscriber : public rclcpp::Node
{
public:
    StatusSubscriber() : Node("status_subscriber")
    {
        // Subscribe to the "robot_status" topic.
        // Message type must match the publisher's message type.
        // Queue depth: 10 messages.
        subscription_ = create_subscription<std_msgs::msg::String>(
            "robot_status",
            10,
            // ROS 2 invokes this callback whenever a message arrives.
            [this](const std_msgs::msg::String::SharedPtr message) {

                // Print the received message using the ROS 2 logger.
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received: %s",
                    message->data.c_str());
            });
    }

private:
    // Keep the subscription alive for the node's lifetime.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library.
    rclcpp::init(argc, argv);

    // Keep the node running and process incoming message callbacks.
    rclcpp::spin(std::make_shared<StatusSubscriber>());

    // Clean up ROS 2 resources.
    rclcpp::shutdown();
    return 0;
}
