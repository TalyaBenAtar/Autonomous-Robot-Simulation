
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ROS 2 node responsible for publishing robot status messages.
class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher() : Node("status_publisher")
    {
        // Create a publisher on the "robot_status" topic.
        // Message type: std_msgs::msg::String
        // Queue depth: 10 messages.
        publisher_ = create_publisher<std_msgs::msg::String>(
            "robot_status", 10);

        // Execute the callback every second.
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String message;
                message.data = "Robot controller is running!";

                // Send the message to all subscribers of this topic.
                publisher_->publish(message);
            });
    }

private:
    // Keep the publisher and timer alive for the node's lifetime.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library.
    rclcpp::init(argc, argv);

    // Create the node and process callbacks until shutdown.
    rclcpp::spin(std::make_shared<StatusPublisher>());

    // Clean up ROS 2 resources.
    rclcpp::shutdown();
    return 0;
}
