
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher() : Node("status_publisher")
    {
        publisher_ = create_publisher<std_msgs::msg::String>(
            "robot_status", 10);

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String message;
                message.data = "Robot controller is running!";

                publisher_->publish(message);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<StatusPublisher>());

    rclcpp::shutdown();
    return 0;
}
