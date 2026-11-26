#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomCovariance : public rclcpp::Node
{
public:
    OdomCovariance() : Node("odom_covariance")
    {
        publisher_ =
            create_publisher<nav_msgs::msg::Odometry>(
                "/odom/with_covariance", 10);

        subscriber_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
                {
                    odom_callback(msg);
                });
    }

private:
    void odom_callback(
        nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        auto output = *msg;

        // Forward velocity variance
        output.twist.covariance[0] = 0.0004;

        // Lateral velocity variance
        output.twist.covariance[7] = 0.0001;

        // Yaw rate variance
        output.twist.covariance[35] = 0.0025;

        publisher_->publish(output);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomCovariance>());
    rclcpp::shutdown();

    return 0;
}