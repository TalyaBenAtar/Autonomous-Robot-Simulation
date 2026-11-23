
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// Controls a differential-drive robot along a square trajectory.
// Uses odometry feedback instead of fixed movement durations.
class MovementController : public rclcpp::Node
{
public:
    MovementController() : Node("movement_controller")
    {
        // Publish velocity commands to the Gazebo DiffDrive controller.
        publisher_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Subscribe to estimated robot position and orientation.
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                const auto &position = msg->pose.pose.position;
                const auto &q = msg->pose.pose.orientation;

                current_x_ = position.x;
                current_y_ = position.y;

                // Convert quaternion orientation to yaw (radians).
                current_yaw_ = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

                has_odometry_ = true;
            });

        // Execute the feedback controller approximately 10 times/sec.
        timer_ = create_wall_timer(
            100ms,
            [this]() { controlLoop(); });

        RCLCPP_INFO(get_logger(), "Square movement controller started");
    }

private:
    enum class State
    {
        MOVING_FORWARD,
        TURNING,
        STOPPED
    };

    // Target trajectory parameters.
    static constexpr double SIDE_LENGTH = 1.0;  // meters
    static constexpr double TURN_ANGLE = 1.57079632679; // 90 degrees
    static constexpr int TOTAL_SIDES = 4;

    // Normalize angles to [-pi, pi] to handle wraparound.
    double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    // Save the current pose as the reference for a new movement.
    void saveStartPose()
    {
        start_x_ = current_x_;
        start_y_ = current_y_;
        start_yaw_ = current_yaw_;
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist command;

        // Don't move until valid odometry has been received.
        if (!has_odometry_)
        {
            publisher_->publish(command);
            return;
        }

        // Initialize the first side from the robot's current pose.
        if (!initialized_)
        {
            saveStartPose();
            initial_x_ = current_x_;
            initial_y_ = current_y_;
            initialized_ = true;

            RCLCPP_INFO(get_logger(), "Starting square trajectory");
        }

        switch (state_)
        {
            case State::MOVING_FORWARD:
            {
                // Measure displacement from the beginning of this side.
                double dx = current_x_ - start_x_;
                double dy = current_y_ - start_y_;
                double distance = std::hypot(dx, dy);

                double remaining = SIDE_LENGTH - distance;

                if (remaining > 0.02)
                {
                    // Slow down near the end of each side.
                    command.linear.x =
                        std::clamp(remaining * 0.8, 0.05, 0.2);
                }
                else
                {
                    // Save heading before beginning the next turn.
                    start_yaw_ = current_yaw_;
                    state_ = State::TURNING;

                    RCLCPP_INFO(
                        get_logger(),
                        "Side %d completed. Distance: %.3f m",
                        completed_sides_ + 1, distance);
                }
                break;
            }

            case State::TURNING:
            {
                // Measure rotation relative to the start of this turn.
                double turned = normalizeAngle(
                    current_yaw_ - start_yaw_);

                double remaining = TURN_ANGLE - turned;

                if (remaining > 0.03)
                {
                    // Reduce angular velocity as we approach 90 degrees.
                    command.angular.z =
                        std::clamp(remaining * 1.0, 0.1, 0.5);
                }
                else
                {
                    completed_sides_++;

                    RCLCPP_INFO(
                        get_logger(),
                        "Turn %d completed. Angle: %.2f degrees",
                        completed_sides_,
                        turned * 180.0 / 3.141592653589793);

                    if (completed_sides_ >= TOTAL_SIDES)
                    {
                        // All four sides and turns are complete.
                        state_ = State::STOPPED;

                        double final_error = std::hypot(
                            current_x_ - initial_x_,
                            current_y_ - initial_y_);

                        RCLCPP_INFO(
                            get_logger(),
                            "Square completed! Position error: %.3f m",
                            final_error);
                    }
                    else
                    {
                        // Begin the next side from the current pose.
                        saveStartPose();
                        state_ = State::MOVING_FORWARD;
                    }
                }
                break;
            }

            case State::STOPPED:
                // Keep publishing zero velocity.
                break;
        }

        publisher_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::MOVING_FORWARD;

    bool has_odometry_ = false;
    bool initialized_ = false;

    int completed_sides_ = 0;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;

    double initial_x_ = 0.0;
    double initial_y_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementController>());
    rclcpp::shutdown();
    return 0;
}
