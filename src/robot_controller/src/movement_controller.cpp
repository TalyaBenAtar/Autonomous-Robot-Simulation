#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// Controls a differential-drive robot along a repeating square trajectory.
// Uses raw odometry feedback and waits for EKF initialization before moving.
class MovementController : public rclcpp::Node
{
public:
    MovementController() : Node("movement_controller")
    {
        // Publish velocity commands to the Gazebo DiffDrive controller.
        publisher_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Subscribe to raw wheel odometry.
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

        // Wait for the EKF to initialize before allowing movement.
        filtered_odom_subscriber_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered", 10,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr)
                {
                    if (!has_filtered_odometry_)
                    {
                        has_filtered_odometry_ = true;

                        RCLCPP_INFO(
                            get_logger(),
                            "EKF ready. Movement enabled.");
                    }
                });

        // Execute the feedback controller approximately 10 times/sec.
        timer_ = create_wall_timer(
            100ms,
            [this]() { controlLoop(); });

        RCLCPP_INFO(
            get_logger(),
            "Square movement controller started. Waiting for odometry and EKF.");
    }

private:
    enum class State
    {
        MOVING_FORWARD,
        TURNING
    };

    // Target trajectory parameters.
    static constexpr double SIDE_LENGTH = 1.0; // meters
    static constexpr double TURN_ANGLE = 1.57079632679; // 90 degrees
    static constexpr int TOTAL_SIDES = 4;

    // Movement accuracy tolerances.
    static constexpr double DISTANCE_TOLERANCE = 0.005; // 5 mm
    static constexpr double ANGLE_TOLERANCE = 0.005; // ~0.29 degrees

    // Movement speed limits.
    static constexpr double MAX_LINEAR_SPEED = 0.4; // m/s
    static constexpr double MAX_ANGULAR_SPEED = 1.0; // rad/s
    static constexpr double MIN_LINEAR_SPEED = 0.01; // m/s
    static constexpr double MIN_ANGULAR_SPEED = 0.01; // rad/s

    // Proportional control gains.
    static constexpr double LINEAR_GAIN = 1.2;
    static constexpr double ANGULAR_GAIN = 1.5;

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

        // Wait for both raw odometry and EKF initialization.
        if (!has_odometry_ || !has_filtered_odometry_)
        {
            publisher_->publish(command);
            return;
        }

        // Initialize the first square from the robot's current pose.
        if (!initialized_)
        {
            saveStartPose();

            initial_x_ = current_x_;
            initial_y_ = current_y_;

            initialized_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Starting square trajectory");
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

                if (remaining > DISTANCE_TOLERANCE)
                {
                    // Slow down near the end of each side.
                    command.linear.x = std::clamp(
                        remaining * LINEAR_GAIN,
                        MIN_LINEAR_SPEED,
                        MAX_LINEAR_SPEED);
                }
                else
                {
                    // Save heading before beginning the next turn.
                    start_yaw_ = current_yaw_;
                    state_ = State::TURNING;

                    RCLCPP_INFO(
                        get_logger(),
                        "Square %d, Side %d completed. Distance: %.3f m",
                        completed_squares_ + 1,
                        completed_sides_ + 1,
                        distance);
                }

                break;
            }

            case State::TURNING:
            {
                // Measure rotation relative to the start of this turn.
                double turned = normalizeAngle(
                    current_yaw_ - start_yaw_);

                double remaining = TURN_ANGLE - turned;

                if (remaining > ANGLE_TOLERANCE)
                {
                    // Reduce angular velocity as we approach 90 degrees.
                    command.angular.z = std::clamp(
                        remaining * ANGULAR_GAIN,
                        MIN_ANGULAR_SPEED,
                        MAX_ANGULAR_SPEED);
                }
                else
                {
                    completed_sides_++;

                    RCLCPP_INFO(
                        get_logger(),
                        "Square %d, Turn %d completed. Angle: %.2f degrees",
                        completed_squares_ + 1,
                        completed_sides_,
                        turned * 180.0 / 3.141592653589793);

                    if (completed_sides_ >= TOTAL_SIDES)
                    {
                        completed_squares_++;

                        // Calculate position error relative to the
                        // beginning of the completed square.
                        double final_error = std::hypot(
                            current_x_ - initial_x_,
                            current_y_ - initial_y_);

                        RCLCPP_INFO(
                            get_logger(),
                            "Square %d completed! Position error: %.3f m",
                            completed_squares_,
                            final_error);

                        // Reset the reference for the next square.
                        completed_sides_ = 0;
                        initial_x_ = current_x_;
                        initial_y_ = current_y_;

                        RCLCPP_INFO(
                            get_logger(),
                            "Starting square %d",
                            completed_squares_ + 1);
                    }

                    // Begin the next side from the current pose.
                    saveStartPose();
                    state_ = State::MOVING_FORWARD;
                }

                break;
            }
        }

        publisher_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_subscriber_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        filtered_odom_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::MOVING_FORWARD;

    bool has_odometry_ = false;
    bool has_filtered_odometry_ = false;
    bool initialized_ = false;

    int completed_sides_ = 0;
    int completed_squares_ = 0;

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