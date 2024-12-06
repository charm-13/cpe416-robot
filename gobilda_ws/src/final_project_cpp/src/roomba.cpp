#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

enum class State {
    Forward,
    Backward,
    Turning,
    Stop
};

class BumpAndGo : public rclcpp::Node {
public:
    BumpAndGo() : Node("bump_and_go"), state_(State::Forward), distance_(std::numeric_limits<float>::infinity()) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive/cmd_vel", 10);
        laser_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "laser_status", 10, std::bind(&BumpAndGo::laser_status_callback, this, std::placeholders::_1));
        distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "distance", 10, std::bind(&BumpAndGo::distance_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&BumpAndGo::process_scan, this));
        last_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Starting... going forward...");
    }

private:
    void laser_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            state_ = State::Stop;
        }
    }

    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        distance_ = msg->data;
    }

    void process_scan() {
        auto current_time = this->get_clock()->now();
        geometry_msgs::msg::Twist twist;

        switch (state_) {
        case State::Forward:
            if (distance_ <= 5.0f) {
                last_time_ = current_time;
                state_ = State::Backward;
                RCLCPP_INFO(this->get_logger(), "Backward!");
            }
            twist.linear.x = linear_velocity_;
            twist.angular.z = 0.0;
            break;

        case State::Backward:
            if ((current_time - last_time_).seconds() >= 2.0) {
                last_time_ = current_time;
                state_ = State::Turning;
                RCLCPP_INFO(this->get_logger(), "Turning!");
            }
            twist.linear.x = -0.5 * linear_velocity_;
            twist.angular.z = 0.0;
            break;

        case State::Turning:
            if (distance_ > 5.0f) {
                state_ = State::Forward;
                RCLCPP_INFO(this->get_logger(), "Forward!");
            }
            twist.linear.x = 0.0;
            twist.angular.z = angular_velocity_;
            break;

        case State::Stop:
        default:
            RCLCPP_INFO(this->get_logger(), "Shutting off...");
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            rclcpp::shutdown();
            return;
        }

        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr laser_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    float distance_;
    rclcpp::Time last_time_;

    const float linear_velocity_ = 0.5f;
    const float angular_velocity_ = 0.35f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BumpAndGo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
