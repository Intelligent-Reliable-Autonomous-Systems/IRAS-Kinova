#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class KinovaResetNode : public rclcpp::Node
{
public:
    KinovaResetNode()
    : Node("kinova_reset_node")
    {
        // Declare parameters with defaults
        this->declare_parameter<std::vector<double>>("joint_positions", {0.0,0.523599,0.0,1.5708,0.0,0.785398,0.0});
        this->declare_parameter<double>("move_time", 2.0); // seconds

        // Get parameters
        this->get_parameter("joint_positions", reset_positions_);
        this->get_parameter("move_time", move_time_);

        if (reset_positions_.size() != 7) {
            RCLCPP_ERROR(this->get_logger(),
                "Expected 7 joint positions, got %zu", reset_positions_.size());
            throw std::runtime_error("Invalid joint positions size");
        }

        // Publisher to joint trajectory topic
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Wait 2 seconds before sending trajectory
        timer_ = this->create_wall_timer(2s, std::bind(&KinovaResetNode::send_reset_position, this));
    }

private:
    void send_reset_position()
    {
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.joint_names = {"joint_1","joint_2","joint_3",
                                "joint_4","joint_5","joint_6","joint_7"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = reset_positions_;
        point.time_from_start.sec = static_cast<int>(move_time_);
        point.time_from_start.nanosec = static_cast<int>((move_time_ - static_cast<int>(move_time_)) * 1e9);

        traj_msg.points.push_back(point);

        joint_pub_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(),
            "Sent reset position in %.2f seconds", move_time_);

        timer_->cancel();
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> reset_positions_;
    double move_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinovaResetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
