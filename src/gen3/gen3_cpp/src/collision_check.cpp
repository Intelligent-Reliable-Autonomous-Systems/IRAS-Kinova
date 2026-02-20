/**
collision_check.cpp

THe file to check for obstacles, and to avoid them.
Going to use MoveIt2 library and ROS2 Geometry Package
to set collision boxes.

Author: Natalia Zaitseva


Command to test:

**/

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std;

class CollisionCheck : public rclcpp::Node {
public:
  CollisionCheck()
  : Node("collision_check") {
    sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/collision_policy", 10, bind(&CollisionCheck::callback, this, placeholders::_1));

    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/filtered_policy", 10);

    RCLCPP_INFO(this->get_logger(), "Collision check node started");
  }

  private:
    void callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Forwarding the function");
      pub_->publish(*msg);
  }

  private:
  //save subscriber and publisher instances so they don't get destroyed
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionCheck>());
  rclcpp::shutdown();
  return 0;
}