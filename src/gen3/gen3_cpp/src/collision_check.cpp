#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <rclcpp/parameter_client.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

class CollisionCheck : public rclcpp::Node{
public:
  CollisionCheck()
  : rclcpp::Node("collision_check"){
    RCLCPP_INFO(get_logger(), "CollisionCheck starting...");
    sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/collision_policy", 10,
      std::bind(&CollisionCheck::trajectory_callback, this, std::placeholders::_1));
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/cmd_safety", 10);
    collision_obj_pub_ = create_publisher<moveit_msgs::msg::CollisionObject>("/collision_object", 10);
  }


  bool init(){
    RCLCPP_INFO(get_logger(), "Initializing MoveIt PlanningScene...");
    if (!load_descriptions_from_move_group()){
      RCLCPP_ERROR(get_logger(), "Failed to load robot_description(_semantic) from /move_group.");
      return false;
    }
    //create planning scene monitor
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description", "robot_description_semantic");
    if (!psm_ || !psm_->getRobotModel()){
      RCLCPP_ERROR(get_logger(), "Robot model not loaded inside PlanningSceneMonitor.");
      return false;
    }

    psm_->startSceneMonitor();//listener to /monitored_planning_scene
    psm_->startWorldGeometryMonitor();//listener to /collision_object, /planning_scene_world
    psm_->startStateMonitor("joint_states");///listener to joint states
    //wait until planning scene is ready
    if (!wait_for_planning_scene(std::chrono::seconds(5))){
      RCLCPP_ERROR(get_logger(), "Planning scene did not become ready.");
      return false;
    }
    publish_table();
    ready_ = true;
    RCLCPP_INFO(get_logger(), "CollisionCheck ready.");
    return true;
  }

private:
  bool load_descriptions_from_move_group(){
    auto client = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "/move_group");
    if (!client->wait_for_service(std::chrono::seconds(5))){
      RCLCPP_ERROR(get_logger(), "Parameter service for /move_group not available.");
      return false;
    }
    //get urdf and srdf
    auto params = client->get_parameters({"robot_description", "robot_description_semantic"});
    if (params.size() != 2){
      RCLCPP_ERROR(get_logger(), "Could not retrieve both robot_description and robot_description_semantic.");
      return false;
    }
    const auto & urdf = params[0].as_string();
    const auto & srdf = params[1].as_string();
    if (urdf.empty() || srdf.empty()){
      RCLCPP_ERROR(get_logger(), "robot_description or robot_description_semantic was empty.");
      return false;
    }
    //store parameters for the robot loader to be able to load them
    if (!has_parameter("robot_description"))
      declare_parameter<std::string>("robot_description", urdf);
    if (!has_parameter("robot_description_semantic"))
      declare_parameter<std::string>("robot_description_semantic", srdf);
    set_parameter(rclcpp::Parameter("robot_description", urdf));
    set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));
    RCLCPP_INFO(get_logger(), "Loaded robot_description + semantic from /move_group.");
    return true;
  }

  bool wait_for_planning_scene(std::chrono::seconds timeout){
    auto start = now();
    while (rclcpp::ok() && (now() - start) < rclcpp::Duration(timeout)){
      auto scene = psm_->getPlanningScene();
      if (scene && psm_->getRobotModel()){
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  void publish_table(){
    //create a collision object on a scene
    moveit_msgs::msg::CollisionObject table;
    //get the robot base
    table.header.frame_id = "base_link";
    table.id = "table_box";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    //x,y,z
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.8;
    primitive.dimensions[2] = 0.06;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = -0.03;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    collision_obj_pub_->publish(table);
    RCLCPP_INFO(get_logger(), "Published table collision object on /collision_object (for RViz + MoveIt world).");
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
    if (!ready_){
      RCLCPP_WARN(get_logger(), "Not ready yet (planning scene/model still initializing).");
      return;
    }
    if (!msg || msg->points.empty()){
      RCLCPP_WARN(get_logger(), "Received empty trajectory.");
      return;
    }
    auto scene = psm_->getPlanningScene();
    if (!scene){
      RCLCPP_ERROR(get_logger(), "Planning scene is null.");
      return;
    }
    //copy current state as baseline
    moveit::core::RobotState state = scene->getCurrentState();
    //grab the arm from urdf/srdf
    const moveit::core::JointModelGroup* jmg = state.getJointModelGroup("manipulator");
    if (!jmg){
      RCLCPP_ERROR(get_logger(), "JointModelGroup 'manipulator' not found: SRDF mismatch.");
      return;
    }
    //check each point
    for (size_t i = 0; i < msg->points.size(); ++i){
      const auto & pt = msg->points[i];
      if (pt.positions.size() != jmg->getVariableCount()){
        RCLCPP_ERROR(
          get_logger(),
          "Point %zu has %zu positions but manipulator expects %zu.",
          i, pt.positions.size(), jmg->getVariableCount());
        return;
      }
      state.setJointGroupPositions(jmg, pt.positions);
      state.update();
      //collision test against robot and the world
      bool colliding = scene->isStateColliding(state, "manipulator");
      if (colliding){
        RCLCPP_WARN(get_logger(), "UNSAFE: Collision detected at trajectory point %zu. Stopping.", i);
        //publish the current position trajectory to stop motion
        trajectory_msgs::msg::JointTrajectory hold;
        hold.joint_names = msg->joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint hold_pt;
        //grab current positions from the planning scene state
        std::vector<double> current;
        state.copyJointGroupPositions(jmg, current);
        hold_pt.positions = current;
        //hold time
        hold_pt.time_from_start.sec = 1;
        hold.points.push_back(hold_pt);
        pub_->publish(hold);
        return;
      }
    }
    RCLCPP_INFO(get_logger(), "SAFE: Forwarding trajectory to /cmd_safety.");
    pub_->publish(*msg);
  }
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_obj_pub_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  bool ready_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionCheck>();
  if (!node->init()){
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}