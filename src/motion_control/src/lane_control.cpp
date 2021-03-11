#include "motion_control/lane_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <functional>

namespace motion_control
{
  LaneControl::LaneControl(const rclcpp::NodeOptions &options)
    : Node("lane_control", options)
  {
    // param
    parse_parameters();
    set_on_parameters_set_callback(
      std::bind(
        &LaneControl::dynamic_load_params, 
        this, std::placeholders::_1));

    // topic
    lane_center_sub_ = create_subscription<std_msgs::msg::Float64>(
      "lane_center_x",
      10,
      std::bind(&LaneControl::lane_callback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),"INIT");
  }

  void LaneControl::parse_parameters()
  {

  }

  rcl_interfaces::msg::SetParametersResult LaneControl::dynamic_load_params(
    const std::vector<rclcpp::Parameter> &params)
  {
    
  }

  void LaneControl::lane_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {

  }

  void pid_controller()
  {
    
  }

  void LaneControl::stop_control()
  {

  }

  void run()
  {

  }

  void pub()
  {

  }

  
} // motion_control

RCLCPP_COMPONENTS_REGISTER_NODE(motion_control::LaneControl)