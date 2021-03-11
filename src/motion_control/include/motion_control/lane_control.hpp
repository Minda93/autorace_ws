#ifndef MOTION_CONTROL_LANE_CONTROL_HPP_
#define MOTION_CONTROL_LANE_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>

namespace motion_control
{
  class LaneControl : public rclcpp::Node
  {
  public:
    explicit LaneControl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LaneControl() = default;

  private:
    void stop_control();
    void run();

  private:
    // init param
    void parse_parameters();
    rcl_interfaces::msg::SetParametersResult dynamic_load_params(
      const std::vector<rclcpp::Parameter> &params);

    // subscriber
    void lane_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // publisher
    void pub();

    // controller
    void pid_controller();

  private:
    // topic
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lane_center_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    
  }; // class LaneControl
  
} // namespace motion_control


#endif // MOTION_CONTROL_LANE_CONTROL_HPP_