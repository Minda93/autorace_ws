#ifndef MOTION_CONTROL_LANE_CONTROL_HPP_
#define MOTION_CONTROL_LANE_CONTROL_HPP_

#include "motion_control/motion_control_param.hpp"
#include "motion_control/controller/pid.hpp"

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
    void pub(const RobotVel &robotVel);

    // controller
    RobotVel pid_controller(const double state);

  private:
    // topic
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lane_center_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::Clock clock_;
    rclcpp::Time prevTime_;
    rclcpp::TimerBase::SharedPtr runTimer_;
    
    LaneCtrlParam cfg_; // param
    bool trigger_; 
    double centerX_; // control error
    std::shared_ptr<controller::PID> pid_ctrl_ptr_;

  }; // class LaneControl
  
} // namespace motion_control


#endif // MOTION_CONTROL_LANE_CONTROL_HPP_