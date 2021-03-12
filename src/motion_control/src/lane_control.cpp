#include "motion_control/lane_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <functional>
#include <chrono>

namespace motion_control
{
  LaneControl::LaneControl(const rclcpp::NodeOptions &options)
    : Node("lane_control", options),
    trigger_(false), centerX_{0.0}
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

    // timer
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    prevTime_ = clock_.now();

    // run rate 25 hz
    runTimer_ = create_wall_timer(
      std::chrono::milliseconds(40),
      std::bind(&LaneControl::run, this));

    // RCLCPP_INFO(this->get_logger(),"INIT");
  }

  void LaneControl::parse_parameters()
  {
    pid_ctrl_ptr_ = std::make_shared<controller::PID>(
      cfg_.ctrlTarget, cfg_.pid.kp, cfg_.pid.ki, cfg_.pid.kd);
  }

  rcl_interfaces::msg::SetParametersResult LaneControl::dynamic_load_params(
    const std::vector<rclcpp::Parameter> &params)
  {
    
  }

  void LaneControl::lane_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    centerX_ = msg->data;
    trigger_ = true;

    // RCLCPP_INFO(this->get_logger(),"centerX : %lf", centerX_);
  }

  RobotVel LaneControl::pid_controller(const double state)
  {
    // time
    rclcpp::Duration duration{clock_.now() - prevTime_};
    double delta{duration.seconds()};

    double pidValue{pid_ctrl_ptr_->run(state, delta)};
    RobotVel robotVel{0.0, 0.0};

    // design control

    return robotVel;
  }

  void LaneControl::stop_control()
  {
    pub(RobotVel{0.0, 0.0});
  }

  void LaneControl::run()
  { 
    if(trigger_)
    {
      RobotVel robotVel{pid_controller(centerX_)};
      pub(robotVel);
    }
    trigger_ = false;
  }

  void LaneControl::pub(const RobotVel &robotVel)
  {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    
    if(robotVel.linearX > cfg_.maxLinearVel)
      twist->linear.x = cfg_.maxLinearVel;
    else if(robotVel.linearX < -cfg_.maxLinearVel)
      twist->linear.x = -cfg_.maxLinearVel;
    else
      twist->linear.x = robotVel.linearX;

    if(robotVel.angularZ > cfg_.maxAngularVel)
      twist->angular.z = cfg_.maxAngularVel;
    else if(robotVel.angularZ < -cfg_.maxAngularVel)
      twist->angular.z = -cfg_.maxAngularVel;
    else
      twist->angular.z = robotVel.angularZ;

    cmd_vel_pub_->publish(std::move(twist));
  }
} // motion_control

RCLCPP_COMPONENTS_REGISTER_NODE(motion_control::LaneControl)