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
    trigger_(false), centerX_{0.0}, testPIDRun_{false}
  {
    // param
    parse_parameters();
    set_on_parameters_set_callback(
      std::bind(
        &LaneControl::dynamic_load_params, 
        this, std::placeholders::_1));

    // set controller
    pid_ctrl_ptr_ = std::make_shared<controller::PID>(
      cfg_.ctrlTarget, cfg_.pid.kp, cfg_.pid.ki, cfg_.pid.kd);
    
    // topic
    lane_center_sub_ = create_subscription<std_msgs::msg::Float64>(
      "lane_center_x",
      1,
      std::bind(&LaneControl::lane_callback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 1);

    // timer
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    prevTime_ = clock_.now();

    // run rate 25 hz
    runTimer_ = create_wall_timer(
      std::chrono::milliseconds(40),
      std::bind(&LaneControl::run, this));

    // shutdown stop robot
    rclcpp::on_shutdown(std::bind(&LaneControl::stop_control, this));
    // RCLCPP_INFO(this->get_logger(),"INIT");
  }

  void LaneControl::parse_parameters()
  {
    // test PID param
    this->declare_parameter<bool>("test_pid_param", false);

    get_parameter_or<bool>(
      "test_pid_param",
      testPIDRun_,
      false);

    // robot param
    this->declare_parameter<double>("robot.maxLinearVel", 0.15);
    this->declare_parameter<double>("robot.maxAngularVel", 2.0);

    // pid 
    this->declare_parameter<double>("pid.kp", 0.1);
    this->declare_parameter<double>("pid.ki", 0);
    this->declare_parameter<double>("pid.kd", 0);

    get_parameter_or<double>(
      "robot.maxLinearVel",
      cfg_.maxLinearVel,
      0.15);
    
    get_parameter_or<double>(
      "robot.maxAngularVel",
      cfg_.maxAngularVel,
      2.0);

    get_parameter_or<double>(
      "pid.kp",
      cfg_.pid.kp,
      0.1);
    
    get_parameter_or<double>(
      "pid.ki",
      cfg_.pid.ki,
      0.0);
    
    get_parameter_or<double>(
      "pid.kd",
      cfg_.pid.kd,
      0.0);
  }

  rcl_interfaces::msg::SetParametersResult LaneControl::dynamic_load_params(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for(const auto &param : params)
    {
      if(param.get_name() == "robot.maxLinearVel" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.maxLinearVel = param.as_double();
      }else if(param.get_name() == "robot.maxAngularVel" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.maxAngularVel = param.as_double();
      }else if(param.get_name() == "pid.kp" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.pid.kp = param.as_double();
        pid_ctrl_ptr_->set_kp(cfg_.pid.kp);
      }else if(param.get_name() == "pid.ki" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.pid.ki = param.as_double();
        pid_ctrl_ptr_->set_ki(cfg_.pid.ki);
      }else if(param.get_name() == "pid.kd" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.pid.kd = param.as_double();
        pid_ctrl_ptr_->set_kd(cfg_.pid.kd);
      }else if(param.get_name() == "test_pid_param" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        testPIDRun_ = param.as_bool();
        if(!testPIDRun_){
          pid_ctrl_ptr_->reset();
          stop_control();
        }
      }else{
        result.successful = false;
        result.reason = "not found param";
      }
    }

    return result;
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

    double turn{pid_ctrl_ptr_->run(state, delta)};
    turn /= 100.0;
    RobotVel robotVel{0.0, 0.0};

    // design control
    robotVel.linearX = cfg_.maxLinearVel*(1 - std::abs(turn));
    // robotVel.linearX = (robotVel.linearX < 0.2)? 0.2 : robotVel.linearX;
    robotVel.angularZ = turn;

    // RCLCPP_INFO(this->get_logger(),"pid value : %lf", turn);
    RCLCPP_INFO(this->get_logger(),"robot vel : %lf %lf", robotVel.linearX, robotVel.angularZ);

    return robotVel;
  }

  void LaneControl::stop_control()
  {
    pub(RobotVel{0.0, 0.0});
  }

  void LaneControl::run()
  { 
    if(trigger_ && testPIDRun_)
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