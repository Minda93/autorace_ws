#include "detect_object/detect_lane.hpp"
#include "detect_object/interface/interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace detect_object
{
  DetectLane::DetectLane(const rclcpp::NodeOptions &options)
    : Node("detect_lane", options)
  {
    // param
    parse_parameters();
    set_on_parameters_set_callback(
      std::bind(&DetectLane::dynamic_load_params, 
        this, std::placeholders::_1));

    // topic
    auto qosSensor = rclcpp::QoS(
      rclcpp::QoSInitialization(
        rmw_qos_profile_sensor_data.history,
        rmw_qos_profile_sensor_data.depth
      ),
      rmw_qos_profile_sensor_data
    );
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      qosSensor,
      std::bind(&DetectLane::image_callback, this, std::placeholders::_1)
    );
    
    // temporary setting
    startTimer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&DetectLane::process, this));
  }

  void DetectLane::parse_parameters()
  {
    this->declare_parameter<bool>("show_image", true);

    // Perspective Transform param
    this->declare_parameter<int>("birdView.top_x", 70);
    this->declare_parameter<int>("birdView.top_y", -40);
    this->declare_parameter<int>("birdView.bottom_x", 320);
    this->declare_parameter<int>("birdView.bottom_y", 240);

    // color model
    this->declare_parameter<int>(
      "hsv_model.yellow.hue_l", 27,
      interface::set_num_range<double>("hsv_model.yellow.hue_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.saturation_l", 130,
      interface::set_num_range<double>("hsv_model.yellow.saturation_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.value_l", 160,
      interface::set_num_range<double>("hsv_model.yellow.value_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.hue_h", 41,
      interface::set_num_range<double>("hsv_model.yellow.hue_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.saturation_h", 255,
      interface::set_num_range<double>("hsv_model.yellow.saturation_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.value_h", 255,
      interface::set_num_range<double>("hsv_model.yellow.value_h", interface::INTEGER, 0, 255, 1));

    this->declare_parameter<int>(
      "hsv_model.white.hue_l", 0,
      interface::set_num_range<double>("hsv_model.white.hue_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.saturation_l", 0,
      interface::set_num_range<double>("hsv_model.white.saturation_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.value_l", 180,
      interface::set_num_range<double>("hsv_model.white.value_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.hue_h", 25,
      interface::set_num_range<double>("hsv_model.white.hue_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.saturation_h", 36,
      interface::set_num_range<double>("hsv_model.white.saturation_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.value_h", 255,
      interface::set_num_range<double>("hsv_model.white.value_h", interface::INTEGER, 0, 255, 1));

    get_parameter_or<bool>(
      "show_image",
      cfg_.showImage,
      true);
    
    get_parameter_or<int>(
      "birdView.top_x",
      cfg_.birdView.top_x,
      70);
    
    get_parameter_or<int>(
      "birdView.top_y",
      cfg_.birdView.top_y,
      -40);
    
    get_parameter_or<int>(
      "birdView.bottom_x",
      cfg_.birdView.bottom_x,
      320);
    
    get_parameter_or<int>(
      "birdView.bottom_y",
      cfg_.birdView.bottom_y,
      240);
    
    // maybe have better method
    std::vector<std::string> param_names = {
      "hsv_model.yellow.hue_l",
      "hsv_model.yellow.saturation_l",
      "hsv_model.yellow.value_l",
      "hsv_model.yellow.hue_h",
      "hsv_model.yellow.saturation_h",
      "hsv_model.yellow.value_h",
      "hsv_model.white.hue_l",
      "hsv_model.white.saturation_l",
      "hsv_model.white.value_l",
      "hsv_model.white.hue_h",
      "hsv_model.white.saturation_h",
      "hsv_model.white.value_h"};
    std::vector<rclcpp::Parameter> params = get_parameters(param_names);
        
    cfg_.yellowHSV.resize(6);
    cfg_.whiteHSV.resize(6);
    for(int idx{0}; idx < params.size(); ++idx)
    {
      if(idx < 6)
      {
        cfg_.yellowHSV[idx] = params[idx].as_int();
      }else{
        cfg_.whiteHSV[idx - 6] = params[idx].as_int();
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult DetectLane::dynamic_load_params(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for(const auto &param : params)
    {
      if(param.get_name() == "show_image" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        cfg_.showImage = param.as_bool();
      }else if(param.get_name() == "birdView.top_x" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.birdView.top_x = param.as_int();
      }else if(param.get_name() == "birdView.top_y" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.birdView.top_y = param.as_int();
      }else if(param.get_name() == "birdView.bottom_x" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.birdView.bottom_x = param.as_int();
      }else if(param.get_name() == "birdView.bottom_y" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.birdView.bottom_y = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.hue_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[0] = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.saturation_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[1] = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.value_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[2] = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.hue_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[3] = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.saturation_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[4] = param.as_int();
      }else if(param.get_name() == "hsv_model.yellow.value_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.yellowHSV[5] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.hue_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[0] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.saturation_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[1] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.value_l" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[2] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.hue_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[3] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.saturation_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[4] = param.as_int();
      }else if(param.get_name() == "hsv_model.white.value_h" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        cfg_.whiteHSV[5] = param.as_int();
      }
    }

    return result;
  }

  void DetectLane::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame{
      msg->height, msg->width,
      encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step};
    
    if(msg->encoding == "rgb8")
    {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    frame.copyTo(src_);
  }

  int DetectLane::encoding2mat_type(const std::string encoding)
  {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    } else if (encoding == "bgra8") {
      return CV_8UC4;
    } else if (encoding == "32FC1") {
      return CV_32FC1;
    } else if (encoding == "rgb8") {
      return CV_8UC3;
    } else {
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  void DetectLane::image_show(const cv::Mat &src, bool showImage)
  {
    if(showImage && !src.empty()){
      cv::imshow("Detect Lane Image", src);
      
      // esc keyboard key = 27
      if(cv::waitKey(3) == 27)
      {
        cv::destroyWindow("Detect Lane Image");
        rclcpp::shutdown();
      }
    }
  }

  void DetectLane::homography_transform_process(cv::Mat &src, cv::Mat &dst)
  {
    if(src.empty()){
      return;
    }

    // src size(640, 480)
    cv::Point2f centerPoint{src_.size()/2};
    cv::Point2f srcVertices[4]{
      {centerPoint.x - cfg_.birdView.top_x, centerPoint.y - cfg_.birdView.top_y},
      {centerPoint.x + cfg_.birdView.top_x, centerPoint.y - cfg_.birdView.top_y},
      {centerPoint.x + cfg_.birdView.bottom_x, centerPoint.y + cfg_.birdView.bottom_y},
      {centerPoint.x - cfg_.birdView.bottom_x, centerPoint.y + cfg_.birdView.bottom_y}
    };

    // dst size(250, 300)
    // [200, 0], [800, 0], [800, 600], [200, 600]
    cv::Point2f dstVertices[4]{
      {0, 0},
      {250, 0},
      {250, 300},
      {0, 300}
    };

    // draw line for bird view
    for(int idx{0}; idx < 4; ++idx)
    {
      cv::line(src, srcVertices[idx], srcVertices[(idx+1)%4], cv::Scalar( 255, 0, 0 ), 1, cv::LINE_8);
    }

    cv::Mat perspectiveMatrix{getPerspectiveTransform(srcVertices, dstVertices)};
    cv::warpPerspective(
      src, dst, perspectiveMatrix,
      cv::Size(250, 300), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  }

  void DetectLane::mask_lane(cv::Mat &src, cv::Mat &maskYellow, cv::Mat &maskWhite)
  {
    if(src.empty()){
      return;
    }

    cv::Mat hsv;

    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(
      hsv,
      cv::Scalar(cfg_.yellowHSV[0], cfg_.yellowHSV[1], cfg_.yellowHSV[2]), 
      cv::Scalar(cfg_.yellowHSV[3], cfg_.yellowHSV[4], cfg_.yellowHSV[5]), 
      maskYellow
    );

    cv::inRange(
      hsv,
      cv::Scalar(cfg_.whiteHSV[0], cfg_.whiteHSV[1], cfg_.whiteHSV[2]), 
      cv::Scalar(cfg_.whiteHSV[3], cfg_.whiteHSV[4], cfg_.whiteHSV[5]), 
      maskWhite
    );
  }

  std::vector<cv::Point2f> DetectLane::sliding_window(const cv::Mat &src, cv::Rect window)
  {
    std::vector<cv::Point2f> points;
    const cv::Size imgSize{src.size()};

    if(src.empty()){
      return points;
    }

    while(true)
    {
      float currX = window.x + window.width*0.5f;

      cv::Mat roi{src(window)};
      std::vector<cv::Point2f> locations;

      // Get all non-black pixels. All are white in our case
      cv::findNonZero(roi, locations);

      float avgX{0.0f};
      for(int i{0}; i < locations.size(); ++i)
      {
        avgX += window.x + locations[i].x;
      }
      avgX = locations.empty() ? currX : avgX / locations.size();

      cv::Point point(avgX, window.y + window.height * 0.5f);
      points.push_back(point);

      // Move the window up
      window.y -= window.height;
      if (window.y < 0)
      {
        window.y = 0;
        break;
      }

      // Move x position
      window.x += (point.x - currX);
      // Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix
      if (window.x < 0)
      {
        window.x = 0;
      }
      if (window.x + window.width >= imgSize.width)
      {
        window.x = imgSize.width - window.width - 1;
      }
    }

    return points;
  }

  void DetectLane::line_fitting(const cv::Mat &mask, std::vector<cv::Point2f> &lane_fit)
  {
    std::vector<cv::Point2f> locations;
    cv::findNonZero(mask, locations);

  }

  void DetectLane::process()
  {
    cv::Mat birdView;
    cv::Mat maskYellow, maskWhite;

    homography_transform_process(src_, birdView);
    mask_lane(birdView, maskYellow, maskWhite);

    // detect lane
    if(!whiteLaneFit_.empty() && !yellowLaneFit_.empty())
    {
      // line_fitting()
    }else{
      
    }
    // 
    // cv::Size maskSize{birdView.size()};
    // float middleX{maskSize.width / 2.};
    // std::vector<cv::Point2f> pts{sliding_window(maskYellow, cv::Rect{0, 270, 50, 30})}; // yellow lane
    
    // if(!birdView.empty()){
    //   for (int i = 0; i < pts.size() - 1; ++i)
    //   {
    //     cv::line(birdView, pts[i], pts[i + 1], cv::Scalar(255, 0, 0), 3);
    //   }
    // }
  
    // std::vector<cv::Point2f> pts{sliding_window(maskWhite, cv::Rect{100, 270, 50, 30})}; // white lane

    // if(!birdView.empty()){
    //   for (int i = 0; i < pts.size() - 1; ++i)
    //   {
    //     cv::line(birdView, pts[i], pts[i + 1], cv::Scalar(255, 0, 0), 3);
    //   }
    // }

    // image_show(src_, cfg_.showImage);
    // image_show(birdView, cfg_.showImage);
    
    // cv::Mat laneMask;
    // cv::bitwise_or(maskYellow, maskWhite, laneMask);
    // image_show(laneMask, cfg_.showImage);
    // cv::Mat test{cv::Mat::zeros(src.size(), src.type())};
    // cv::bitwise_and(src, src, test, laneMask);
    // image_show(test, cfg_.showImage);
  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)