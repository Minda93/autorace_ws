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
    lossLane_ = true;
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
      {50, 0},
      {200, 0},
      {200, 300},
      {50, 300}
    };

    // dst (1000, 600)
    // cv::Point2f dstVertices[4]{
    //   {200, 0},
    //   {800, 0},
    //   {800, 600},
    //   {200, 600}
    // };

    // draw line for bird view
    // for(int idx{0}; idx < 4; ++idx)
    // {
    //   cv::line(src, srcVertices[idx], srcVertices[(idx+1)%4], cv::Scalar( 255, 0, 0 ), 1, cv::LINE_8);
    // }

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

  void DetectLane::sliding_window(const cv::Mat &src, const std::string &left_or_right, cv::Mat &dst)
  {
    if(src.empty())
    {
      return;
    }
    
    // param
    std::vector<cv::Point2f> points;
    cv::Size imgSize{src.size()};
    int nWindows{20}; // Choose the number of sliding windows
    int margin{50}; // Set the width of the windows +/- margin
    int minpix{50}; // Set minimum number of pixels found to recenter window

    // histogram
    cv::Mat hist; // hist size = image.width * 1
    cv::reduce(src(cv::Rect{0, imgSize.height/2, imgSize.width, imgSize.height/2}),
      hist, 0, cv::REDUCE_SUM, CV_32FC1);

    // find lane_base
    int middlePoint{imgSize.width / 2};
    int currX{0};

    if(left_or_right == "left"){
      auto lane_base{std::max_element(hist.begin<float>(), hist.end<float>()) - middlePoint};
      currX = std::distance(hist.begin<float>(), lane_base);
    }else if(left_or_right == "right"){
      auto lane_base{std::max_element(hist.begin<float>() + middlePoint, hist.end<float>())};
      currX = std::distance(hist.begin<float>(), lane_base);
    }

    // Extract line pixel positions for window
    int winHeight{imgSize.height / nWindows};
    for(int idx{0}; idx < nWindows; ++idx){
      // create window
      int winX{currX - margin};
      int winY{imgSize.height - (idx+1)*winHeight};
      int winWidth{margin+margin};
      
      // limit range
      if(winX < 0)
      {
        winX = 0;
      }
      if(winY < 0)
      {
        winY = 0;
      }
      if(winX + winWidth >= imgSize.width)
      {
        winX = imgSize.width - winWidth - 1;
      }      

      // Extract line pixel positions
      cv::Rect window{winX, winY, margin+margin, winHeight};
      cv::Mat roi{src(window)};
      std::vector<cv::Point2f> locations;
      cv::findNonZero(roi, locations);

      float avgX{0.0f};
      for(auto &point : locations)
      {
        avgX += (window.x + point.x);
        point.y += winY;
        points.push_back(point);
      }
      avgX = (locations.empty()) ? currX : (avgX / locations.size());

      // update currX
      if(locations.size() > minpix)
      {
        currX = static_cast<int>(avgX);
      }

      // draw rect
      // cv::rectangle(dst, window, (0, 255, 0), 2);
    }

    // polyfit
    if(!points.empty())
    {
      // use points.y for input
      cv::Mat lane_fit{polyfit(points, 2, false)};
      if(!lane_fit.empty())
      {
        if(left_or_right == "left")
        {
          yellowLaneFit_ = lane_fit;
          // Generate x and y values for plotting
          yellowLaneFitX_ = generate_lane_plotting(lane_fit, imgSize.height);
        }else if(left_or_right == "right")
        {
          whiteLaneFit_ = lane_fit;
          // Generate x and y values for plotting
          whiteLaneFitX_ = generate_lane_plotting(lane_fit, imgSize.height);
        }
        lossLane_ = false;
      }
    }
  }

  void DetectLane::line_fitting(const cv::Mat &src, cv::Mat &lane_fit, std::vector<float> &lane_fitX, cv::Mat &dst)
  {
    if(src.empty() || lane_fit.empty())
    {
      return;
    }

    // param
    int margin{100};
    std::vector<cv::Point2f> points;

    // Extract line pixel positions
    std::vector<cv::Point2f> nonzero;
    cv::findNonZero(src, nonzero);
    const float *lane_fitPtr = lane_fit.ptr<float>(0);
    for(int idx{0}; idx < nonzero.size(); idx++)
    {
      int x = lane_fitPtr[2] * nonzero[idx].y * nonzero[idx].y + lane_fitPtr[1] * nonzero[idx].y + lane_fitPtr[0];
      if(nonzero[idx].x > (x - margin) || nonzero[idx].x > (x + margin))
      {
        points.push_back(nonzero[idx]);
      }
    }

    // std::cout << points.size() << "\n";
    // std::flush(std::cout);

    // polyfit
    if(!points.empty())
    {      
      // use points.y for input
      cv::Mat lane_fit_tmp{polyfit(points, 2, false)};
      if(!lane_fit_tmp.empty())
      {
        lane_fit = lane_fit_tmp;
        // Generate x and y values for plotting
        lane_fitX = generate_lane_plotting(lane_fit_tmp, src.rows);
      }else{
        lossLane_ = true;
      }
    }else{
      lossLane_ = true;
    }
  }

  cv::Mat DetectLane::polyfit(const std::vector<cv::Point2f> &points, int order, bool choose_x_input)
  {
    /* *******  polyfit  ******* /
    /*
    * Xp = y
    * (X^T*X)p = X^T*y
    * p = (X^T*X)^-1*X^T*y
    * (1x3) = (3*n*n*3)*(3*n)*(n*1)
    * The polynomial fitting function is a polynomial power function
    * f(x)=a0+a1*x+a2*x^2+a3*x^3+......+an*x^n
    * 
    * Overdetermined matrix X = 
    *   1 x1 x1^2 ... ... x1^n
    *   1 x2 x2^2 ... ... x2^n
    *   1 x3 x3^2 ... ... x3^n
    *     ... ... ... ...
    *     ... ... ... ...
    *   1 xm xm^2 ... ... xm^n
    * ***************************/

    cv::Mat y{points.size(), 1, CV_32F};
    cv::Mat X{points.size(), order+1, CV_32F};
    cv::Mat p;

    if(choose_x_input) // choose x for function input
    {
      for(int row{0}; row < X.rows; ++row)
      {
        float *xPtr = X.ptr<float>(row);
        for(int col{0}; col < X.cols; ++col)
        {
          xPtr[col] = pow(points[row].x, col);
        }
        y.at<float>(row) = points[row].y;
      }
    }else{ // choose y for function input
      for(int row{0}; row < X.rows; ++row)
      {
        float *xPtr = X.ptr<float>(row);
        for(int col{0}; col < X.cols; ++col)
        {
          xPtr[col] = pow(points[row].y, col);
        }
        y.at<float>(row) = points[row].x;
      }
    }
    
    return (X.t()*X).inv()*X.t()*y;
  }

  std::vector<float> DetectLane::generate_lane_plotting(const cv::Mat &lane_fit, size_t size)
  {
    std::vector<float> lane_fit_vec(size);

    const float *xPtr = lane_fit.ptr<float>(0);
    for(int idx{0}; idx < size; ++idx)
    {
      lane_fit_vec[idx] = xPtr[2] * idx * idx + xPtr[1] * idx + xPtr[0];
    }

    return lane_fit_vec;
  }

  void DetectLane::process()
  {
    cv::Mat birdView;
    cv::Mat maskYellow, maskWhite;

    homography_transform_process(src_, birdView);
    mask_lane(birdView, maskYellow, maskWhite);
    if(lossLane_)
    {
      sliding_window(maskYellow, "left", birdView);
      sliding_window(maskWhite, "right", birdView);
    }else{
      line_fitting(maskYellow, yellowLaneFit_, yellowLaneFitX_, birdView);
      line_fitting(maskWhite, whiteLaneFit_, whiteLaneFitX_, birdView);
    }
      
    if(!yellowLaneFitX_.empty() || !whiteLaneFitX_.empty())
    {
      // std::vector<cv::Point> points;
      // for(int y{0}; y < birdView.rows; ++y){
      //   points.push_back(cv::Point{yellowLaneFitX_[y], y});
      // }
      // cv::polylines(birdView, points, false, (0, 0, 255), 5);

      std::cout << "yellow : " << yellowLaneFitX_[150] << "\n";
      std::cout << "white : " << whiteLaneFitX_[150] << "\n";
      std::cout << "center : " << (yellowLaneFitX_[150] + whiteLaneFitX_[150]) / 2 << "\n";
      std::flush(std::cout);
    }

    // image_show(src_, cfg_.showImage);
    image_show(birdView, cfg_.showImage);
    // image_show(maskYellow, cfg_.showImage);
    
    // cv::Mat laneMask;
    // cv::bitwise_or(maskYellow, maskWhite, laneMask);
    // image_show(laneMask, cfg_.showImage);
    // cv::Mat test{cv::Mat::zeros(src.size(), src.type())};
    // cv::bitwise_and(src, src, test, laneMask);
    // image_show(test, cfg_.showImage);
  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)