#include "detect_object/detect_lane.hpp"
#include "detect_object/image.hpp"
#include "detect_object/interface/interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>
#include <functional>
#include <string>
#include <vector>

namespace detect_object
{
  DetectLane::DetectLane(const rclcpp::NodeOptions &options)
    : Image("detect_lane", options),
    yellowReliability_{0.0}, whiteReliability_{0.0}
  {
    // param
    parse_parameters();
    set_on_parameters_set_callback(
      std::bind(&DetectLane::dynamic_load_params, 
        this, std::placeholders::_1));
    
    // topic
    lane_center_pub_ = create_publisher<std_msgs::msg::Float64>("lane_center_x", 10);
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
      interface::set_num_range<int>("hsv_model.yellow.hue_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.saturation_l", 130,
      interface::set_num_range<int>("hsv_model.yellow.saturation_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.value_l", 160,
      interface::set_num_range<int>("hsv_model.yellow.value_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.hue_h", 41,
      interface::set_num_range<int>("hsv_model.yellow.hue_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.saturation_h", 255,
      interface::set_num_range<int>("hsv_model.yellow.saturation_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.yellow.value_h", 255,
      interface::set_num_range<int>("hsv_model.yellow.value_h", interface::INTEGER, 0, 255, 1));

    this->declare_parameter<int>(
      "hsv_model.white.hue_l", 0,
      interface::set_num_range<int>("hsv_model.white.hue_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.saturation_l", 0,
      interface::set_num_range<int>("hsv_model.white.saturation_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.value_l", 180,
      interface::set_num_range<int>("hsv_model.white.value_l", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.hue_h", 25,
      interface::set_num_range<int>("hsv_model.white.hue_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.saturation_h", 36,
      interface::set_num_range<int>("hsv_model.white.saturation_h", interface::INTEGER, 0, 255, 1));
    this->declare_parameter<int>(
      "hsv_model.white.value_h", 255,
      interface::set_num_range<int>("hsv_model.white.value_h", interface::INTEGER, 0, 255, 1));

    // mask_lane : none zero excepted value
    this->declare_parameter<double>("mask_lane.expected_value", 2500.0);

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
    for(size_t idx{0}; idx < params.size(); ++idx)
    {
      if(idx < 6)
      {
        cfg_.yellowHSV[idx] = params[idx].as_int();
      }else{
        cfg_.whiteHSV[idx - 6] = params[idx].as_int();
      }
    }

    get_parameter_or<double>(
      "mask_lane.expected_value",
      cfg_.expectedValue,
      2500.0);
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
      }else if(param.get_name() == "mask_lane.expected_value" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        cfg_.expectedValue = param.as_double();
        if(cfg_.expectedValue <= 0)
        {
          cfg_.expectedValue = 1;
        }
      }
    }

    return result;
  }

  void DetectLane::homography_transform_process(const cv::Mat &src, cv::Mat &dst)
  {
    if(src.empty()){
      return;
    }

    // src size(640, 480)
    cv::Point2f centerPoint{src.size()/2};
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

  void DetectLane::mask_lane(const cv::Mat &src, cv::Mat &maskYellow, cv::Mat &maskWhite)
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

  void DetectLane::cal_line_reliability(const cv::Mat &maskYellow, const cv::Mat &maskWhite)
  {
    std::vector<cv::Point2f> nonzero;
    cv::findNonZero(maskYellow, nonzero);

    yellowReliability_ = (!nonzero.empty())? (static_cast<double>(nonzero.size()) / cfg_.expectedValue) : 0.0;

    nonzero.clear();
    cv::findNonZero(maskWhite, nonzero);

    whiteReliability_ = (!nonzero.empty())? (static_cast<double>(nonzero.size()) / cfg_.expectedValue) : 0.0;
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
      // cv::rectangle(dst, window, cv::Scalar(0, 255, 0), 2);
    }

    // polyfit
    if(!points.empty())
    {
      // draw 
      // for(auto &point : points)
      //   cv::circle(dst, point, 1, cv::Scalar(255,0,255), -1);

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

    // polyfit
    if(!points.empty())
    {
      // draw 
      // for(auto &point : points)
      //   cv::circle(dst, point, 1, cv::Scalar(255,0,255), -1);
            
      // use points.y for input
      cv::Mat lane_fit_tmp{polyfit(points, 2, false)};
      if(!lane_fit_tmp.empty())
      {
        lane_fit = lane_fit_tmp;
        // Generate x and y values for plotting
        lane_fitX = generate_lane_plotting(lane_fit_tmp, src.rows);
      }
    }
  }

  void DetectLane::make_lane(cv::Mat &dst)
  {
    // param
    int datumY{150};

    // draw lane
    // if(!yellowLaneFitX_.empty())
    // {
    //   std::vector<cv::Point> points;
    //   for(size_t y{0}; y < dst.rows; ++y){
    //     points.push_back(cv::Point{yellowLaneFitX_[y], y});
    //   }
    //   cv::polylines(dst, points, false, cv::Scalar(0, 0, 255), 5);
    // }

    // if(!whiteLaneFitX_.empty())
    // {
    //   std::vector<cv::Point> points;
    //   for(size_t y{0}; y < dst.rows; ++y){
    //     points.push_back(cv::Point{whiteLaneFitX_[y], y});
    //   }
    //   cv::polylines(dst, points, false, cv::Scalar(255, 0, 0), 5);
    // }

    if(!yellowLaneFitX_.empty() || !whiteLaneFitX_.empty())
    {
      if(yellowReliability_ > 0.15 && whiteReliability_ > 0.15)
      {
        double centerX{(yellowLaneFitX_[datumY] + whiteLaneFitX_[datumY]) / 2.0};
        double normCenterX{normalize_lane_center(centerX, dst.size())};
        pub(normCenterX);
      }else if(yellowReliability_ <= 0.15 && whiteReliability_ > 0.15)
      {
        double centerX = whiteLaneFitX_[datumY] - (static_cast<double>(dst.cols) / 2.0);
        double normCenterX{normalize_lane_center(centerX, dst.size())};
        pub(normCenterX);
      }else if(yellowReliability_ > 0.15 && whiteReliability_ <= 0.15)
      {
        double centerX = yellowLaneFitX_[datumY] + (static_cast<double>(dst.cols) / 2.0);
        double normCenterX{normalize_lane_center(centerX, dst.size())};
        pub(normCenterX); 
      }
    }
  }

  cv::Mat DetectLane::polyfit(const std::vector<cv::Point2f> &points, int order, bool choose_x_input)
  {
    /* *******  polyfit  ******* */
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
    * ************************** */

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

  double DetectLane::normalize_lane_center(double centerX, const cv::Size imageSize)
  {
    double normCenterX{centerX};

    normCenterX -= (imageSize.width / 2.0);

    return normCenterX;
  }

  void DetectLane::pub(double centerX)
  {
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = centerX;

    lane_center_pub_->publish(std::move(msg));
  }

  void DetectLane::process(const cv::Mat &src)
  {
    cv::Mat birdView;
    cv::Mat maskYellow, maskWhite;

    homography_transform_process(src, birdView);
    mask_lane(birdView, maskYellow, maskWhite);
    cal_line_reliability(maskYellow, maskWhite);
    
    if(yellowReliability_ <= 0.15 || yellowLaneFitX_.empty())
    {
      sliding_window(maskYellow, "left", birdView);
    }else{
      line_fitting(maskYellow, yellowLaneFit_, yellowLaneFitX_, birdView);
    }

    if(whiteReliability_ <= 0.15 || whiteLaneFitX_.empty())
    {
      sliding_window(maskWhite, "right", birdView);
    }else{
      line_fitting(maskWhite, whiteLaneFit_, whiteLaneFitX_, birdView);
    }

    make_lane(birdView);

    // debug
    // image_show(src, cfg_.showImage, "detect_lane_ori");
    // image_show(birdView, cfg_.showImage, "bird_view");
    // image_show(maskYellow, cfg_.showImage, "mask_yellow");
    // image_show(maskWhite, cfg_.showImage, "mask_white");

    // cv::Mat laneMask;
    // cv::bitwise_or(maskYellow, maskWhite, laneMask);
    // image_show(laneMask, cfg_.showImage);
    // cv::Mat test{cv::Mat::zeros(src.size(), src.type())};
    // cv::bitwise_and(src, src, test, laneMask);
    // image_show(test, cfg_.showImage);
  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)