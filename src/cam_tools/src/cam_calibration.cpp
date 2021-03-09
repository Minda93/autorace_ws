#include "cam_tools/cam_calibration.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>

namespace cam_tools
{
  CamCalibration::CamCalibration(const rclcpp::NodeOptions &options)
    : Node("cam_calibration", options),
    atImageList_{0}
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    if(help(options.arguments()))
    {
      exit(0);
    }

    parse_parameters();
    load_image_lists();
    process();
  }

  bool CamCalibration::help(const std::vector<std::string> args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
      std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;

      ss << "Usage: showimage [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "camera calibration" << std::endl;
      ss << "Example: ros2 run cam_tools cam_calibration --ros-args -p folder:=src_folder";
      ss << std::endl << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\tDisplay this help message and exit";
      ss << std::endl << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  images_path\tpath of images of the chess board. Default value is null";
      ss << std::endl;

      std::cout << ss.str();
      return true;
    }
    return false;
  }

  void CamCalibration::parse_parameters()
  {
    // Parse 'folderPath' parameter
    this->declare_parameter("images_path", "");
    
    this->declare_parameter("chessboard_width", 6);
    this->declare_parameter("chessboard_height", 5);
    this->declare_parameter("chessboard_squareSize", 1.0);

    this->declare_parameter("calibration_matrix", std::vector<double>{});
    this->declare_parameter("calibration_dist", std::vector<double>{});

    get_parameter_or<std::string>(
      "images_path", imagesPath_, "");

    get_parameter_or<size_t>(
      "chessboard_width", chessBoard_.width, 6);
    
    get_parameter_or<size_t>(
      "chessboard_height", chessBoard_.height, 5);

    get_parameter_or<float>(
      "chessboard_squareSize", chessBoard_.squareSize, 1.0);

    chessBoard_.gridWidth = chessBoard_.squareSize * (chessBoard_.width - 1);

    std::vector<double> matrix;
    std::vector<double> dist;

    get_parameter_or<std::vector<double> >(
      "calibration_matrix", matrix, std::vector<double>{});

    get_parameter_or<std::vector<double> >(
      "calibration_dist", dist, std::vector<double>{});

    if(!matrix.empty() && !dist.empty())
    {
      int idx{0};
      calibrationParam_.matrix = cv::Mat::zeros(3, 3, CV_64F);
      for(int y{0}; y < calibrationParam_.matrix.rows; ++y)
      {
        double *Mi{calibrationParam_.matrix.ptr<double>(y)};
        for(int x{0}; x < calibrationParam_.matrix.cols; ++x){
          Mi[x] = matrix[idx++];
        }
      }

      calibrationParam_.dist = cv::Mat::zeros(dist.size(), 1, CV_64F);
      for(int y{0}; y < chessBoard_.gridWidth; ++y)
      {
        *calibrationParam_.dist.ptr<double>(y) = dist[y];
      }
    }

    std::cout << "init \n";
    std::cout << "matrix : "<< calibrationParam_.matrix << "\n";
    std::cout << "dist : "<< calibrationParam_.dist << "\n";
    std::flush(std::cout);
  }

  void CamCalibration::load_image_lists()
  {
    std::ifstream imagesPathFile;

    imagesPathFile.open(imagesPath_);
    if(!imagesPathFile)
    {
      std::cout << "not found file\n";
      std::flush(std::cout);
      rclcpp::shutdown();
    }

    std::string imgsFolder{imagesPath_.substr(0, imagesPath_.find("."))};
    std::string imgName{""};
    while(imagesPathFile >> imgName)
    {
      imageList_.push_back(imgsFolder+"/"+imgName);
    }

    imagesPathFile.close();
  }

  cv::Mat CamCalibration::next_image()
  {
    cv::Mat dst;
    if(atImageList_ < imageList_.size())
    {
      dst = cv::imread(imageList_[atImageList_++], cv::IMREAD_COLOR);
    }

    return dst;
  }

  void CamCalibration::find_corners(cv::Mat &src, std::vector<std::vector<cv::Point2f> > &imagePoints)
  {
    bool found{false};
    std::vector<cv::Point2f> corners;
    // int chessBoardFlags{cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE};

    found = cv::findChessboardCorners(
      src, 
      cv::Size(chessBoard_.height, chessBoard_.width),
      corners);

    if(found && (corners.size() == chessBoard_.width*chessBoard_.height))
    {
      cv::Mat viewGray;
      cvtColor(src, viewGray, cv::COLOR_BGR2GRAY);
      cornerSubPix( viewGray, corners, cv::Size(5, 5),
          cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1 ));

      imagePoints.push_back(corners);

      cv::drawChessboardCorners(
        src,
        cv::Size(chessBoard_.height, chessBoard_.width), 
        cv::Mat(corners), 
        found);
    }
  }

  void CamCalibration::run_calibration_and_save(
    const cv::Size &imageSize, const std::vector<std::vector<cv::Point2f> > &imagePoints)
  {
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr{0};
    std::vector<cv::Point3f> newObjPoints;

    bool ok{false};

    ok = image_calibration(
      imageSize,
      imagePoints,
      rvecs,
      tvecs,
      newObjPoints,
      reprojErrs,
      totalAvgErr 
    );

    // output param
    std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
              << ". avg re projection error = " 
              << totalAvgErr << "\n";
    std::cout << "matrix : "<< calibrationParam_.matrix << "\n";
    std::cout << "dist : "<< calibrationParam_.dist << "\n";
    std::flush(std::cout);

    if(ok){
      save_param();
    }
  }

  bool CamCalibration::image_calibration(
    const cv::Size &imageSize, 
    const std::vector<std::vector<cv::Point2f> > &imagePoints,
    std::vector<cv::Mat> &rvecs,
    std::vector<cv::Mat> &tvecs,
    std::vector<cv::Point3f> &newObjPoints,
    std::vector<float> &reprojErrs,  
    double &totalAvgErr)
  {
    calibrationParam_.matrix = cv::Mat::eye(3, 3, CV_64F);
    calibrationParam_.dist = cv::Mat::zeros(8, 1, CV_64F);

    // 3D corners
    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calc_board_corner_pos(objectPoints[0]);
    objectPoints[0][chessBoard_.width - 1].x = objectPoints[0][0].x + chessBoard_.gridWidth;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    
    // //Find intrinsic and extrinsic camera parameters
    double rms{0};
    int iFixedPoint{-1};
    int flag{0};

    // method 1
    // flag |= cv::CALIB_FIX_K4;
    // flag |= cv::CALIB_FIX_K5;
    flag |= cv::CALIB_USE_LU;

    rms = cv::calibrateCameraRO(
      objectPoints, imagePoints, imageSize, iFixedPoint,
      calibrationParam_.matrix, calibrationParam_.dist, rvecs, tvecs, newObjPoints,
      flag);

    // objectPoints.clear();
    // objectPoints.resize(imagePoints.size(), newObjPoints);
    // totalAvgErr = compute_reprojection_errors(
    //   objectPoints, imagePoints, rvecs, tvecs, reprojErrs);
  
    bool ok{checkRange(calibrationParam_.matrix) && checkRange(calibrationParam_.dist)};
    
    // std::cout << "check \n";
    // std::cout << "objectPoints : " << objectPoints.size() << "\n";
    // std::cout << "objectPoints[0] : " << objectPoints[0].size() << "\n";
    // std::cout << "imagePoints : " << imagePoints.size() << "\n";
    // std::cout << "imagePoints[0] : " << imagePoints[0].size() << "\n";
    // std::cout << "imageSize : " << imageSize << "\n";
    // std::cout << "newObjPoints : " << newObjPoints.size() << "\n";
    std::cout << "rms : " << rms << "\n";
    std::flush(std::cout);

    return ok;
  }

  void CamCalibration::calc_board_corner_pos(std::vector<cv::Point3f> &corners)
  {
    for(int y{0}; y < chessBoard_.height; ++y){
      for(int x{0}; x < chessBoard_.width; ++x){
        corners.push_back(cv::Point3f(x*chessBoard_.squareSize, y*chessBoard_.squareSize, 0));
      }
    }
  }

  double CamCalibration::compute_reprojection_errors( 
    const std::vector<std::vector<cv::Point3f> > &objectPoints,
    const std::vector<std::vector<cv::Point2f> > &imagePoints,
    const std::vector<cv::Mat> &rvecs, 
    const std::vector<cv::Mat> &tvecs,
    std::vector<float> &perViewErrors)
  {
    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints{0};
    double totalErr{0}; 
    double err{0};
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i ){
      projectPoints(objectPoints[i], rvecs[i], tvecs[i], calibrationParam_.matrix, calibrationParam_.dist, imagePoints2);
      err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

      size_t n = objectPoints[i].size();
      perViewErrors[i] = (float) std::sqrt(err*err/n);
      totalErr        += err*err;
      totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
  }
    
  void CamCalibration::save_param()
  {
    std::vector<double> matrix;
    std::vector<double> dist;

    for(int y{0}; y < calibrationParam_.matrix.rows; ++y)
    {
      const double *Mi{calibrationParam_.matrix.ptr<double>(y)};
      for(int x{0}; x < calibrationParam_.matrix.cols; x++){
        matrix.push_back(Mi[x]);
      }
    }

    for(int y{0}; y < calibrationParam_.dist.rows; ++y)
    {
      dist.push_back(*calibrationParam_.dist.ptr<double>(y));
    }

    this->set_parameter(rclcpp::Parameter("calibration_matrix", matrix));
    this->set_parameter(rclcpp::Parameter("calibration_dist", dist));
  }
  
  void CamCalibration::show_image(const cv::Mat &src)
  {
    cv::imshow("Image View", src);
    if(cv::waitKey(0) == 27){
      rclcpp::shutdown();
    }
  }

  void CamCalibration::process()
  {
    std::vector<std::vector<cv::Point2f> > imagePoints;
    cv::Size imageSize{0,0};

    while(rclcpp::ok())
    {
      cv::Mat src{this->next_image()};

      if(src.empty())
      {
        if(!imagePoints.empty())
        {
          run_calibration_and_save(imageSize, imagePoints);
        }

        break;
      }

      imageSize = src.size();
      find_corners(src, imagePoints);
      // show_image(src);
    }

    cv::Mat dst;
    atImageList_ = 0;
    while(rclcpp::ok())
    {
      cv::Mat src{this->next_image()};

      if(src.empty())
      {
        break;
      }

      // method 1
      // cv::initUndistortRectifyMap(
      //   calibrationParam_.matrix, calibrationParam_.dist, cv::Mat(),
      //   cv::getOptimalNewCameraMatrix(
      //     calibrationParam_.matrix,
      //     calibrationParam_.dist, 
      //     src.size(), 1, src.size(), 0), 
      //   src.size(),CV_16SC2, map1, map2);
      // cv::remap(src, dst, map1, map2, cv::INTER_LINEAR);

      // method 2
      cv::undistort(src, dst, calibrationParam_.matrix, calibrationParam_.dist);
      show_image(dst);
    }

    rclcpp::shutdown();
  }

} // namespace cam_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cam_tools::CamCalibration)