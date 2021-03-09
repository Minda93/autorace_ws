#ifndef CAM_TOOLS_CAM_CALIBRATION_HPP_
#define CAM_TOOLS_CAM_CALIBRATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <vector>

namespace cam_tools
{ 
  struct ChessBoard
  {
    size_t width;
    size_t height;
    float squareSize;
    float gridWidth;
  };

  struct CalibrationParam
  {
    cv::Mat matrix;
    cv::Mat dist;
  };

  class CamCalibration : public rclcpp::Node 
  {
  public:
    explicit CamCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CamCalibration() = default;

    void show_image(const cv::Mat &src);
    void process();

  private:
    bool help(const std::vector<std::string> args);
    void parse_parameters();

    void load_image_lists();
    cv::Mat next_image();
    
    void find_corners(cv::Mat &src, std::vector<std::vector<cv::Point2f> > &imagePoints);
    void run_calibration_and_save(
      const cv::Size &imageSize,
      const std::vector<std::vector<cv::Point2f> > &imagePoints
    );
    
    bool image_calibration(
      const cv::Size &imageSize, 
      const std::vector<std::vector<cv::Point2f> > &imagePoints,
      std::vector<cv::Mat> &rvecs,
      std::vector<cv::Mat> &tvecs,
      std::vector<cv::Point3f> &newObjPoints,
      std::vector<float> &reprojErrs,  
      double &totalAvgErr
    );

    void calc_board_corner_pos(std::vector<cv::Point3f> &corners);

    double compute_reprojection_errors( 
      const std::vector<std::vector<cv::Point3f> > &objectPoints,
      const std::vector<std::vector<cv::Point2f> > &imagePoints,
      const std::vector<cv::Mat> &rvecs, 
      const std::vector<cv::Mat> &tvecs,
      std::vector<float> &perViewErrors
    );

    void save_param();
  
  private:
    std::string imagesPath_;
    std::vector<std::string> imageList_;
    
    size_t atImageList_;

    CalibrationParam calibrationParam_;
    ChessBoard chessBoard_;
  }; // class CamCalibration
} // namespace cam_tools

#endif // CAM_TOOLS_CAM_CALIBRATION_HPP_