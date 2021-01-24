#include "cam_tools/cam_calibration.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>

namespace cam_tools
{
  CamCalibration::CamCalibration(const rclcpp::NodeOptions &options)
    : Node("cam_calibration", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    if(help(options.arguments()))
    {
      exit(0);
    }

    parse_parameters();
    load_image_lists();
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
    imagesPath_ = this->declare_parameter("images_path", "");
  }

  void CamCalibration::load_image_lists()
  {
    std::ifstream imagesPathFile;

    imagesPathFile.open(imagesPath_);
    if(!imagesPathFile)
    {
      std::cout << "not found file\n";
      std::flush(std::cout);
      exit(1);
    }

    std::string imgsFolder{imagesPath_.substr(0, imagesPath_.find("."))};
    std::string imgName{""};
    while(imagesPathFile >> imgName)
    {
      imageLists_.push_back(imgsFolder+"/"+imgName);
    }

    imagesPathFile.close();
  }

  // cv::Mat CamCalibration::image_calibration(cv::Mat &src)
  // {

  // }

} // namespace cam_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cam_tools::CamCalibration)