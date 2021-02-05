<!-- TOC -->

- [1. detect_object](#1-detect_object)
- [2. detect_lane](#2-detect_lane)
- [TODO](#todo)
- [Issue](#issue)
- [Reference](#reference)

<!-- /TOC -->

# 1. detect_object
* plugins
  * detect_lane

# 2. detect_lane
  * method
    * Perspective Transform
      * param 
      ```yaml
        # Perspective Transform param
        # src size(640, 480)
        bottom_x: 330.0
        bottom_y: 240.0
        top_x: 70.0
        top_y: -40.0
        
        # maybe add
        # dst size (250, 300)
        # points : [50, 0], [200, 0], [200, 300], [50, 300]
      ```
      
    * Lane-line pixel identification
      * param
      ``` yaml
        hsv_model:
        white:
          hue_h: 25
          hue_l: 0
          saturation_h: 36
          saturation_l: 0
          value_h: 255
          value_l: 180
        yellow:
          hue_h: 41
          hue_l: 27
          saturation_h: 255
          saturation_l: 130
          value_h: 255
          value_l: 160
      ```
    
# TODO  
* detect_lane
  * param slider bar(~~int~~, ~~double~~, int_array, double_array)
    - [x] ~~for detect_lane hsv bar (yellow, white)~~
    - [x] callback function
  - [x] Perspective Transform
  - [x] Lane-line pixel identification

# Issue  
* rqt_gui dynamic_reconfigure has no implementation of an array
  * [dynamic_reconfigure issue 67](https://github.com/ros/dynamic_reconfigure/issues/67)
  
* rqt_reconfigure is NOT supported group (for ros2)
  * [rqt_reconfigure issues 69](https://github.com/ros-visualization/rqt_reconfigure/issues/69)

* ros2 get group parameters method
  * now use method
    ```cpp
      std::vector<std::string> param_names = {
        "hsv_model.yellow.hue_l",
        "hsv_model.yellow.saturation_l", ...
      }
      std::vector<rclcpp::Parameter> params = get_parameters(param_names);
    ```
  * CANNOT use group to get below parameters
    ```cpp
      std::vector<int> params = get_parameters("hsv_model");
    ```

# Reference  
* [画像の幾何変換](http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html)